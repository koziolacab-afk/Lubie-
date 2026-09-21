#include <ModbusMaster.h>

#include <SuplaDevice.h>
#include <supla/network/esp_wifi.h>
#include <supla/storage/littlefs_config.h>
#include <supla/network/esp_web_server.h>

#include <supla/network/html/device_info.h>
#include <supla/network/html/protocol_parameters.h>
#include <supla/network/html/wifi_parameters.h>

#include <supla/sensor/virtual_thermometer.h>
#include <supla/sensor/general_purpose_measurement.h>
#include <supla/sensor/virtual_binary.h>
#include <supla/control/virtual_relay.h>
#include <supla/control/relay.h>
#include <supla/control/button.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>

#include "esp_ota_ops.h"
#include "esp_err.h"


// =====================================================
// WERSJA
// =====================================================

#define FW_VERSION "1.0.14"


// =====================================================
// OTA
// =====================================================

const char *OTA_URL =
  "https://github.com/koziolacab-afk/Lubie-/releases/download/ota/firmware.bin";

const unsigned long OTA_VERIFY_DELAY = 60000;
const unsigned long OTA_ROLLBACK_TIMEOUT = 180000;

bool otaInProgress = false;
bool otaPendingVerification = false;

unsigned long otaVerifyStart = 0;
unsigned long lastOtaVerifyAttempt = 0;

int otaLastProgress = -1;


// =====================================================
// ROLLBACK
//
// Arduino ESP32 normalnie moze automatycznie
// zatwierdzic nowy firmware bardzo szybko.
//
// Odkladamy walidacje i robimy ja sami
// po 60 sekundach poprawnej pracy.
// =====================================================

extern "C" bool verifyRollbackLater() {
  return true;
}


// =====================================================
// RS485 / MODBUS
// =====================================================

#define RS485_TX 17
#define RS485_RX 18

#define MODBUS_BAUD 9600

const uint8_t PUMP_COUNT = 3;
const uint8_t PUMP_SLAVE_IDS[PUMP_COUNT] = {1, 2, 3};
const char *const PUMP_NAMES[PUMP_COUNT] = {"CAHV 1", "CAHV 2", "QAHV"};

HardwareSerial RS485(1);
ModbusMaster node;
uint8_t lastModbusError[PUMP_COUNT] = {};

const uint8_t TANK_COUNT = 5;
const uint8_t DS_SENSOR_COUNT = 8;
const uint8_t DS_DATA_PIN = 4;
const uint8_t RELAY_COUNT = 5;
const uint8_t RELAY_PINS[RELAY_COUNT] = {1, 2, 41, 42, 45};
const uint8_t BOOT_BUTTON_PIN = 0;
const uint32_t BOOT_BUTTON_HOLD_MS = 5000;
const double TANK_VOLUME_LITERS = 1000.0;
const double CWU_COLD_REFERENCE_C = 10.0;
const unsigned long DS_READ_INTERVAL = 15000;
const unsigned long DS_CONVERSION_TIME = 800;

OneWire oneWire(DS_DATA_PIN);
DallasTemperature dsBus(&oneWire);
DeviceAddress dsAddress[DS_SENSOR_COUNT] = {};
bool dsBound[DS_SENSOR_COUNT] = {};
Supla::Sensor::VirtualThermometer *dsChannel[DS_SENSOR_COUNT] = {};
Supla::Sensor::GeneralPurposeMeasurement *cwuEnergy = nullptr;
Supla::Control::Relay *physicalRelay[RELAY_COUNT] = {};
bool dsConversionPending = false;
unsigned long dsConversionStarted = 0;
unsigned long lastDsDiscovery = 0;

const char *const DS_KEYS[DS_SENSOR_COUNT] = {
  "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8"
};
const char *const DS_CAPTIONS[DS_SENSOR_COUNT] = {
  "CWU zbiornik 1 - srodek", "CWU zbiornik 2 - srodek",
  "CWU zbiornik 3 - srodek", "CWU zbiornik 4 - srodek",
  "CWU zbiornik 5 - srodek", "Kociol olejowy - temperatura",
  "Kociol olejowy - zasilanie CO", "Kociol olejowy - zasilanie CWU"
};
const char *const RELAY_CAPTIONS[RELAY_COUNT] = {
  "Kociol olejowy - zalaczanie", "Kociol olejowy - pompa CO",
  "Kociol olejowy - pompa CWU", "CAHV 1 - tryb reczny",
  "CAHV 2 - tryb reczny"
};


// =====================================================
// SUPLA
// =====================================================

Supla::ESPWifi wifi;
// The default 1024-byte config buffer cannot hold settings for all channels.
Supla::LittleFsConfig configSupla(16384);
Supla::EspWebServer suplaServer;


// =====================================================
// KANALY SUPLA
// =====================================================

// 0
Supla::Sensor::VirtualThermometer *outdoorTemp;

// 1
Supla::Sensor::VirtualThermometer *flowTemp;

// 2
Supla::Sensor::VirtualThermometer *returnTemp;

// 3
Supla::Sensor::GeneralPurposeMeasurement *hpFrequency;

// 4
Supla::Sensor::VirtualBinary *hpRunning;

// 5
Supla::Sensor::GeneralPurposeMeasurement *faultCode;

// 6-8: zachowane kanaly SUPLA, bez odpytywania Modbus.
Supla::Sensor::GeneralPurposeMeasurement *firmwareA1M;
Supla::Sensor::GeneralPurposeMeasurement *modbusCounter;
Supla::Sensor::GeneralPurposeMeasurement *systemType;

// 9
Supla::Control::VirtualRelay *otaTrigger;

// 10
Supla::Sensor::GeneralPurposeMeasurement *defrostStatus;

// 11
Supla::Sensor::GeneralPurposeMeasurement *operatingMode;

// 12
Supla::Sensor::VirtualBinary *modbusStatus;

struct PumpChannels {
  Supla::Sensor::VirtualThermometer *outdoor;
  Supla::Sensor::VirtualThermometer *outlet;
  Supla::Sensor::VirtualThermometer *inlet;
  Supla::Sensor::GeneralPurposeMeasurement *frequency;
  Supla::Sensor::VirtualBinary *running;
  Supla::Sensor::GeneralPurposeMeasurement *fault;
  Supla::Sensor::GeneralPurposeMeasurement *firmware;
  Supla::Sensor::GeneralPurposeMeasurement *counter;
  Supla::Sensor::GeneralPurposeMeasurement *systemType;
  Supla::Sensor::GeneralPurposeMeasurement *defrost;
  Supla::Sensor::GeneralPurposeMeasurement *mode;
  Supla::Sensor::VirtualBinary *online;
  Supla::Sensor::VirtualThermometer *settingWater;
  Supla::Sensor::GeneralPurposeMeasurement *systemOn;
  Supla::Sensor::GeneralPurposeMeasurement *runtimeHours;
  Supla::Sensor::VirtualThermometer *thermoOff;
  Supla::Sensor::GeneralPurposeMeasurement *temperatureDelta;
  Supla::Sensor::GeneralPurposeMeasurement *compressorStarts;
  Supla::Sensor::GeneralPurposeMeasurement *compressorRuntime;
};

PumpChannels pumpChannels[PUMP_COUNT] = {};


// =====================================================
// MODBUS - SCHEDULER
// =====================================================

enum RegisterType {
  REG_OUTDOOR,
  REG_FLOW,
  REG_RETURN,
  REG_FREQUENCY,
  REG_HP_RUNNING,
  REG_DEFROST,
  REG_OPERATING_MODE,
  REG_FAULT,
  REG_FLOW_SETPOINT,
  REG_SYSTEM_ON,
  REG_RUNTIME_HOURS,
  REG_RUNTIME_HUNDREDS,
  REG_THERMO_OFF
};

void applyRegisterValue(uint8_t pump, uint8_t type, uint16_t raw);

struct ModbusPollItem {
  uint16_t address;
  unsigned long interval;
  unsigned long lastPoll[PUMP_COUNT];
  RegisterType type;
  uint8_t pumpMask;
};

const uint8_t ALL_PUMPS = 0x07;
const uint8_t CAHV_PUMPS = 0x03;
const uint8_t QAHV_PUMP = 0x04;

// Adresy sa wspolne, ale kazdy wpis ma jawna maske modeli i osobny czas
// ostatniego odczytu dla slave 1 (CAHV 1), 2 (CAHV 2) i 3 (QAHV).
ModbusPollItem pollItems[] = {

  {99,  5000, {0}, REG_OUTDOOR, ALL_PUMPS},
  {101, 5000, {0}, REG_FLOW, ALL_PUMPS},
  {103, 5000, {0}, REG_RETURN, ALL_PUMPS},
  {73,  5000, {0}, REG_FREQUENCY, ALL_PUMPS},
  {127, 5000, {0}, REG_HP_RUNNING, ALL_PUMPS},
  {67,  5000, {0}, REG_DEFROST, ALL_PUMPS},
  {26,  5000, {0}, REG_OPERATING_MODE, ALL_PUMPS},
  {9,   5000, {0}, REG_FAULT, ALL_PUMPS},
  {85,  5000, {0}, REG_FLOW_SETPOINT, ALL_PUMPS},
  {25,  5000, {0}, REG_SYSTEM_ON, ALL_PUMPS},

  {136, 60000, {0}, REG_RUNTIME_HOURS, CAHV_PUMPS},
  {137, 60000, {0}, REG_RUNTIME_HUNDREDS, CAHV_PUMPS},
  {30, 60000, {0}, REG_THERMO_OFF, QAHV_PUMP}
};


const uint8_t POLL_ITEM_COUNT =
  sizeof(pollItems) / sizeof(pollItems[0]);


// Minimalna przerwa pomiedzy poprawnymi transakcjami.
const unsigned long MODBUS_MIN_GAP = 250;


// Po utracie komunikacji probujemy tylko jednego
// rejestru co 10 sekund.
const unsigned long MODBUS_OFFLINE_RETRY = 10000;


uint8_t pollCursor[PUMP_COUNT] = {};
uint8_t pumpCursor = 0;

unsigned long lastModbusTransaction = 0;
struct PumpState {
  unsigned long nextProbe;
  uint8_t failures;
  bool online;
  bool stateKnown;
  uint16_t defrost;
  uint16_t operatingMode;
  uint16_t fault;
  bool haveDefrost;
  bool haveOperatingMode;
  bool haveFault;
  uint16_t runtimeRemainder;
  uint16_t runtimeHundreds;
  bool haveRuntimeRemainder;
  bool haveRuntimeHundreds;
  int16_t flowRaw;
  int16_t returnRaw;
  bool haveFlow;
  bool haveReturn;
  bool frequencyKnown;
  bool compressorRunning;
  uint32_t compressorStarts;
  uint64_t compressorRuntimeMs;
  uint64_t savedCompressorRuntimeMs;
  uint64_t publishedRuntimeHundredths;
  uint32_t lastRuntimeTick;
  uint32_t lastFrequencySample;
  uint32_t lastRuntimeSaveFailure;
  bool runtimeSaveFailed;
  int32_t lastDefrostDescription;
  int32_t lastOperatingDescription;
  int32_t lastFaultDescription;
};

PumpState pumpState[PUMP_COUNT] = {};

const char *const START_COUNT_KEYS[PUMP_COUNT] = {"s1", "s2", "s3"};
const char *const RUNTIME_KEYS[PUMP_COUNT] = {"h1", "h2", "h3"};
const uint32_t RUNTIME_SAVE_INTERVAL_MS = 300000;
const uint32_t FREQUENCY_MAX_AGE_MS = 30000;

void loadCompressorStartCounts() {
  Preferences preferences;
  if (!preferences.begin("melco-starts", true)) {
    Serial.println("Nie mozna odczytac licznikow startow z NVS");
    return;
  }
  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    pumpState[pump].compressorStarts =
      preferences.getUInt(START_COUNT_KEYS[pump], 0);
    pumpChannels[pump].compressorStarts->setValue(
      pumpState[pump].compressorStarts);
  }
  preferences.end();
}

void saveCompressorStartCount(uint8_t pump) {
  Preferences preferences;
  if (!preferences.begin("melco-starts", false)) {
    Serial.println("Nie mozna zapisac licznika startow do NVS");
    return;
  }
  if (preferences.putUInt(START_COUNT_KEYS[pump],
                          pumpState[pump].compressorStarts) != sizeof(uint32_t)) {
    Serial.println("Blad zapisu licznika startow do NVS");
  }
  preferences.end();
}

void loadCompressorRuntime() {
  Preferences preferences;
  bool opened = preferences.begin("melco-hours", true);
  if (!opened) Serial.println("Nie mozna odczytac czasu sprezarek z NVS");
  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    PumpState &state = pumpState[pump];
    state.compressorRuntimeMs = opened ?
      preferences.getULong64(RUNTIME_KEYS[pump], 0) : 0;
    state.savedCompressorRuntimeMs = state.compressorRuntimeMs;
    state.publishedRuntimeHundredths = state.compressorRuntimeMs / 36000ULL;
    state.lastRuntimeTick = millis();
    pumpChannels[pump].compressorRuntime->setValue(
      state.compressorRuntimeMs / 3600000.0);
  }
  if (opened) preferences.end();
}

void saveCompressorRuntime(uint8_t pump) {
  PumpState &state = pumpState[pump];
  if (state.compressorRuntimeMs == state.savedCompressorRuntimeMs) return;
  if (state.runtimeSaveFailed &&
      millis() - state.lastRuntimeSaveFailure < 60000) return;
  Preferences preferences;
  if (!preferences.begin("melco-hours", false)) {
    Serial.println("Nie mozna zapisac czasu sprezarek do NVS");
    state.runtimeSaveFailed = true;
    state.lastRuntimeSaveFailure = millis();
    return;
  }
  if (preferences.putULong64(RUNTIME_KEYS[pump],
                             state.compressorRuntimeMs) == sizeof(uint64_t)) {
    state.savedCompressorRuntimeMs = state.compressorRuntimeMs;
    state.runtimeSaveFailed = false;
  } else {
    Serial.println("Blad zapisu czasu sprezarek do NVS");
    state.runtimeSaveFailed = true;
    state.lastRuntimeSaveFailure = millis();
  }
  preferences.end();
}

void updateCompressorRuntime(uint8_t pump) {
  PumpState &state = pumpState[pump];
  uint32_t now = millis();
  uint32_t elapsed = now - state.lastRuntimeTick;
  state.lastRuntimeTick = now;

  if (state.online && state.frequencyKnown && state.compressorRunning &&
      now - state.lastFrequencySample <= FREQUENCY_MAX_AGE_MS) {
    state.compressorRuntimeMs += elapsed;
  }

  uint64_t hundredths = state.compressorRuntimeMs / 36000ULL;
  if (hundredths != state.publishedRuntimeHundredths) {
    state.publishedRuntimeHundredths = hundredths;
    pumpChannels[pump].compressorRuntime->setValue(
      state.compressorRuntimeMs / 3600000.0);
  }
  if (state.compressorRuntimeMs - state.savedCompressorRuntimeMs >=
      RUNTIME_SAVE_INTERVAL_MS) {
    saveCompressorRuntime(pump);
  }
}


// =====================================================
// STATUSY DO HACKU GPM
// =====================================================

// =====================================================
// SUPLA CONFIG
// =====================================================

bool suplaWasReady = false;
bool historyConfigured = false;
unsigned long lastHistoryCheck = 0;

unsigned long suplaReadySince = 0;

const unsigned long SUPLA_CONFIG_SETTLE_TIME = 5000;


// =====================================================
// DIAGNOSTYKA
// =====================================================

int lastWifiStatus = -100;
int lastSuplaStatus = -100;

unsigned long lastDiagnosticsCheck = 0;


// =====================================================
// POMOCNICZE
// =====================================================

bool timeReached(
  unsigned long now,
  unsigned long target
) {

  return (int32_t)(now - target) >= 0;
}


// =====================================================
// MODBUS IDLE
//
// ModbusMaster moze czekac 2 sekundy na timeout.
// W tym czasie MUSIMY obslugiwac SUPLA.
// =====================================================

void modbusIdle() {

  SuplaDevice.iterate();

  yield();
}


// =====================================================
// MODBUS READ
// =====================================================

bool readHR(
  uint8_t pump,
  uint16_t address,
  uint16_t &value
) {

  node.begin(PUMP_SLAVE_IDS[pump], RS485);

  uint8_t result =
    node.readHoldingRegisters(
      address,
      1
    );


  if (result ==
      node.ku8MBSuccess) {

    value =
      node.getResponseBuffer(0);

    return true;
  }


  lastModbusError[pump] = result;

  return false;
}


// =====================================================
// MODBUS ONLINE / OFFLINE
// =====================================================

void setModbusOnline(uint8_t pump, bool online) {

  PumpState &state = pumpState[pump];
  if (state.stateKnown && state.online == online) {

    return;
  }

  updateCompressorRuntime(pump);

  state.stateKnown = true;
  state.online = online;

  if (!online) {
    state.haveFlow = false;
    state.haveReturn = false;
    state.frequencyKnown = false;
    state.compressorRunning = false;
    saveCompressorRuntime(pump);
  }


  if (online) {

    pumpChannels[pump].online->set();

    Serial.println();
    Serial.println(
      "MODBUS: ONLINE"
    );
  }
  else {

    pumpChannels[pump].online->clear();

    Serial.println();
    Serial.println(
      "MODBUS: OFFLINE"
    );
  }

  Serial.print("Slave: ");
  Serial.print(PUMP_SLAVE_IDS[pump]);
  Serial.print(" (");
  Serial.print(PUMP_NAMES[pump]);
  Serial.println(")");
}


// =====================================================
// TEKSTY STATUSOW
//
// Unit GPM ma ograniczona dlugosc,
// dlatego teksty sa krotkie.
// =====================================================

const char *getDefrostText(
  uint16_t value
) {

  switch (value) {

    case 0:
      return "Normal";

    case 1:
      return "Standby";

    case 2:
      return "Defrost";

    case 3:
      return "Wait restart";

    default:
      return "Unknown";
  }
}


const char *getOperatingModeText(
  uint16_t value
) {

  switch (value) {

    case 0:
      return "Stop";

    case 1:
      return "Hot water";

    case 2:
      return "Heating";

    case 3:
      return "Cooling";

    case 4:
      return "DHW contact";

    case 5:
      return "Freeze stat";

    case 6:
      return "Legionella";

    case 7:
      return "Heating Eco";

    case 8:
      return "Mode 1";

    case 9:
      return "Mode 2";

    case 10:
      return "Mode 3";

    case 11:
      return "Heat contact";

    default:
      return "Unknown";
  }
}


const char *getFaultText(
  uint16_t value
) {

  if (value == 0x8000) {

    return "OK";
  }

  if (value == 0x6999) {

    return "Comm error";
  }

  return "Fault";
}


// =====================================================
// PRZETWARZANIE REJESTROW
// =====================================================

void applyRegisterValue(
  uint8_t pump,
  uint8_t type,
  uint16_t raw
) {

  PumpChannels &channels = pumpChannels[pump];
  PumpState &state = pumpState[pump];

  switch (type) {


    // -----------------------------------------------
    // Outdoor temp
    // addr 99
    // signed / 10
    // -----------------------------------------------

    case REG_OUTDOOR: {

      double value =
        ((int16_t)raw) / 10.0;

      channels.outdoor->setValue(value);

      break;
    }


    // -----------------------------------------------
    // Flow temp
    // addr 101
    // signed / 100
    // -----------------------------------------------

    case REG_FLOW: {

      state.flowRaw = (int16_t)raw;
      state.haveFlow = true;
      double value = state.flowRaw / 100.0;

      channels.outlet->setValue(value);

      break;
    }


    // -----------------------------------------------
    // Return temp
    // addr 103
    // signed / 100
    // -----------------------------------------------

    case REG_RETURN: {

      state.returnRaw = (int16_t)raw;
      state.haveReturn = true;
      double value = state.returnRaw / 100.0;

      channels.inlet->setValue(value);

      break;
    }


    // -----------------------------------------------
    // HP Frequency
    // addr 73
    // -----------------------------------------------

    case REG_FREQUENCY: {

      updateCompressorRuntime(pump);
      channels.frequency->setValue(raw);

      if (raw <= 255) {
        bool running = raw > 0;
        if (state.frequencyKnown && !state.compressorRunning && running &&
            state.compressorStarts < UINT32_MAX) {
          state.compressorStarts++;
          channels.compressorStarts->setValue(state.compressorStarts);
          saveCompressorStartCount(pump);
        }
        state.compressorRunning = running;
        state.frequencyKnown = true;
        state.lastFrequencySample = millis();
        if (!running) saveCompressorRuntime(pump);
      } else {
        state.frequencyKnown = false;
        state.compressorRunning = false;
        saveCompressorRuntime(pump);
      }

      break;
    }


    // -----------------------------------------------
    // HP Run
    // addr 127
    // -----------------------------------------------

    case REG_HP_RUNNING: {

      if (raw == 1) {

        channels.running->set();
      }
      else {

        channels.running->clear();
      }


      break;
    }


    // -----------------------------------------------
    // Defrost
    // addr 67
    // -----------------------------------------------

    case REG_DEFROST: {

      channels.defrost->setValue(raw);

      state.defrost = raw;
      state.haveDefrost = true;


      break;
    }


    // -----------------------------------------------
    // Operating mode
    // addr 26
    // -----------------------------------------------

    case REG_OPERATING_MODE: {

      channels.mode->setValue(raw);

      state.operatingMode = raw;
      state.haveOperatingMode = true;


      break;
    }


    // -----------------------------------------------
    // Fault code
    // addr 9
    // -----------------------------------------------

    case REG_FAULT: {

      channels.fault->setValue(raw);

      state.fault = raw;
      state.haveFault = true;


      break;
    }


    case REG_FLOW_SETPOINT:
      channels.settingWater->setValue(((int16_t)raw) / 100.0);
      break;

    case REG_SYSTEM_ON:
      channels.systemOn->setValue(raw);
      break;

    case REG_RUNTIME_HOURS:
      state.runtimeRemainder = raw;
      state.haveRuntimeRemainder = true;
      break;

    case REG_RUNTIME_HUNDREDS:
      state.runtimeHundreds = raw;
      state.haveRuntimeHundreds = true;
      break;

    case REG_THERMO_OFF:
      channels.thermoOff->setValue(((int16_t)raw) / 100.0);
      break;
  }

  if ((type == REG_FLOW || type == REG_RETURN) &&
      state.haveFlow && state.haveReturn) {
    int32_t deltaRaw = (int32_t)state.flowRaw - state.returnRaw;
    channels.temperatureDelta->setValue(deltaRaw / 100.0);
  }

  if (channels.runtimeHours &&
      (type == REG_RUNTIME_HOURS || type == REG_RUNTIME_HUNDREDS) &&
      state.haveRuntimeRemainder &&
      state.haveRuntimeHundreds) {
    uint32_t hours = (uint32_t)state.runtimeHundreds * 100U +
                     state.runtimeRemainder;
    channels.runtimeHours->setValue(hours);
  }
}


// =====================================================
// MODBUS PROBE
//
// Kiedy A1M jest offline nie odpytujemy wszystkich
// rejestrow. Probujemy tylko addr 99 co 10 s.
// =====================================================

void handleOfflineModbusProbe(uint8_t pump) {
  PumpState &state = pumpState[pump];
  uint16_t raw;

  bool ok = readHR(pump, 99, raw);
  lastModbusTransaction = millis();

  if (ok) {
    state.failures = 0;
    setModbusOnline(pump, true);
    applyRegisterValue(pump, REG_OUTDOOR, raw);
    pollItems[0].lastPoll[pump] = millis();
  } else {
    setModbusOnline(pump, false);
    state.nextProbe = millis() + MODBUS_OFFLINE_RETRY;
  }
}


// =====================================================
// MODBUS SCHEDULER
//
// Jedna transakcja na raz.
// Nie robimy juz 8 timeoutow jeden po drugim.
// =====================================================

void handleModbusScheduler() {
  if (otaInProgress) return;

  unsigned long now = millis();
  if (now - lastModbusTransaction < MODBUS_MIN_GAP) return;

  // Round-robin: nieobecny slave nie blokuje pozostalych pomp.
  for (uint8_t checked = 0; checked < PUMP_COUNT; checked++) {
    uint8_t pump = (pumpCursor + checked) % PUMP_COUNT;
    PumpState &state = pumpState[pump];

    if (!state.online) {
      if (!timeReached(now, state.nextProbe)) continue;
      pumpCursor = (pump + 1) % PUMP_COUNT;
      handleOfflineModbusProbe(pump);
      return;
    }

    for (uint8_t i = 0; i < POLL_ITEM_COUNT; i++) {
      uint8_t index = (pollCursor[pump] + i) % POLL_ITEM_COUNT;
      ModbusPollItem &item = pollItems[index];
      if (!(item.pumpMask & (1U << pump))) continue;
      if (now - item.lastPoll[pump] < item.interval) continue;

      pollCursor[pump] = (index + 1) % POLL_ITEM_COUNT;
      pumpCursor = (pump + 1) % PUMP_COUNT;

      uint16_t raw;
      bool ok = readHR(pump, item.address, raw);
      item.lastPoll[pump] = millis();
      lastModbusTransaction = millis();

      if (ok) {
        state.failures = 0;
        applyRegisterValue(pump, item.type, raw);
      } else {
        if (state.failures < 255) state.failures++;
        if (state.failures >= 3) {
          Serial.print("MODBUS slave ");
          Serial.print(PUMP_SLAVE_IDS[pump]);
          Serial.print(" addr ");
          Serial.print(item.address);
          Serial.print(" error 0x");
          Serial.println(lastModbusError[pump], HEX);
          setModbusOnline(pump, false);
          state.failures = 0;
          state.nextProbe = millis() + MODBUS_OFFLINE_RETRY;
        }
      }
      return;
    }
  }
}


// =====================================================
// HISTORIA GPM
// =====================================================

void configureGpmHistory(bool notifyCloud) {
  bool changed = false;
  uint8_t enabled = 0;
  uint8_t total = 0;
  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    PumpChannels &channels = pumpChannels[pump];
    Supla::Sensor::GeneralPurposeMeasurement *measurements[] = {
      channels.frequency, channels.fault, channels.defrost,
      channels.mode, channels.systemOn, channels.runtimeHours,
      channels.temperatureDelta, channels.compressorStarts,
      channels.compressorRuntime
    };
    for (auto *measurement : measurements) {
      if (!measurement) continue;
      total++;
      if (measurement->getKeepHistory() != 1) {
        measurement->setKeepHistory(1, notifyCloud);
        changed = true;
      }
      enabled += measurement->getKeepHistory() == 1;
    }
  }
  if (cwuEnergy) {
    total++;
    if (cwuEnergy->getKeepHistory() != 1) {
      cwuEnergy->setKeepHistory(1, notifyCloud);
      changed = true;
    }
    enabled += cwuEnergy->getKeepHistory() == 1;
  }
  if (notifyCloud && (!historyConfigured || changed)) {
    Serial.print("SUPLA GPM KeepHistory local: ");
    Serial.print(enabled);
    Serial.print("/");
    Serial.println(total);
  }
}


// =====================================================
// STATUS TEXT HACK
//
// Wynik w SUPLA:
//
// 2 Defrost
// 2 Heating
// 32768 OK
// 1 ATW
//
// Aktualizujemy unit TYLKO po zmianie statusu.
// =====================================================

void updateStatusDescriptions() {

  bool ready =
    SuplaDevice.getCurrentStatus() ==
    STATUS_REGISTERED_AND_READY;


  if (!ready) {

    suplaWasReady = false;
    suplaReadySince = 0;
    historyConfigured = false;

    return;
  }


  if (!suplaWasReady) {

    suplaWasReady = true;
    historyConfigured = false;

    suplaReadySince =
      millis();


    // Pozwalamy ponownie wyslac opis
    // po reconnect.
    for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
      PumpState &state = pumpState[pump];
      state.lastDefrostDescription = -1;
      state.lastOperatingDescription = -1;
      state.lastFaultDescription = -1;
    }


    return;
  }


  // SUPLA musi miec chwile na pobranie
  // konfiguracji kanalow.
  if (
    millis() -
      suplaReadySince <
    SUPLA_CONFIG_SETTLE_TIME
  ) {

    return;
  }


  // Historia ustawiana po synchronizacji
  // konfiguracji z Cloud.
  if (!historyConfigured || millis() - lastHistoryCheck >= 60000) {
    configureGpmHistory(true);
    historyConfigured = true;
    lastHistoryCheck = millis();
  }


  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    PumpState &state = pumpState[pump];
    PumpChannels &channels = pumpChannels[pump];
    if (state.haveDefrost &&
        state.defrost != state.lastDefrostDescription) {
      channels.defrost->setUnitAfterValue(getDefrostText(state.defrost));
      state.lastDefrostDescription = state.defrost;
    }
    if (state.haveOperatingMode &&
        state.operatingMode != state.lastOperatingDescription) {
      channels.mode->setUnitAfterValue(getOperatingModeText(state.operatingMode));
      state.lastOperatingDescription = state.operatingMode;
    }
    if (state.haveFault &&
        state.fault != state.lastFaultDescription) {
      channels.fault->setUnitAfterValue(getFaultText(state.fault));
      state.lastFaultDescription = state.fault;
    }
  }
}


// =====================================================
// ROLLBACK - STAN
// =====================================================

void initRollbackState() {

  const esp_partition_t *running =
    esp_ota_get_running_partition();


  Serial.print(
    "Running partition: "
  );


  if (running) {

    Serial.println(
      running->label
    );
  }
  else {

    Serial.println(
      "unknown"
    );

    return;
  }


  esp_ota_img_states_t state;


  if (
    esp_ota_get_state_partition(
      running,
      &state
    ) == ESP_OK
  ) {

    if (
      state ==
      ESP_OTA_IMG_PENDING_VERIFY
    ) {

      otaPendingVerification =
        true;

      otaVerifyStart =
        millis();


      Serial.println();
      Serial.println(
        "=============================="
      );

      Serial.println(
        "OTA PENDING_VERIFY"
      );

      Serial.println(
        "Rollback aktywny"
      );

      Serial.println(
        "Test firmware: 60 s, rollback po 180 s bez SUPLA"
      );

      Serial.println(
        "=============================="
      );


      return;
    }
  }


  Serial.println(
    "OTA: firmware VALID"
  );
}


// =====================================================
// ROLLBACK - WALIDACJA
// =====================================================

void handleRollbackVerification() {

  if (
    !otaPendingVerification
  ) {

    return;
  }


  unsigned long now =
    millis();


  if (
    now -
      otaVerifyStart <
    OTA_VERIFY_DELAY
  ) {

    return;
  }


  if (
    now -
      lastOtaVerifyAttempt <
    5000
  ) {

    return;
  }


  lastOtaVerifyAttempt =
    now;


  bool wifiOK =
    WiFi.status() ==
    WL_CONNECTED;


  bool suplaOK =
    SuplaDevice.getCurrentStatus() ==
    STATUS_REGISTERED_AND_READY;


  Serial.print(
    "OTA VERIFY | WiFi: "
  );

  Serial.print(
    wifiOK ?
    "OK" :
    "BRAK"
  );


  Serial.print(
    " | SUPLA: "
  );

  Serial.println(
    suplaOK ?
    "OK" :
    "BRAK"
  );

  if (!suplaOK && now - otaVerifyStart >= OTA_ROLLBACK_TIMEOUT) {
    Serial.println("OTA: brak polaczenia z SUPLA, przywracam poprzedni firmware");
    esp_err_t result = esp_ota_mark_app_invalid_rollback_and_reboot();
    Serial.print("OTA ROLLBACK ERROR: ");
    Serial.println(esp_err_to_name(result));
    return;
  }


  if (
    wifiOK &&
    suplaOK
  ) {

    esp_err_t result =
      esp_ota_mark_app_valid_cancel_rollback();


    if (
      result ==
      ESP_OK
    ) {

      otaPendingVerification =
        false;


      Serial.println();
      Serial.println(
        "=============================="
      );

      Serial.println(
        "OTA FIRMWARE VALID"
      );

      Serial.println(
        "Rollback anulowany"
      );

      Serial.println(
        "=============================="
      );
    }

    else {

      Serial.print(
        "OTA VALID ERROR: "
      );

      Serial.println(
        esp_err_to_name(
          result
        )
      );
    }
  }
}


// =====================================================
// OTA
// =====================================================

void performOTA() {

  if (otaInProgress) {

    return;
  }


  if (
    otaPendingVerification
  ) {

    Serial.println(
      "OTA zablokowane: firmware PENDING_VERIFY"
    );

    return;
  }


  if (
    WiFi.status() !=
    WL_CONNECTED
  ) {

    Serial.println(
      "OTA ERROR: brak WiFi"
    );

    return;
  }


  otaInProgress = true;
  otaLastProgress = -1;


  Serial.println();
  Serial.println(
    "=============================="
  );

  Serial.println(
    "START OTA"
  );

  Serial.print(
    "Firmware: "
  );

  Serial.println(
    FW_VERSION
  );

  Serial.println(
    "=============================="
  );


  WiFiClientSecure client;


  // Na razie HTTPS bez lokalnej
  // weryfikacji certyfikatu CA.
  client.setInsecure();


  // GitHub Release przekierowuje
  // do serwera z assetem.
  httpUpdate.setFollowRedirects(
    HTTPC_FORCE_FOLLOW_REDIRECTS
  );


  httpUpdate.rebootOnUpdate(
    true
  );


  httpUpdate.onStart([]() {

    Serial.println(
      "OTA: pobieranie..."
    );
  });


  httpUpdate.onProgress(
    [](int current, int total) {

      if (
        total <= 0
      ) {

        return;
      }


      int percent =
        (current * 100) /
        total;


      if (
        percent !=
          otaLastProgress &&
        percent % 10 == 0
      ) {

        otaLastProgress =
          percent;


        Serial.print(
          "OTA: "
        );

        Serial.print(
          percent
        );

        Serial.println(
          "%"
        );
      }
    }
  );


  httpUpdate.onEnd([]() {

    Serial.println(
      "OTA zapis zakonczony"
    );

    Serial.println(
      "Restart..."
    );
  });


  httpUpdate.onError(
    [](int error) {

      Serial.print(
        "OTA ERROR: "
      );

      Serial.println(
        error
      );


      Serial.print(
        "Opis: "
      );

      Serial.println(
        httpUpdate.getLastErrorString()
      );
    }
  );


  t_httpUpdate_return result =
    httpUpdate.update(
      client,
      OTA_URL
    );


  if (
    result ==
    HTTP_UPDATE_FAILED
  ) {

    Serial.print("OTA ERROR: ");
    Serial.println(httpUpdate.getLastError());
    Serial.print("Opis: ");
    Serial.println(httpUpdate.getLastErrorString());

    Serial.println(
      "OTA zakonczone bledem"
    );
  }

  else if (
    result ==
    HTTP_UPDATE_NO_UPDATES
  ) {

    Serial.println(
      "OTA: brak aktualizacji"
    );
  }

  else if (
    result ==
    HTTP_UPDATE_OK
  ) {

    Serial.println(
      "OTA OK"
    );
  }


  otaInProgress = false;
}


// =====================================================
// CZYTELNY STATUS SUPLA
// =====================================================

const char *getSuplaStatusText(
  int status
) {

  switch (status) {

    case STATUS_INITIALIZED:
      return "INITIALIZED";

    case STATUS_SERVER_DISCONNECTED:
      return "SERVER DISCONNECTED";

    case STATUS_NETWORK_DISCONNECTED:
      return "NETWORK DISCONNECTED";

    case STATUS_REGISTER_IN_PROGRESS:
      return "REGISTERING";

    case STATUS_REGISTERED_AND_READY:
      return "READY";

    case STATUS_CHANNEL_CONFLICT:
      return "CHANNEL CONFLICT";

    case STATUS_BAD_CREDENTIALS:
      return "BAD CREDENTIALS";

    case STATUS_MISSING_CREDENTIALS:
      return "MISSING CREDENTIALS";

    case STATUS_INVALID_AUTHKEY:
      return "INVALID AUTHKEY";

    case STATUS_CONFIG_MODE:
      return "CONFIG MODE";

    default:
      return "OTHER";
  }
}


// =====================================================
// DIAGNOSTYKA WIFI + SUPLA
// drukuje tylko po zmianie
// =====================================================

void handleDiagnostics() {

  if (
    millis() -
      lastDiagnosticsCheck <
    1000
  ) {

    return;
  }


  lastDiagnosticsCheck =
    millis();


  int wifiStatus =
    WiFi.status();


  int suplaStatus =
    SuplaDevice.getCurrentStatus();


  if (
    wifiStatus !=
    lastWifiStatus
  ) {

    lastWifiStatus =
      wifiStatus;


    Serial.print(
      "WiFi status: "
    );

    Serial.println(
      wifiStatus
    );


    if (
      wifiStatus ==
      WL_CONNECTED
    ) {

      Serial.print(
        "IP: "
      );

      Serial.println(
        WiFi.localIP()
      );


      Serial.print(
        "RSSI: "
      );

      Serial.print(
        WiFi.RSSI()
      );

      Serial.println(
        " dBm"
      );
    }
  }


  if (
    suplaStatus !=
    lastSuplaStatus
  ) {

    lastSuplaStatus =
      suplaStatus;


    Serial.print(
      "SUPLA status: "
    );

    Serial.print(
      suplaStatus
    );

    Serial.print(
      " - "
    );

    Serial.println(
      getSuplaStatusText(
        suplaStatus
      )
    );
  }
}


// =====================================================
// TWORZENIE KANALOW
// =====================================================

void namePumpChannel(Supla::Element *element, uint8_t pump,
                     const char *description) {
  String caption = String(PUMP_NAMES[pump]) + " " + description;
  element->setInitialCaption(caption.c_str());
}

void numberChannel(Supla::Element *element, uint8_t number) {
  if (!element->getChannel()->setChannelNumber(number)) {
    Serial.print("Nie mozna przypisac kanalu SUPLA nr ");
    Serial.println(number);
  }
}

void printDsBindings() {
  for (uint8_t slot = 0; slot < DS_SENSOR_COUNT; slot++) {
    if (!dsBound[slot]) continue;
    Serial.print("DS18B20 czujnik ");
    Serial.print(slot + 1);
    Serial.print(" ROM: ");
    for (uint8_t byteIndex = 0; byteIndex < 8; byteIndex++) {
      if (dsAddress[slot][byteIndex] < 0x10) Serial.print('0');
      Serial.print(dsAddress[slot][byteIndex], HEX);
    }
    Serial.println();
  }
}

void discoverDsSensors() {
  lastDsDiscovery = millis();
  dsBus.begin();
  dsBus.setWaitForConversion(false);
  dsBus.setResolution(12);

  Preferences preferences;
  if (!preferences.begin("melco-1wire", false)) {
    Serial.println("DS18B20: blad NVS, nowe czujniki nie beda przypisane");
    return;
  }

  bool changed = false;
  DeviceAddress foundAddress;
  for (uint8_t index = 0; index < dsBus.getDeviceCount(); index++) {
    if (!dsBus.getAddress(foundAddress, index) ||
        foundAddress[0] != 0x28 ||
        OneWire::crc8(foundAddress, 7) != foundAddress[7]) continue;

    bool known = false;
    for (uint8_t slot = 0; slot < DS_SENSOR_COUNT; slot++) {
      if (dsBound[slot] && memcmp(dsAddress[slot], foundAddress, 8) == 0) {
        known = true;
        break;
      }
    }
    if (known) continue;

    for (uint8_t slot = 0; slot < DS_SENSOR_COUNT; slot++) {
      if (dsBound[slot]) continue;
      if (preferences.putBytes(DS_KEYS[slot], foundAddress, 8) == 8) {
        memcpy(dsAddress[slot], foundAddress, 8);
        dsBound[slot] = true;
        changed = true;
        Serial.print("DS18B20: przypisano nowy czujnik ");
        Serial.println(slot + 1);
      } else {
        Serial.println("DS18B20: blad zapisu adresu do NVS");
      }
      break;
    }
  }
  preferences.end();
  if (changed) printDsBindings();
}

void initDsSensors() {
  Preferences preferences;
  if (preferences.begin("melco-1wire", true)) {
    for (uint8_t slot = 0; slot < DS_SENSOR_COUNT; slot++) {
      if (preferences.getBytesLength(DS_KEYS[slot]) == 8 &&
          preferences.getBytes(DS_KEYS[slot], dsAddress[slot], 8) == 8 &&
          dsAddress[slot][0] == 0x28 &&
          OneWire::crc8(dsAddress[slot], 7) == dsAddress[slot][7]) {
        dsBound[slot] = true;
      }
    }
    preferences.end();
  }
  discoverDsSensors();
  printDsBindings();
}

void handleDsSensors() {
  unsigned long now = millis();
  if (!dsConversionPending) {
    if (now - lastDsDiscovery >= 60000) discoverDsSensors();
    if (now - dsConversionStarted >= DS_READ_INTERVAL) {
      dsBus.requestTemperatures();
      dsConversionStarted = millis();
      dsConversionPending = true;
    }
    return;
  }
  if (now - dsConversionStarted < DS_CONVERSION_TIME) return;

  double totalKwh = 0;
  bool complete = true;
  for (uint8_t slot = 0; slot < DS_SENSOR_COUNT; slot++) {
    double temperature = TEMPERATURE_NOT_AVAILABLE;
    if (dsBound[slot]) {
      temperature = dsBus.getTempC(dsAddress[slot]);
      if (temperature == DEVICE_DISCONNECTED_C || temperature == 85.0 ||
          temperature < -55.0 || temperature > 125.0) {
        temperature = TEMPERATURE_NOT_AVAILABLE;
      }
    }
    dsChannel[slot]->setValue(temperature);
    if (slot < TANK_COUNT) {
      if (temperature == TEMPERATURE_NOT_AVAILABLE) {
        complete = false;
      } else if (temperature > CWU_COLD_REFERENCE_C) {
        totalKwh += TANK_VOLUME_LITERS *
                    (temperature - CWU_COLD_REFERENCE_C) * 0.001163;
      }
    }
  }
  cwuEnergy->setValue(complete ? totalKwh : NAN);
  dsConversionPending = false;
}

void createSuplaChannels() {


  // =================================================
  // 0 - Temperatura zewnetrzna
  // =================================================

  outdoorTemp =
    new Supla::Sensor::VirtualThermometer();

  namePumpChannel(outdoorTemp, 0, "Temperatura zewnetrzna");


  // =================================================
  // 1 - Temperatura zasilania
  // =================================================

  flowTemp =
    new Supla::Sensor::VirtualThermometer();

  namePumpChannel(flowTemp, 0, "Wylot wody");


  // =================================================
  // 2 - Temperatura powrotu
  // =================================================

  returnTemp =
    new Supla::Sensor::VirtualThermometer();

  namePumpChannel(returnTemp, 0, "Wlot wody");


  // =================================================
  // 3 - Czestotliwosc HP
  // =================================================

  hpFrequency =
    new Supla::Sensor::GeneralPurposeMeasurement();

  namePumpChannel(hpFrequency, 0, "Czestotliwosc HP");

  hpFrequency->setDefaultUnitAfterValue(
    "Hz"
  );

  hpFrequency->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 4 - Praca pompy ciepla
  // =================================================

  hpRunning =
    new Supla::Sensor::VirtualBinary();

  namePumpChannel(hpRunning, 0, "Praca pompy");


  // =================================================
  // 5 - Kod bledu
  // =================================================

  faultCode =
    new Supla::Sensor::GeneralPurposeMeasurement();

  namePumpChannel(faultCode, 0, "Kod bledu");

  faultCode->setDefaultValuePrecision(
    0
  );

  // Zachowujemy stare numery i typy, aby Cloud nie odrzucil rejestracji.
  firmwareA1M = new Supla::Sensor::GeneralPurposeMeasurement();
  numberChannel(firmwareA1M, 6);
  namePumpChannel(firmwareA1M, 0, "Firmware A1M");
  firmwareA1M->setDefaultValuePrecision(0);

  modbusCounter = new Supla::Sensor::GeneralPurposeMeasurement();
  numberChannel(modbusCounter, 7);
  namePumpChannel(modbusCounter, 0, "Licznik Modbus");
  modbusCounter->setDefaultValuePrecision(0);

  systemType = new Supla::Sensor::GeneralPurposeMeasurement();
  numberChannel(systemType, 8);
  namePumpChannel(systemType, 0, "Typ systemu");
  systemType->setDefaultValuePrecision(0);


  // =================================================
  // 9 - Aktualizacja OTA
  // =================================================

  otaTrigger =
    new Supla::Control::VirtualRelay();
  numberChannel(otaTrigger, 9);

  otaTrigger->setInitialCaption(
    "Aktualizacja OTA"
  );

  otaTrigger->setDefaultFunction(
    SUPLA_CHANNELFNC_POWERSWITCH
  );

  otaTrigger->setDefaultStateOff();


  // =================================================
  // 10 - Odszranianie
  // =================================================

  defrostStatus =
    new Supla::Sensor::GeneralPurposeMeasurement();
  numberChannel(defrostStatus, 10);

  namePumpChannel(defrostStatus, 0, "Odszranianie");

  defrostStatus->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 11 - Tryb pracy
  // =================================================

  operatingMode =
    new Supla::Sensor::GeneralPurposeMeasurement();
  numberChannel(operatingMode, 11);

  namePumpChannel(operatingMode, 0, "Tryb pracy");

  operatingMode->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 12 - Komunikacja Modbus
  // =================================================

  modbusStatus =
    new Supla::Sensor::VirtualBinary();
  numberChannel(modbusStatus, 12);

  namePumpChannel(modbusStatus, 0, "Komunikacja Modbus");

  modbusStatus->clear();

  PumpChannels &first = pumpChannels[0];
  first.outdoor = outdoorTemp;
  first.outlet = flowTemp;
  first.inlet = returnTemp;
  first.frequency = hpFrequency;
  first.running = hpRunning;
  first.fault = faultCode;
  first.firmware = firmwareA1M;
  first.counter = modbusCounter;
  first.systemType = systemType;
  first.defrost = defrostStatus;
  first.mode = operatingMode;
  first.online = modbusStatus;

  // 13-14: dodatkowe odczyty CAHV 1.
  first.settingWater = new Supla::Sensor::VirtualThermometer();
  numberChannel(first.settingWater, 13);
  namePumpChannel(first.settingWater, 0, "Nastawa temperatury wylotu");
  first.systemOn = new Supla::Sensor::GeneralPurposeMeasurement();
  numberChannel(first.systemOn, 14);
  namePumpChannel(first.systemOn, 0, "System ON/OFF");
  first.systemOn->setDefaultValuePrecision(0);

  // 15-28: CAHV 2, 29-42: QAHV. W obu zestawach identyczna kolejnosc.
  for (uint8_t pump = 1; pump < PUMP_COUNT; pump++) {
    PumpChannels &channels = pumpChannels[pump];
    const uint8_t base = pump == 1 ? 15 : 29;
    channels.outdoor = new Supla::Sensor::VirtualThermometer();
    numberChannel(channels.outdoor, base);
    namePumpChannel(channels.outdoor, pump, "Temperatura zewnetrzna");
    channels.outlet = new Supla::Sensor::VirtualThermometer();
    numberChannel(channels.outlet, base + 1);
    namePumpChannel(channels.outlet, pump, "Wylot wody");
    channels.inlet = new Supla::Sensor::VirtualThermometer();
    numberChannel(channels.inlet, base + 2);
    namePumpChannel(channels.inlet, pump, "Wlot wody");

    channels.frequency = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.frequency, base + 3);
    namePumpChannel(channels.frequency, pump, "Czestotliwosc HP");
    channels.frequency->setDefaultUnitAfterValue("Hz");
    channels.frequency->setDefaultValuePrecision(0);
    channels.running = new Supla::Sensor::VirtualBinary();
    numberChannel(channels.running, base + 4);
    namePumpChannel(channels.running, pump, "Praca pompy");

    channels.fault = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.fault, base + 5);
    namePumpChannel(channels.fault, pump, "Kod bledu");
    channels.fault->setDefaultValuePrecision(0);
    channels.firmware = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.firmware, base + 6);
    namePumpChannel(channels.firmware, pump, "Firmware A1M");
    channels.firmware->setDefaultValuePrecision(0);
    channels.counter = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.counter, base + 7);
    namePumpChannel(channels.counter, pump, "Licznik Modbus");
    channels.counter->setDefaultValuePrecision(0);
    channels.systemType = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.systemType, base + 8);
    namePumpChannel(channels.systemType, pump, "Typ systemu");
    channels.systemType->setDefaultValuePrecision(0);
    channels.defrost = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.defrost, base + 9);
    namePumpChannel(channels.defrost, pump, "Odszranianie");
    channels.defrost->setDefaultValuePrecision(0);
    channels.mode = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.mode, base + 10);
    namePumpChannel(channels.mode, pump, "Tryb pracy");
    channels.mode->setDefaultValuePrecision(0);
    channels.online = new Supla::Sensor::VirtualBinary();
    numberChannel(channels.online, base + 11);
    namePumpChannel(channels.online, pump, "Komunikacja Modbus");
    channels.online->clear();
    channels.settingWater = new Supla::Sensor::VirtualThermometer();
    numberChannel(channels.settingWater, base + 12);
    namePumpChannel(channels.settingWater, pump, "Nastawa temperatury wylotu");
    channels.systemOn = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.systemOn, base + 13);
    namePumpChannel(channels.systemOn, pump, "System ON/OFF");
    channels.systemOn->setDefaultValuePrecision(0);
  }

  // Kanaly 43-45 dopisane na koncu, aby zachowac identyfikatory 0-42.
  for (uint8_t pump = 0; pump < 2; pump++) {
    PumpChannels &channels = pumpChannels[pump];
    channels.runtimeHours = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.runtimeHours, 43 + pump);
    namePumpChannel(channels.runtimeHours, pump, "Godziny pracy");
    channels.runtimeHours->setDefaultUnitAfterValue("h");
    channels.runtimeHours->setDefaultValuePrecision(0);
  }
  pumpChannels[2].thermoOff = new Supla::Sensor::VirtualThermometer();
  numberChannel(pumpChannels[2].thermoOff, 45);
  namePumpChannel(pumpChannels[2].thermoOff, 2, "Temperatura Thermo-off");

  // 46-51: nowe kanaly na koncu, bez zmiany numerow istniejacych kanalow.
  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    PumpChannels &channels = pumpChannels[pump];
    channels.temperatureDelta = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.temperatureDelta, 46 + pump * 2);
    namePumpChannel(channels.temperatureDelta, pump, "Delta zasilanie-powrot");
    channels.temperatureDelta->setDefaultUnitAfterValue("\xC2\xB0" "C");
    channels.temperatureDelta->setDefaultValuePrecision(2);

    channels.compressorStarts = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.compressorStarts, 47 + pump * 2);
    namePumpChannel(channels.compressorStarts, pump,
                    "Zaobserwowane starty sprezarki");
    channels.compressorStarts->setDefaultValuePrecision(0);
  }

  // 52-59: osiem czujnikow temperatury na wspolnej magistrali 1-Wire.
  for (uint8_t slot = 0; slot < DS_SENSOR_COUNT; slot++) {
    dsChannel[slot] = new Supla::Sensor::VirtualThermometer();
    numberChannel(dsChannel[slot], 52 + slot);
    dsChannel[slot]->setInitialCaption(DS_CAPTIONS[slot]);
  }

  // 60: energia cieplna powyzej 10 C, wyliczona z pieciu zbiornikow.
  cwuEnergy = new Supla::Sensor::GeneralPurposeMeasurement();
  numberChannel(cwuEnergy, 60);
  cwuEnergy->setInitialCaption("CWU - szacowana energia 5 zbiornikow");
  cwuEnergy->setDefaultUnitAfterValue("kWh");
  cwuEnergy->setDefaultValuePrecision(1);
  cwuEnergy->setValue(NAN);

  // 61-65: tylko sterowanie reczne. Automatyka kotla zostanie dodana pozniej.
  for (uint8_t relay = 0; relay < RELAY_COUNT; relay++) {
    physicalRelay[relay] = new Supla::Control::Relay(
      RELAY_PINS[relay], true, SUPLA_BIT_FUNC_POWERSWITCH);
    numberChannel(physicalRelay[relay], 61 + relay);
    physicalRelay[relay]->setInitialCaption(RELAY_CAPTIONS[relay]);
    physicalRelay[relay]->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
    physicalRelay[relay]->setDefaultStateOff();
  }

  // Nowe kanaly na koncu; numery i typy kanalow 0-65 pozostaja bez zmian.
  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    PumpChannels &channels = pumpChannels[pump];
    channels.compressorRuntime = new Supla::Sensor::GeneralPurposeMeasurement();
    numberChannel(channels.compressorRuntime, 66 + pump);
    namePumpChannel(channels.compressorRuntime, pump,
                    "Szacowany czas pracy sprezarki");
    channels.compressorRuntime->setDefaultUnitAfterValue("h");
    channels.compressorRuntime->setDefaultValuePrecision(2);
  }
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  for (uint8_t relay = 0; relay < RELAY_COUNT; relay++) {
    digitalWrite(RELAY_PINS[relay], LOW);
    pinMode(RELAY_PINS[relay], OUTPUT);
  }

  Serial.begin(
    115200
  );


  delay(
    1000
  );

  // BOOT (GPIO 0): przytrzymanie po uruchomieniu wlacza AP konfiguracji.
  // Krotkie nacisniecie nie wykonuje zadnej akcji i nie kasuje ustawien.
  auto configButton = new Supla::Control::Button(
    BOOT_BUTTON_PIN, true, true);
  configButton->dontUseOnLoadConfig();
  configButton->setHoldTime(BOOT_BUTTON_HOLD_MS);
  configButton->addAction(
    Supla::ENTER_CONFIG_MODE, &SuplaDevice, Supla::ON_HOLD, true);


  Serial.println();
  Serial.println(
    "=============================="
  );

  Serial.println(
    "Mitsubishi MelcoBEMS SUPLA"
  );

  Serial.print(
    "Firmware: "
  );

  Serial.println(
    FW_VERSION
  );

  Serial.println(
    "=============================="
  );


  // =================================================
  // MODBUS
  // =================================================

  RS485.begin(
    MODBUS_BAUD,
    SERIAL_8N1,
    RS485_RX,
    RS485_TX
  );


  node.begin(PUMP_SLAVE_IDS[0], RS485);


  // Kluczowa rzecz:
  // podczas oczekiwania Modbus obslugujemy SUPLA.
  node.idle(
    modbusIdle
  );


  // =================================================
  // SUPLA WWW
  // =================================================

  new Supla::Html::DeviceInfo(
    &SuplaDevice
  );

  new Supla::Html::WifiParameters;

  new Supla::Html::ProtocolParameters;


  // =================================================
  // KANALY
  // =================================================

  createSuplaChannels();
  configureGpmHistory(false);
  loadCompressorStartCounts();
  loadCompressorRuntime();
  initDsSensors();

  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    pumpState[pump].nextProbe = millis() + 1500;
  }


  // =================================================
  // DEVICE
  // =================================================

  SuplaDevice.setName(
    "Kotlownia - 3 pompy MelcoBEMS"
  );


  SuplaDevice.setSwVersion(
    FW_VERSION
  );


  SuplaDevice.setInitialMode(
    Supla::InitialMode::StartInCfgMode
  );

  SuplaDevice.setProtoVerboseLog(false);


  // =================================================
  // START
  // =================================================

  SuplaDevice.begin();


  // =================================================
  // ROLLBACK
  // =================================================

  initRollbackState();


  Serial.print(
    "Free sketch space: "
  );

  Serial.println(
    ESP.getFreeSketchSpace()
  );


  Serial.println(
    "Start schedulera Modbus"
  );
}


// =====================================================
// LOOP
// =====================================================

void loop() {


  // SUPLA zawsze ma najwyzszy priorytet.
  SuplaDevice.iterate();


  // Czytelny debug WiFi/SUPLA.
  handleDiagnostics();


  // Walidacja firmware po OTA.
  handleRollbackVerification();


  // Historia + teksty statusow GPM.
  updateStatusDescriptions();


  // =================================================
  // OTA Z SUPLA
  // =================================================

  if (
    otaTrigger->isOn() &&
    !otaInProgress
  ) {

    Serial.println();
    Serial.println(
      "OTA wyzwolone z SUPLA"
    );


    // Zachowuje sie jak przycisk.
    otaTrigger->turnOff();


    SuplaDevice.iterate();

    for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
      updateCompressorRuntime(pump);
      saveCompressorRuntime(pump);
    }


    delay(
      250
    );


    performOTA();
  }


  // =================================================
  // MODBUS
  // =================================================

  handleDsSensors();
  handleModbusScheduler();
  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    updateCompressorRuntime(pump);
  }
}
