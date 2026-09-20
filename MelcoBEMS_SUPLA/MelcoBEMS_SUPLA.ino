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

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>

#include "esp_ota_ops.h"
#include "esp_err.h"


// =====================================================
// WERSJA
// =====================================================

#define FW_VERSION "1.0.8"


// =====================================================
// OTA
// =====================================================

const char *OTA_URL =
  "https://github.com/koziolacab-afk/Lubie-/releases/download/ota/firmware.bin";

const unsigned long OTA_VERIFY_DELAY = 60000;

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


// =====================================================
// SUPLA
// =====================================================

Supla::ESPWifi wifi;
// The default 1024-byte config buffer cannot hold settings for all 46 channels.
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

// 6
Supla::Sensor::GeneralPurposeMeasurement *firmwareA1M;

// 7
Supla::Sensor::GeneralPurposeMeasurement *modbusCounter;

// 8
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
  REG_FIRMWARE,
  REG_COUNTER,
  REG_SYSTEM_TYPE,
  REG_SETTING_WATER,
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
  {52,  5000, {0}, REG_SETTING_WATER, ALL_PUMPS},
  {25,  5000, {0}, REG_SYSTEM_ON, ALL_PUMPS},

  {10, 60000, {0}, REG_FIRMWARE, ALL_PUMPS},
  {11, 60000, {0}, REG_COUNTER, ALL_PUMPS},
  {13, 60000, {0}, REG_SYSTEM_TYPE, ALL_PUMPS},
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
  uint16_t systemType;
  bool haveDefrost;
  bool haveOperatingMode;
  bool haveFault;
  bool haveSystemType;
  uint16_t runtimeRemainder;
  uint16_t runtimeHundreds;
  bool haveRuntimeRemainder;
  bool haveRuntimeHundreds;
  int32_t lastDefrostDescription;
  int32_t lastOperatingDescription;
  int32_t lastFaultDescription;
  int32_t lastSystemDescription;
};

PumpState pumpState[PUMP_COUNT] = {};


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


  Serial.print(
    "Modbus ERROR slave "
  );

  Serial.print(PUMP_SLAVE_IDS[pump]);
  Serial.print(" addr ");
  Serial.print(address);

  Serial.print(
    " -> 0x"
  );

  Serial.println(
    result,
    HEX
  );


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


  state.stateKnown = true;
  state.online = online;


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


const char *getSystemTypeText(
  uint16_t value
) {

  switch (value) {

    case 0:
      return "ATA";

    case 1:
      return "ATW";

    case 2:
      return "Lossnay";

    case 255:
      return "Unknown";

    default:
      return "Unknown";
  }
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

  Serial.print(PUMP_NAMES[pump]);
  Serial.print(" - ");

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

      Serial.print("Outdoor: ");
      Serial.print(value);
      Serial.println(" C");

      break;
    }


    // -----------------------------------------------
    // Flow temp
    // addr 101
    // signed / 100
    // -----------------------------------------------

    case REG_FLOW: {

      double value =
        ((int16_t)raw) / 100.0;

      channels.outlet->setValue(value);

      Serial.print("Flow: ");
      Serial.print(value);
      Serial.println(" C");

      break;
    }


    // -----------------------------------------------
    // Return temp
    // addr 103
    // signed / 100
    // -----------------------------------------------

    case REG_RETURN: {

      double value =
        ((int16_t)raw) / 100.0;

      channels.inlet->setValue(value);

      Serial.print("Return: ");
      Serial.print(value);
      Serial.println(" C");

      break;
    }


    // -----------------------------------------------
    // HP Frequency
    // addr 73
    // -----------------------------------------------

    case REG_FREQUENCY: {

      channels.frequency->setValue(raw);

      Serial.print(
        "HP Frequency: "
      );

      Serial.print(raw);

      Serial.println(" Hz");

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


      Serial.print(
        "HP Run: "
      );

      Serial.println(raw);

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


      Serial.print(
        "Defrost: "
      );

      Serial.println(raw);

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


      Serial.print(
        "Operating mode: "
      );

      Serial.println(raw);

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


      Serial.print(
        "Fault: 0x"
      );

      Serial.println(
        raw,
        HEX
      );

      break;
    }


    // -----------------------------------------------
    // Melco firmware
    // addr 10
    // -----------------------------------------------

    case REG_FIRMWARE: {

      channels.firmware->setValue(raw);

      Serial.print(
        "Melco firmware: "
      );

      Serial.println(raw);

      break;
    }


    // -----------------------------------------------
    // Modbus counter
    // addr 11
    // -----------------------------------------------

    case REG_COUNTER: {

      channels.counter->setValue(raw);

      Serial.print(
        "Modbus counter: "
      );

      Serial.println(raw);

      break;
    }


    // -----------------------------------------------
    // System type
    // addr 13
    // -----------------------------------------------

    case REG_SYSTEM_TYPE: {

      channels.systemType->setValue(raw);

      state.systemType = raw;
      state.haveSystemType = true;


      Serial.print(
        "System type: "
      );

      Serial.println(raw);

      break;
    }

    case REG_SETTING_WATER:
      channels.settingWater->setValue(((int16_t)raw) / 100.0);
      Serial.print("Setting water: ");
      Serial.println(((int16_t)raw) / 100.0);
      break;

    case REG_SYSTEM_ON:
      channels.systemOn->setValue(raw);
      Serial.print("System on/off: ");
      Serial.println(raw);
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
      Serial.print("Thermo-off: ");
      Serial.println(((int16_t)raw) / 100.0);
      break;
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

  Serial.print("MODBUS: probe slave ");
  Serial.println(PUMP_SLAVE_IDS[pump]);

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
        Serial.print("Modbus failure streak slave ");
        Serial.print(PUMP_SLAVE_IDS[pump]);
        Serial.print(": ");
        Serial.println(state.failures);
        if (state.failures >= 3) {
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

void configureGpmHistory() {
  bool changed = false;
  for (uint8_t pump = 0; pump < PUMP_COUNT; pump++) {
    PumpChannels &channels = pumpChannels[pump];
    Supla::Sensor::GeneralPurposeMeasurement *measurements[] = {
      channels.frequency, channels.fault, channels.defrost,
      channels.mode, channels.systemOn, channels.runtimeHours
    };
    for (auto *measurement : measurements) {
      if (measurement && measurement->getKeepHistory() != 1) {
        measurement->setKeepHistory(1);
        changed = true;
      }
    }
  }
  if (changed) Serial.println("SUPLA: wlaczono historie pomiarow GPM");
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
      state.lastSystemDescription = -1;
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
    configureGpmHistory();
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
    if (state.haveSystemType &&
        state.systemType != state.lastSystemDescription) {
      channels.systemType->setUnitAfterValue(getSystemTypeText(state.systemType));
      state.lastSystemDescription = state.systemType;
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
        "Test firmware: 60 s"
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


  // =================================================
  // 6 - Firmware MelcoBEMS
  // =================================================

  firmwareA1M =
    new Supla::Sensor::GeneralPurposeMeasurement();

  namePumpChannel(firmwareA1M, 0, "Firmware A1M");

  firmwareA1M->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 7 - Licznik Modbus
  // =================================================

  modbusCounter =
    new Supla::Sensor::GeneralPurposeMeasurement();

  namePumpChannel(modbusCounter, 0, "Licznik Modbus");

  modbusCounter->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 8 - Typ systemu
  // =================================================

  systemType =
    new Supla::Sensor::GeneralPurposeMeasurement();

  namePumpChannel(systemType, 0, "Typ systemu");

  systemType->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 9 - Aktualizacja OTA
  // =================================================

  otaTrigger =
    new Supla::Control::VirtualRelay();

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

  namePumpChannel(defrostStatus, 0, "Odszranianie");

  defrostStatus->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 11 - Tryb pracy
  // =================================================

  operatingMode =
    new Supla::Sensor::GeneralPurposeMeasurement();

  namePumpChannel(operatingMode, 0, "Tryb pracy");

  operatingMode->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 12 - Komunikacja Modbus
  // =================================================

  modbusStatus =
    new Supla::Sensor::VirtualBinary();

  namePumpChannel(modbusStatus, 0, "Komunikacja Modbus");

  modbusStatus->clear();

  // Kanaly 0-12 zachowuja typ i numer, zmieniaja tylko podpis na CAHV 1.
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
  namePumpChannel(first.settingWater, 0, "Temperatura zadana wody");
  first.systemOn = new Supla::Sensor::GeneralPurposeMeasurement();
  namePumpChannel(first.systemOn, 0, "System ON/OFF");
  first.systemOn->setDefaultValuePrecision(0);

  // 15-28: CAHV 2, 29-42: QAHV. W obu zestawach identyczna kolejnosc.
  for (uint8_t pump = 1; pump < PUMP_COUNT; pump++) {
    PumpChannels &channels = pumpChannels[pump];
    channels.outdoor = new Supla::Sensor::VirtualThermometer();
    namePumpChannel(channels.outdoor, pump, "Temperatura zewnetrzna");
    channels.outlet = new Supla::Sensor::VirtualThermometer();
    namePumpChannel(channels.outlet, pump, "Wylot wody");
    channels.inlet = new Supla::Sensor::VirtualThermometer();
    namePumpChannel(channels.inlet, pump, "Wlot wody");

    channels.frequency = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.frequency, pump, "Czestotliwosc HP");
    channels.frequency->setDefaultUnitAfterValue("Hz");
    channels.frequency->setDefaultValuePrecision(0);
    channels.running = new Supla::Sensor::VirtualBinary();
    namePumpChannel(channels.running, pump, "Praca pompy");

    channels.fault = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.fault, pump, "Kod bledu");
    channels.fault->setDefaultValuePrecision(0);
    channels.firmware = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.firmware, pump, "Firmware A1M");
    channels.firmware->setDefaultValuePrecision(0);
    channels.counter = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.counter, pump, "Licznik Modbus");
    channels.counter->setDefaultValuePrecision(0);
    channels.systemType = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.systemType, pump, "Typ systemu");
    channels.systemType->setDefaultValuePrecision(0);

    channels.defrost = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.defrost, pump, "Odszranianie");
    channels.defrost->setDefaultValuePrecision(0);
    channels.mode = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.mode, pump, "Tryb pracy");
    channels.mode->setDefaultValuePrecision(0);
    channels.online = new Supla::Sensor::VirtualBinary();
    namePumpChannel(channels.online, pump, "Komunikacja Modbus");
    channels.online->clear();
    channels.settingWater = new Supla::Sensor::VirtualThermometer();
    namePumpChannel(channels.settingWater, pump, "Temperatura zadana wody");
    channels.systemOn = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.systemOn, pump, "System ON/OFF");
    channels.systemOn->setDefaultValuePrecision(0);
  }

  // Kanaly 43-45 dopisane na koncu, aby zachowac identyfikatory 0-42.
  for (uint8_t pump = 0; pump < 2; pump++) {
    PumpChannels &channels = pumpChannels[pump];
    channels.runtimeHours = new Supla::Sensor::GeneralPurposeMeasurement();
    namePumpChannel(channels.runtimeHours, pump, "Godziny pracy");
    channels.runtimeHours->setDefaultUnitAfterValue("h");
    channels.runtimeHours->setDefaultValuePrecision(0);
  }
  pumpChannels[2].thermoOff = new Supla::Sensor::VirtualThermometer();
  namePumpChannel(pumpChannels[2].thermoOff, 2, "Temperatura Thermo-off");
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(
    1000
  );


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


    delay(
      250
    );


    performOTA();
  }


  // =================================================
  // MODBUS
  // =================================================

  handleModbusScheduler();
}
