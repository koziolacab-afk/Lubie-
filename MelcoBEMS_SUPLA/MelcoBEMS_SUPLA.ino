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

#define FW_VERSION "1.0.3"


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

#define MODBUS_SLAVE_ID 1
#define MODBUS_BAUD 9600

HardwareSerial RS485(1);
ModbusMaster node;


// =====================================================
// SUPLA
// =====================================================

Supla::ESPWifi wifi;
Supla::LittleFsConfig configSupla;
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
  REG_SYSTEM_TYPE
};

void applyRegisterValue(uint8_t type, uint16_t raw);

struct ModbusPollItem {
  uint16_t address;
  unsigned long interval;
  unsigned long lastPoll;
  RegisterType type;
};


// Rejestry dynamiczne: 5 s
// Rejestry diagnostyczne: 60 s
ModbusPollItem pollItems[] = {

  {99,  5000, 0, REG_OUTDOOR},
  {101, 5000, 0, REG_FLOW},
  {103, 5000, 0, REG_RETURN},
  {73,  5000, 0, REG_FREQUENCY},
  {127, 5000, 0, REG_HP_RUNNING},
  {67,  5000, 0, REG_DEFROST},
  {26,  5000, 0, REG_OPERATING_MODE},
  {9,   5000, 0, REG_FAULT},

  {10, 60000, 0, REG_FIRMWARE},
  {11, 60000, 0, REG_COUNTER},
  {13, 60000, 0, REG_SYSTEM_TYPE}
};


const uint8_t POLL_ITEM_COUNT =
  sizeof(pollItems) / sizeof(pollItems[0]);


// Minimalna przerwa pomiedzy poprawnymi transakcjami.
const unsigned long MODBUS_MIN_GAP = 250;


// Po utracie komunikacji probujemy tylko jednego
// rejestru co 10 sekund.
const unsigned long MODBUS_OFFLINE_RETRY = 10000;


uint8_t pollCursor = 0;

unsigned long lastModbusTransaction = 0;
unsigned long nextModbusProbe = 1500;

uint8_t consecutiveModbusFailures = 0;

bool modbusOnline = false;
bool modbusStateKnown = false;


// =====================================================
// STATUSY DO HACKU GPM
// =====================================================

uint16_t currentDefrost = 0;
uint16_t currentOperatingMode = 0;
uint16_t currentFault = 0;
uint16_t currentSystemType = 0;

bool haveDefrost = false;
bool haveOperatingMode = false;
bool haveFault = false;
bool haveSystemType = false;

int32_t lastDefrostDescription = -1;
int32_t lastOperatingDescription = -1;
int32_t lastFaultDescription = -1;
int32_t lastSystemDescription = -1;


// =====================================================
// SUPLA CONFIG
// =====================================================

bool suplaWasReady = false;
bool historyConfigured = false;

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
  uint16_t address,
  uint16_t &value
) {

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
    "Modbus ERROR addr "
  );

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

void setModbusOnline(bool online) {

  if (modbusStateKnown &&
      modbusOnline == online) {

    return;
  }


  modbusStateKnown = true;
  modbusOnline = online;


  if (online) {

    modbusStatus->set();

    Serial.println();
    Serial.println(
      "MODBUS: ONLINE"
    );
  }
  else {

    modbusStatus->clear();

    Serial.println();
    Serial.println(
      "MODBUS: OFFLINE"
    );
  }
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
  uint8_t type,
  uint16_t raw
) {

  switch (type) {


    // -----------------------------------------------
    // Outdoor temp
    // addr 99
    // signed / 10
    // -----------------------------------------------

    case REG_OUTDOOR: {

      double value =
        ((int16_t)raw) / 10.0;

      outdoorTemp->setValue(value);

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

      flowTemp->setValue(value);

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

      returnTemp->setValue(value);

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

      hpFrequency->setValue(raw);

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

        hpRunning->set();
      }
      else {

        hpRunning->clear();
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

      defrostStatus->setValue(raw);

      currentDefrost = raw;
      haveDefrost = true;


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

      operatingMode->setValue(raw);

      currentOperatingMode = raw;
      haveOperatingMode = true;


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

      faultCode->setValue(raw);

      currentFault = raw;
      haveFault = true;


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

      firmwareA1M->setValue(raw);

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

      modbusCounter->setValue(raw);

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

      systemType->setValue(raw);

      currentSystemType = raw;
      haveSystemType = true;


      Serial.print(
        "System type: "
      );

      Serial.println(raw);

      break;
    }
  }
}


// =====================================================
// MODBUS PROBE
//
// Kiedy A1M jest offline nie odpytujemy wszystkich
// rejestrow. Probujemy tylko addr 99 co 10 s.
// =====================================================

void handleOfflineModbusProbe() {

  unsigned long now =
    millis();


  if (!timeReached(
        now,
        nextModbusProbe)) {

    return;
  }


  uint16_t raw;


  Serial.println(
    "MODBUS: probe addr 99..."
  );


  bool ok =
    readHR(
      99,
      raw
    );


  lastModbusTransaction =
    millis();


  if (ok) {

    Serial.println(
      "MODBUS: odpowiedz OK"
    );


    consecutiveModbusFailures = 0;

    setModbusOnline(true);


    // addr 99 juz mamy odczytany,
    // wiec od razu wykorzystujemy wartosc.
    applyRegisterValue(
      REG_OUTDOOR,
      raw
    );


    pollItems[0].lastPoll =
      millis();


    return;
  }


  setModbusOnline(false);


  nextModbusProbe =
    millis() +
    MODBUS_OFFLINE_RETRY;


  Serial.println(
    "MODBUS: kolejna proba za 10 s"
  );
}


// =====================================================
// MODBUS SCHEDULER
//
// Jedna transakcja na raz.
// Nie robimy juz 8 timeoutow jeden po drugim.
// =====================================================

void handleModbusScheduler() {

  if (otaInProgress) {

    return;
  }


  // -----------------------------------------------
  // OFFLINE
  // -----------------------------------------------

  if (!modbusOnline) {

    handleOfflineModbusProbe();

    return;
  }


  unsigned long now =
    millis();


  // Minimalna przerwa pomiedzy transakcjami.
  if (now -
        lastModbusTransaction <
      MODBUS_MIN_GAP) {

    return;
  }


  // Szukamy pierwszego rejestru,
  // ktorego termin juz minal.
  for (
    uint8_t checked = 0;
    checked < POLL_ITEM_COUNT;
    checked++
  ) {

    uint8_t index =
      (pollCursor + checked) %
      POLL_ITEM_COUNT;


    ModbusPollItem &item =
      pollItems[index];


    if (now -
          item.lastPoll <
        item.interval) {

      continue;
    }


    pollCursor =
      (index + 1) %
      POLL_ITEM_COUNT;


    uint16_t raw;


    bool ok =
      readHR(
        item.address,
        raw
      );


    item.lastPoll =
      millis();


    lastModbusTransaction =
      millis();


    if (ok) {

      consecutiveModbusFailures = 0;

      applyRegisterValue(
        item.type,
        raw
      );
    }

    else {

      if (
        consecutiveModbusFailures <
        255
      ) {

        consecutiveModbusFailures++;
      }


      Serial.print(
        "Modbus failure streak: "
      );

      Serial.println(
        consecutiveModbusFailures
      );


      // 3 kolejne timeouty/bledy =
      // przechodzimy w tryb offline.
      if (
        consecutiveModbusFailures >= 3
      ) {

        setModbusOnline(false);

        consecutiveModbusFailures = 0;

        nextModbusProbe =
          millis() +
          MODBUS_OFFLINE_RETRY;
      }
    }


    // Tylko JEDNA transakcja na jedno
    // wywolanie schedulera.
    return;
  }
}


// =====================================================
// HISTORIA GPM
// =====================================================

void configureGpmHistory() {

  // Czestotliwosc HP
  if (
    hpFrequency->getKeepHistory() != 1
  ) {

    hpFrequency->setKeepHistory(1);
  }


  // Kod bledu
  if (
    faultCode->getKeepHistory() != 1
  ) {

    faultCode->setKeepHistory(1);
  }


  // Defrost
  if (
    defrostStatus->getKeepHistory() != 1
  ) {

    defrostStatus->setKeepHistory(1);
  }


  // Tryb pracy
  if (
    operatingMode->getKeepHistory() != 1
  ) {

    operatingMode->setKeepHistory(1);
  }


  historyConfigured = true;


  Serial.println(
    "SUPLA: historia GPM wlaczona"
  );
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

    return;
  }


  if (!suplaWasReady) {

    suplaWasReady = true;

    suplaReadySince =
      millis();


    // Pozwalamy ponownie wyslac opis
    // po reconnect.
    lastDefrostDescription = -1;
    lastOperatingDescription = -1;
    lastFaultDescription = -1;
    lastSystemDescription = -1;


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
  if (!historyConfigured) {

    configureGpmHistory();
  }


  // -----------------------------------------------
  // DEFROST
  // -----------------------------------------------

  if (
    haveDefrost &&
    currentDefrost !=
      lastDefrostDescription
  ) {

    defrostStatus->setUnitAfterValue(
      getDefrostText(
        currentDefrost
      )
    );


    lastDefrostDescription =
      currentDefrost;
  }


  // -----------------------------------------------
  // OPERATING MODE
  // -----------------------------------------------

  if (
    haveOperatingMode &&
    currentOperatingMode !=
      lastOperatingDescription
  ) {

    operatingMode->setUnitAfterValue(
      getOperatingModeText(
        currentOperatingMode
      )
    );


    lastOperatingDescription =
      currentOperatingMode;
  }


  // -----------------------------------------------
  // FAULT
  // -----------------------------------------------

  if (
    haveFault &&
    currentFault !=
      lastFaultDescription
  ) {

    faultCode->setUnitAfterValue(
      getFaultText(
        currentFault
      )
    );


    lastFaultDescription =
      currentFault;
  }


  // -----------------------------------------------
  // SYSTEM TYPE
  // -----------------------------------------------

  if (
    haveSystemType &&
    currentSystemType !=
      lastSystemDescription
  ) {

    systemType->setUnitAfterValue(
      getSystemTypeText(
        currentSystemType
      )
    );


    lastSystemDescription =
      currentSystemType;
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

void createSuplaChannels() {


  // =================================================
  // 0 - Temperatura zewnetrzna
  // =================================================

  outdoorTemp =
    new Supla::Sensor::VirtualThermometer();

  outdoorTemp->setInitialCaption(
    "Temperatura zewnetrzna"
  );


  // =================================================
  // 1 - Temperatura zasilania
  // =================================================

  flowTemp =
    new Supla::Sensor::VirtualThermometer();

  flowTemp->setInitialCaption(
    "Temperatura zasilania"
  );


  // =================================================
  // 2 - Temperatura powrotu
  // =================================================

  returnTemp =
    new Supla::Sensor::VirtualThermometer();

  returnTemp->setInitialCaption(
    "Temperatura powrotu"
  );


  // =================================================
  // 3 - Czestotliwosc HP
  // =================================================

  hpFrequency =
    new Supla::Sensor::GeneralPurposeMeasurement();

  hpFrequency->setInitialCaption(
    "Czestotliwosc HP"
  );

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

  hpRunning->setInitialCaption(
    "Praca pompy ciepla"
  );


  // =================================================
  // 5 - Kod bledu
  // =================================================

  faultCode =
    new Supla::Sensor::GeneralPurposeMeasurement();

  faultCode->setInitialCaption(
    "Kod bledu"
  );

  faultCode->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 6 - Firmware MelcoBEMS
  // =================================================

  firmwareA1M =
    new Supla::Sensor::GeneralPurposeMeasurement();

  firmwareA1M->setInitialCaption(
    "Firmware MelcoBEMS"
  );

  firmwareA1M->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 7 - Licznik Modbus
  // =================================================

  modbusCounter =
    new Supla::Sensor::GeneralPurposeMeasurement();

  modbusCounter->setInitialCaption(
    "Licznik Modbus"
  );

  modbusCounter->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 8 - Typ systemu
  // =================================================

  systemType =
    new Supla::Sensor::GeneralPurposeMeasurement();

  systemType->setInitialCaption(
    "Typ systemu"
  );

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

  defrostStatus->setInitialCaption(
    "Odszranianie"
  );

  defrostStatus->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 11 - Tryb pracy
  // =================================================

  operatingMode =
    new Supla::Sensor::GeneralPurposeMeasurement();

  operatingMode->setInitialCaption(
    "Tryb pracy"
  );

  operatingMode->setDefaultValuePrecision(
    0
  );


  // =================================================
  // 12 - Komunikacja Modbus
  // =================================================

  modbusStatus =
    new Supla::Sensor::VirtualBinary();

  modbusStatus->setInitialCaption(
    "Komunikacja Modbus"
  );

  modbusStatus->clear();
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


  node.begin(
    MODBUS_SLAVE_ID,
    RS485
  );


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


  // =================================================
  // DEVICE
  // =================================================

  SuplaDevice.setName(
    "Mitsubishi MelcoBEMS"
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
