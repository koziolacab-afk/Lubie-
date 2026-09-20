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

#define FW_VERSION "1.0.2"


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


// Arduino ESP32 ma domyslna automatyczna walidacje.
// My odraczamy ja i zatwierdzamy firmware dopiero po 60 s.
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

// Temperatury
Supla::Sensor::VirtualThermometer *outdoorTemp;
Supla::Sensor::VirtualThermometer *flowTemp;
Supla::Sensor::VirtualThermometer *returnTemp;

// GPM
Supla::Sensor::GeneralPurposeMeasurement *hpFrequency;
Supla::Sensor::GeneralPurposeMeasurement *faultCode;
Supla::Sensor::GeneralPurposeMeasurement *firmwareA1M;
Supla::Sensor::GeneralPurposeMeasurement *modbusCounter;
Supla::Sensor::GeneralPurposeMeasurement *systemType;

// Binary
Supla::Sensor::VirtualBinary *hpRunning;

// OTA
Supla::Control::VirtualRelay *otaTrigger;

// Statusy z opisami
Supla::Sensor::GeneralPurposeMeasurement *defrostStatus;
Supla::Sensor::GeneralPurposeMeasurement *operatingMode;


// =====================================================
// MODBUS TIMERY
// =====================================================

const unsigned long FAST_POLL_INTERVAL = 5000;
const unsigned long SLOW_POLL_INTERVAL = 60000;

unsigned long lastFastPoll = 0;
unsigned long lastSlowPoll = 0;

bool firstSlowPoll = true;


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


// SUPLA config sync
bool suplaWasReady = false;
bool historyConfigured = false;

unsigned long suplaReadySince = 0;

const unsigned long SUPLA_CONFIG_SETTLE_TIME = 5000;


// =====================================================
// MODBUS
// =====================================================

bool readHR(uint16_t address, uint16_t &value) {

  uint8_t result =
    node.readHoldingRegisters(address, 1);

  if (result == node.ku8MBSuccess) {

    value = node.getResponseBuffer(0);

    return true;
  }

  Serial.print("Modbus ERROR addr ");
  Serial.print(address);
  Serial.print(" -> 0x");
  Serial.println(result, HEX);

  return false;
}


// =====================================================
// TEKSTY STATUSOW
// max 14 bajtow dla unit GPM
// =====================================================

const char *getDefrostText(uint16_t value) {

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


const char *getOperatingModeText(uint16_t value) {

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


const char *getFaultText(uint16_t value) {

  if (value == 0x8000) {
    return "OK";
  }

  if (value == 0x6999) {
    return "Comm error";
  }

  return "Fault";
}


const char *getSystemTypeText(uint16_t value) {

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
// HISTORIA GPM
// =====================================================

void configureGpmHistory() {

  // Ustawiamy tylko kanaly, gdzie historia ma sens.

  if (hpFrequency->getKeepHistory() != 1) {
    hpFrequency->setKeepHistory(1);
  }

  if (faultCode->getKeepHistory() != 1) {
    faultCode->setKeepHistory(1);
  }

  if (defrostStatus->getKeepHistory() != 1) {
    defrostStatus->setKeepHistory(1);
  }

  if (operatingMode->getKeepHistory() != 1) {
    operatingMode->setKeepHistory(1);
  }

  historyConfigured = true;

  Serial.println("SUPLA: historia GPM skonfigurowana");
}


// =====================================================
// HACK STATUS -> GPM
//
// np.
// 2 Defrost
// 2 Heating
// 32768 OK
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
    suplaReadySince = millis();

    // Po reconnect pozwalamy odswiezyc opis.
    lastDefrostDescription = -1;
    lastOperatingDescription = -1;
    lastFaultDescription = -1;
    lastSystemDescription = -1;

    return;
  }


  // Dajemy SUPLA chwile na pobranie konfiguracji kanalow.
  if (millis() - suplaReadySince <
      SUPLA_CONFIG_SETTLE_TIME) {

    return;
  }


  // Historia ustawiana raz po uruchomieniu.
  if (!historyConfigured) {

    configureGpmHistory();
  }


  // Defrost
  if (haveDefrost &&
      currentDefrost != lastDefrostDescription) {

    defrostStatus->setUnitAfterValue(
      getDefrostText(currentDefrost)
    );

    lastDefrostDescription =
      currentDefrost;
  }


  // Operating mode
  if (haveOperatingMode &&
      currentOperatingMode != lastOperatingDescription) {

    operatingMode->setUnitAfterValue(
      getOperatingModeText(currentOperatingMode)
    );

    lastOperatingDescription =
      currentOperatingMode;
  }


  // Fault
  if (haveFault &&
      currentFault != lastFaultDescription) {

    faultCode->setUnitAfterValue(
      getFaultText(currentFault)
    );

    lastFaultDescription =
      currentFault;
  }


  // System type
  if (haveSystemType &&
      currentSystemType != lastSystemDescription) {

    systemType->setUnitAfterValue(
      getSystemTypeText(currentSystemType)
    );

    lastSystemDescription =
      currentSystemType;
  }
}


// =====================================================
// FAST MODBUS
// co 5 sekund
// =====================================================

void readFastModbus() {

  uint16_t raw;


  // -------------------------------------------------
  // 99 - Outdoor Ambient Temperature
  // signed / 10
  // -------------------------------------------------

  if (readHR(99, raw)) {

    double value =
      ((int16_t)raw) / 10.0;

    outdoorTemp->setValue(value);

    Serial.print("Outdoor: ");
    Serial.print(value);
    Serial.println(" C");
  }


  // -------------------------------------------------
  // 101 - Flow Temperature
  // signed / 100
  // -------------------------------------------------

  if (readHR(101, raw)) {

    double value =
      ((int16_t)raw) / 100.0;

    flowTemp->setValue(value);

    Serial.print("Flow: ");
    Serial.print(value);
    Serial.println(" C");
  }


  // -------------------------------------------------
  // 103 - Return Temperature
  // signed / 100
  // -------------------------------------------------

  if (readHR(103, raw)) {

    double value =
      ((int16_t)raw) / 100.0;

    returnTemp->setValue(value);

    Serial.print("Return: ");
    Serial.print(value);
    Serial.println(" C");
  }


  // -------------------------------------------------
  // 73 - Heat Pump Frequency
  // Hz
  // -------------------------------------------------

  if (readHR(73, raw)) {

    hpFrequency->setValue(raw);

    Serial.print("HP Frequency: ");
    Serial.print(raw);
    Serial.println(" Hz");
  }


  // -------------------------------------------------
  // 127 - Heat Pump Master RUN
  // -------------------------------------------------

  if (readHR(127, raw)) {

    if (raw == 1) {
      hpRunning->set();
    }
    else {
      hpRunning->clear();
    }

    Serial.print("HP Run: ");
    Serial.println(raw);
  }


  // -------------------------------------------------
  // 67 - Defrost status
  //
  // 0 Normal
  // 1 Standby
  // 2 Defrost
  // 3 Waiting Restart
  // -------------------------------------------------

  if (readHR(67, raw)) {

    defrostStatus->setValue(raw);

    currentDefrost = raw;
    haveDefrost = true;

    Serial.print("Defrost status: ");
    Serial.println(raw);
  }


  // -------------------------------------------------
  // 26 - Operating Mode
  // -------------------------------------------------

  if (readHR(26, raw)) {

    operatingMode->setValue(raw);

    currentOperatingMode = raw;
    haveOperatingMode = true;

    Serial.print("Operating mode: ");
    Serial.println(raw);
  }


  // -------------------------------------------------
  // 9 - Fault Code
  // -------------------------------------------------

  if (readHR(9, raw)) {

    faultCode->setValue(raw);

    currentFault = raw;
    haveFault = true;

    Serial.print("Fault: 0x");
    Serial.println(raw, HEX);
  }


  Serial.println("-----------------------------");
}


// =====================================================
// SLOW MODBUS
// co 60 sekund
// =====================================================

void readSlowModbus() {

  uint16_t raw;


  // 10 - MelcoBEMS firmware

  if (readHR(10, raw)) {

    firmwareA1M->setValue(raw);

    Serial.print("Melco FW raw: ");
    Serial.println(raw);
  }


  // 11 - Modbus communication counter

  if (readHR(11, raw)) {

    modbusCounter->setValue(raw);

    Serial.print("Modbus counter: ");
    Serial.println(raw);
  }


  // 13 - System type

  if (readHR(13, raw)) {

    systemType->setValue(raw);

    currentSystemType = raw;
    haveSystemType = true;

    Serial.print("System type: ");
    Serial.println(raw);
  }
}


// =====================================================
// ROLLBACK - START
// =====================================================

void initRollbackState() {

  const esp_partition_t *running =
    esp_ota_get_running_partition();

  esp_ota_img_states_t state;


  Serial.print("Running partition: ");

  if (running) {
    Serial.println(running->label);
  }
  else {
    Serial.println("unknown");
  }


  if (esp_ota_get_state_partition(
        running,
        &state) == ESP_OK) {

    if (state ==
        ESP_OTA_IMG_PENDING_VERIFY) {

      otaPendingVerification = true;
      otaVerifyStart = millis();

      Serial.println();
      Serial.println("==============================");
      Serial.println("OTA PENDING_VERIFY");
      Serial.println("Rollback aktywny");
      Serial.println("Test firmware: 60 sekund");
      Serial.println("==============================");

      return;
    }
  }


  Serial.println("OTA: firmware VALID");
}


// =====================================================
// ROLLBACK - WALIDACJA
// =====================================================

void handleRollbackVerification() {

  if (!otaPendingVerification) {
    return;
  }


  unsigned long now = millis();


  if (now - otaVerifyStart <
      OTA_VERIFY_DELAY) {

    return;
  }


  if (now - lastOtaVerifyAttempt <
      5000) {

    return;
  }


  lastOtaVerifyAttempt = now;


  bool wifiOK =
    WiFi.status() == WL_CONNECTED;

  bool suplaOK =
    SuplaDevice.getCurrentStatus() ==
    STATUS_REGISTERED_AND_READY;


  Serial.print("OTA VERIFY | WiFi: ");
  Serial.print(
    wifiOK ? "OK" : "BRAK"
  );

  Serial.print(" | SUPLA: ");
  Serial.println(
    suplaOK ? "OK" : "BRAK"
  );


  if (wifiOK && suplaOK) {

    esp_err_t result =
      esp_ota_mark_app_valid_cancel_rollback();


    if (result == ESP_OK) {

      otaPendingVerification = false;

      Serial.println();
      Serial.println("==============================");
      Serial.println("OTA FIRMWARE VALID");
      Serial.println("Rollback anulowany");
      Serial.println("==============================");
    }
    else {

      Serial.print(
        "OTA VALID ERROR: "
      );

      Serial.println(
        esp_err_to_name(result)
      );
    }
  }
}


// =====================================================
// RECZNE OTA
// =====================================================

void performOTA() {

  if (otaInProgress) {
    return;
  }


  if (otaPendingVerification) {

    Serial.println(
      "OTA zablokowane: obecny firmware PENDING_VERIFY"
    );

    return;
  }


  if (WiFi.status() != WL_CONNECTED) {

    Serial.println(
      "OTA ERROR: brak WiFi"
    );

    return;
  }


  otaInProgress = true;
  otaLastProgress = -1;


  Serial.println();
  Serial.println("==============================");
  Serial.println("START OTA");
  Serial.print("Firmware: ");
  Serial.println(FW_VERSION);
  Serial.println("==============================");


  WiFiClientSecure client;

  client.setInsecure();


  httpUpdate.setFollowRedirects(
    HTTPC_FORCE_FOLLOW_REDIRECTS
  );

  httpUpdate.rebootOnUpdate(true);


  httpUpdate.onStart([]() {

    Serial.println(
      "OTA: pobieranie firmware..."
    );
  });


  httpUpdate.onProgress(
    [](int current, int total) {

      if (total <= 0) {
        return;
      }

      int percent =
        (current * 100) / total;


      if (percent != otaLastProgress &&
          percent % 10 == 0) {

        otaLastProgress = percent;

        Serial.print("OTA: ");
        Serial.print(percent);
        Serial.println("%");
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


  httpUpdate.onError([](int error) {

    Serial.print("OTA ERROR: ");
    Serial.println(error);

    Serial.print("Opis: ");

    Serial.println(
      httpUpdate.getLastErrorString()
    );
  });


  t_httpUpdate_return result =
    httpUpdate.update(
      client,
      OTA_URL
    );


  if (result == HTTP_UPDATE_FAILED) {

    Serial.println(
      "OTA zakonczone bledem"
    );
  }

  else if (
    result == HTTP_UPDATE_NO_UPDATES) {

    Serial.println(
      "OTA: brak aktualizacji"
    );
  }

  else if (
    result == HTTP_UPDATE_OK) {

    Serial.println(
      "OTA OK"
    );
  }


  otaInProgress = false;
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(1000);


  Serial.println();
  Serial.println("==============================");
  Serial.println("Mitsubishi MelcoBEMS SUPLA");
  Serial.print("Firmware: ");
  Serial.println(FW_VERSION);
  Serial.println("==============================");


  // -------------------------------------------------
  // MODBUS
  // -------------------------------------------------

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


  // -------------------------------------------------
  // WWW SUPLA
  // -------------------------------------------------

  new Supla::Html::DeviceInfo(
    &SuplaDevice
  );

  new Supla::Html::WifiParameters;

  new Supla::Html::ProtocolParameters;


  // =================================================
  // KANAL 0
  // Temperatura zewnetrzna
  // =================================================

  outdoorTemp =
    new Supla::Sensor::VirtualThermometer();

  outdoorTemp->setInitialCaption(
    "Temperatura zewnetrzna"
  );


  // =================================================
  // KANAL 1
  // Temperatura zasilania
  // =================================================

  flowTemp =
    new Supla::Sensor::VirtualThermometer();

  flowTemp->setInitialCaption(
    "Temperatura zasilania"
  );


  // =================================================
  // KANAL 2
  // Temperatura powrotu
  // =================================================

  returnTemp =
    new Supla::Sensor::VirtualThermometer();

  returnTemp->setInitialCaption(
    "Temperatura powrotu"
  );


  // =================================================
  // KANAL 3
  // Czestotliwosc HP
  // =================================================

  hpFrequency =
    new Supla::Sensor::GeneralPurposeMeasurement();

  hpFrequency->setInitialCaption(
    "Czestotliwosc HP"
  );

  hpFrequency->setDefaultUnitAfterValue(
    "Hz"
  );

  hpFrequency->setDefaultValuePrecision(0);


  // =================================================
  // KANAL 4
  // Praca pompy ciepla
  // =================================================

  hpRunning =
    new Supla::Sensor::VirtualBinary();

  hpRunning->setInitialCaption(
    "Praca pompy ciepla"
  );


  // =================================================
  // KANAL 5
  // Kod bledu
  // =================================================

  faultCode =
    new Supla::Sensor::GeneralPurposeMeasurement();

  faultCode->setInitialCaption(
    "Kod bledu"
  );

  faultCode->setDefaultValuePrecision(0);


  // =================================================
  // KANAL 6
  // Firmware A1M
  // =================================================

  firmwareA1M =
    new Supla::Sensor::GeneralPurposeMeasurement();

  firmwareA1M->setInitialCaption(
    "Firmware MelcoBEMS"
  );

  firmwareA1M->setDefaultValuePrecision(0);


  // =================================================
  // KANAL 7
  // Licznik Modbus
  // =================================================

  modbusCounter =
    new Supla::Sensor::GeneralPurposeMeasurement();

  modbusCounter->setInitialCaption(
    "Licznik Modbus"
  );

  modbusCounter->setDefaultValuePrecision(0);


  // =================================================
  // KANAL 8
  // Typ systemu
  // =================================================

  systemType =
    new Supla::Sensor::GeneralPurposeMeasurement();

  systemType->setInitialCaption(
    "Typ systemu"
  );

  systemType->setDefaultValuePrecision(0);


  // =================================================
  // KANAL 9
  // OTA
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
  // KANAL 10
  // Odszranianie
  // =================================================

  defrostStatus =
    new Supla::Sensor::GeneralPurposeMeasurement();

  defrostStatus->setInitialCaption(
    "Odszranianie"
  );

  defrostStatus->setDefaultValuePrecision(0);


  // =================================================
  // KANAL 11
  // Tryb pracy
  // =================================================

  operatingMode =
    new Supla::Sensor::GeneralPurposeMeasurement();

  operatingMode->setInitialCaption(
    "Tryb pracy"
  );

  operatingMode->setDefaultValuePrecision(0);


  // -------------------------------------------------
  // DEVICE
  // -------------------------------------------------

  SuplaDevice.setName(
    "Mitsubishi MelcoBEMS"
  );

  SuplaDevice.setSwVersion(
    FW_VERSION
  );

  SuplaDevice.setInitialMode(
    Supla::InitialMode::StartInCfgMode
  );


  SuplaDevice.begin();


  // -------------------------------------------------
  // ROLLBACK
  // -------------------------------------------------

  initRollbackState();


  Serial.print(
    "Free sketch space: "
  );

  Serial.println(
    ESP.getFreeSketchSpace()
  );
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  SuplaDevice.iterate();


  // -------------------------------------------------
  // Rollback
  // -------------------------------------------------

  handleRollbackVerification();


  // -------------------------------------------------
  // Historia + status text hack
  // -------------------------------------------------

  updateStatusDescriptions();


  // -------------------------------------------------
  // OTA z SUPLA
  // -------------------------------------------------

  if (otaTrigger->isOn() &&
      !otaInProgress) {

    Serial.println();
    Serial.println(
      "OTA wyzwolone z SUPLA"
    );


    // Przycisk zachowuje sie chwilowo.
    otaTrigger->turnOff();

    SuplaDevice.iterate();

    delay(250);


    performOTA();
  }


  // -------------------------------------------------
  // FAST MODBUS
  // -------------------------------------------------

  if (!otaInProgress &&
      millis() - lastFastPoll >=
      FAST_POLL_INTERVAL) {

    lastFastPoll = millis();

    readFastModbus();
  }


  // -------------------------------------------------
  // SLOW MODBUS
  //
  // pierwszy raz po ok. 10 s,
  // potem co 60 s
  // -------------------------------------------------

  if (!otaInProgress) {

    if (
      (firstSlowPoll &&
       millis() >= 10000)

      ||

      (!firstSlowPoll &&
       millis() - lastSlowPoll >=
       SLOW_POLL_INTERVAL)
    ) {

      firstSlowPoll = false;
      lastSlowPoll = millis();

      readSlowModbus();
    }
  }
}
