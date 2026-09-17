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


// =====================================================
// WERSJA
// =====================================================

#define FW_VERSION "1.0.1"


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


// =====================================================
// WAZNE - ODRACZAMY AUTOMATYCZNE ZATWIERDZENIE OTA
// =====================================================

// Arduino ESP32 normalnie moglby zatwierdzic firmware
// bardzo wczesnie podczas startu.
//
// My chcemy sami zatwierdzic firmware dopiero wtedy,
// gdy przez 60 sekund dziala i polaczy sie z SUPLA.

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
// KANALY
// =====================================================

Supla::Sensor::VirtualThermometer *outdoorTemp;
Supla::Sensor::VirtualThermometer *flowTemp;
Supla::Sensor::VirtualThermometer *returnTemp;

Supla::Sensor::GeneralPurposeMeasurement *hpFrequency;
Supla::Sensor::GeneralPurposeMeasurement *faultCode;
Supla::Sensor::GeneralPurposeMeasurement *firmwareA1M;
Supla::Sensor::GeneralPurposeMeasurement *modbusCounter;
Supla::Sensor::GeneralPurposeMeasurement *systemType;

Supla::Sensor::VirtualBinary *hpRunning;

Supla::Control::VirtualRelay *otaTrigger;


// =====================================================
// MODBUS TIMER
// =====================================================

unsigned long lastModbusRead = 0;

const unsigned long MODBUS_READ_INTERVAL = 5000;


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

  Serial.print("Blad Modbus addr ");
  Serial.print(address);
  Serial.print(" -> 0x");
  Serial.println(result, HEX);

  return false;
}


// =====================================================
// SPRAWDZENIE STANU ROLLBACK
// =====================================================

void initRollbackState() {

  const esp_partition_t *running =
    esp_ota_get_running_partition();

  esp_ota_img_states_t state;

  if (esp_ota_get_state_partition(
        running,
        &state) == ESP_OK) {

    if (state == ESP_OTA_IMG_PENDING_VERIFY) {

      otaPendingVerification = true;
      otaVerifyStart = millis();

      Serial.println();
      Serial.println("================================");
      Serial.println("OTA: NOWY FIRMWARE");
      Serial.println("Stan: PENDING_VERIFY");
      Serial.println("Rollback jest aktywny");
      Serial.println("Czekam 60 s na test firmware");
      Serial.println("================================");

      return;
    }
  }

  Serial.println("OTA: firmware nie wymaga weryfikacji");
}


// =====================================================
// ZATWIERDZENIE NOWEGO FIRMWARE
// =====================================================

void handleRollbackVerification() {

  if (!otaPendingVerification) {
    return;
  }

  unsigned long now = millis();


  // Najpierw musi przezyc minimum 60 sekund
  if (now - otaVerifyStart < OTA_VERIFY_DELAY) {
    return;
  }


  // Nie sprawdzamy co kazda petle
  if (now - lastOtaVerifyAttempt < 5000) {
    return;
  }

  lastOtaVerifyAttempt = now;


  bool wifiOK =
    WiFi.status() == WL_CONNECTED;

  bool suplaOK =
    SuplaDevice.getCurrentStatus() ==
    STATUS_REGISTERED_AND_READY;


  Serial.print("OTA VERIFY | WiFi: ");
  Serial.print(wifiOK ? "OK" : "BRAK");

  Serial.print(" | SUPLA: ");
  Serial.println(suplaOK ? "OK" : "BRAK");


  // Firmware zatwierdzamy dopiero gdy:
  //
  // 1. przepracowal minimum 60 sekund
  // 2. WiFi dziala
  // 3. polaczyl sie z SUPLA

  if (wifiOK && suplaOK) {

    esp_err_t result =
      esp_ota_mark_app_valid_cancel_rollback();

    if (result == ESP_OK) {

      otaPendingVerification = false;

      Serial.println();
      Serial.println("================================");
      Serial.println("OTA: FIRMWARE ZATWIERDZONY");
      Serial.println("Rollback anulowany");
      Serial.println("Nowa wersja jest teraz VALID");
      Serial.println("================================");
    }
    else {

      Serial.print("Blad zatwierdzenia OTA: ");
      Serial.println(esp_err_to_name(result));
    }
  }
}


// =====================================================
// RECZNA AKTUALIZACJA OTA
// =====================================================

void performOTA() {

  if (otaInProgress) {
    return;
  }


  // Nie pozwalamy instalowac kolejnego firmware,
  // dopoki obecny firmware nie zostal zatwierdzony.

  if (otaPendingVerification) {

    Serial.println(
      "OTA zablokowane - obecny firmware jest jeszcze PENDING_VERIFY"
    );

    return;
  }


  if (WiFi.status() != WL_CONNECTED) {

    Serial.println(
      "OTA BLAD: brak WiFi"
    );

    return;
  }


  otaInProgress = true;

  Serial.println();
  Serial.println("================================");
  Serial.println("START OTA");
  Serial.print("Firmware: ");
  Serial.println(FW_VERSION);
  Serial.print("URL: ");
  Serial.println(OTA_URL);
  Serial.println("================================");


  WiFiClientSecure client;

  client.setInsecure();


  // GitHub Release robi przekierowanie
  // do serwera z plikiem firmware.bin.

  httpUpdate.setFollowRedirects(
    HTTPC_FORCE_FOLLOW_REDIRECTS
  );


  // Po udanym zapisie reboot.
  httpUpdate.rebootOnUpdate(true);


  httpUpdate.onStart([]() {

    Serial.println("OTA: pobieranie...");
  });


  httpUpdate.onProgress(
    [](int current, int total) {

      if (total <= 0) {
        return;
      }

      int percent =
        (current * 100) / total;

      static int lastPercent = -1;

      if (percent != lastPercent &&
          percent % 10 == 0) {

        lastPercent = percent;

        Serial.print("OTA: ");
        Serial.print(percent);
        Serial.println("%");
      }
    }
  );


  httpUpdate.onEnd([]() {

    Serial.println("OTA: zapis zakonczony");
    Serial.println("Restart...");
  });


  httpUpdate.onError([](int error) {

    Serial.print("OTA BLAD: ");
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


  switch (result) {

    case HTTP_UPDATE_FAILED:

      Serial.println(
        "OTA zakonczone bledem"
      );

      break;


    case HTTP_UPDATE_NO_UPDATES:

      Serial.println(
        "OTA: brak aktualizacji"
      );

      break;


    case HTTP_UPDATE_OK:

      Serial.println(
        "OTA OK"
      );

      break;
  }


  otaInProgress = false;
}


// =====================================================
// MODBUS -> SUPLA
// =====================================================

void readModbus() {

  uint16_t raw;


  // 99 - Outdoor temperature x10

  if (readHR(99, raw)) {

    int16_t value =
      (int16_t)raw;

    double temp =
      value / 10.0;

    outdoorTemp->setValue(temp);

    Serial.print("Outdoor: ");
    Serial.print(temp);
    Serial.println(" C");
  }


  // 101 - Flow temperature x100

  if (readHR(101, raw)) {

    int16_t value =
      (int16_t)raw;

    double temp =
      value / 100.0;

    flowTemp->setValue(temp);

    Serial.print("Flow: ");
    Serial.print(temp);
    Serial.println(" C");
  }


  // 103 - Return temperature x100

  if (readHR(103, raw)) {

    int16_t value =
      (int16_t)raw;

    double temp =
      value / 100.0;

    returnTemp->setValue(temp);

    Serial.print("Return: ");
    Serial.print(temp);
    Serial.println(" C");
  }


  // 73 - Heat Pump Frequency

  if (readHR(73, raw)) {

    hpFrequency->setValue(raw);

    Serial.print("Frequency: ");
    Serial.print(raw);
    Serial.println(" Hz");
  }


  // 127 - RUN / STOP

  if (readHR(127, raw)) {

    if (raw == 1) {

      hpRunning->set();

    }
    else {

      hpRunning->clear();
    }

    Serial.print("HP Running: ");
    Serial.println(raw);
  }


  // 9 - Fault

  if (readHR(9, raw)) {

    faultCode->setValue(raw);

    Serial.print("Fault: 0x");
    Serial.println(raw, HEX);
  }


  // 10 - MelcoBEMS firmware

  if (readHR(10, raw)) {

    firmwareA1M->setValue(raw);
  }


  // 11 - Modbus counter

  if (readHR(11, raw)) {

    modbusCounter->setValue(raw);
  }


  // 13 - System type

  if (readHR(13, raw)) {

    systemType->setValue(raw);
  }


  Serial.println("-----------------------------");
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


  // MODBUS

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


  // SUPLA WWW

  new Supla::Html::DeviceInfo(
    &SuplaDevice
  );

  new Supla::Html::WifiParameters;

  new Supla::Html::ProtocolParameters;


  // TEMPERATURY

  outdoorTemp =
    new Supla::Sensor::VirtualThermometer();

  flowTemp =
    new Supla::Sensor::VirtualThermometer();

  returnTemp =
    new Supla::Sensor::VirtualThermometer();


  // FREQUENCY

  hpFrequency =
    new Supla::Sensor::GeneralPurposeMeasurement();

  hpFrequency->
    setDefaultUnitAfterValue("Hz");

  hpFrequency->
    setDefaultValuePrecision(0);


  // RUN

  hpRunning =
    new Supla::Sensor::VirtualBinary();


  // FAULT

  faultCode =
    new Supla::Sensor::GeneralPurposeMeasurement();

  faultCode->
    setDefaultValuePrecision(0);


  // MELCO FW

  firmwareA1M =
    new Supla::Sensor::GeneralPurposeMeasurement();

  firmwareA1M->
    setDefaultValuePrecision(0);


  // MODBUS COUNTER

  modbusCounter =
    new Supla::Sensor::GeneralPurposeMeasurement();

  modbusCounter->
    setDefaultValuePrecision(0);


  // SYSTEM TYPE

  systemType =
    new Supla::Sensor::GeneralPurposeMeasurement();

  systemType->
    setDefaultValuePrecision(0);


  // OTA BUTTON

  otaTrigger =
    new Supla::Control::VirtualRelay();

  otaTrigger->setDefaultFunction(
    SUPLA_CHANNELFNC_POWERSWITCH
  );

  otaTrigger->setDefaultStateOff();


  // DEVICE

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


  Serial.println("SUPLA uruchomiona");


  // Sprawdz czy jestesmy po OTA
  // i czy trzeba zatwierdzic firmware.

  initRollbackState();


  Serial.print("Free sketch space: ");
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
  // WALIDACJA NOWEGO FIRMWARE
  // -------------------------------------------------

  handleRollbackVerification();


  // -------------------------------------------------
  // PRZYCISK OTA
  // -------------------------------------------------

  if (otaTrigger->isOn() &&
      !otaInProgress) {

    Serial.println();
    Serial.println(
      "OTA wyzwolone z SUPLA"
    );


    // Wylaczamy przycisk od razu.

    otaTrigger->turnOff();

    SuplaDevice.iterate();

    delay(250);


    performOTA();
  }


  // -------------------------------------------------
  // MODBUS
  // -------------------------------------------------

  if (!otaInProgress &&
      millis() - lastModbusRead >=
      MODBUS_READ_INTERVAL) {

    lastModbusRead = millis();

    readModbus();
  }
}