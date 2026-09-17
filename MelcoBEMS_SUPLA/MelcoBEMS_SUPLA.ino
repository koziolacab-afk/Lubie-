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


// =====================================================
// WERSJA FIRMWARE
// =====================================================

#define FW_VERSION "1.0.0"


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
// KANAŁY SUPLA
// =====================================================

// Temperatury
Supla::Sensor::VirtualThermometer *outdoorTemp;
Supla::Sensor::VirtualThermometer *flowTemp;
Supla::Sensor::VirtualThermometer *returnTemp;

// KPOP / GPM
Supla::Sensor::GeneralPurposeMeasurement *hpFrequency;
Supla::Sensor::GeneralPurposeMeasurement *faultCode;
Supla::Sensor::GeneralPurposeMeasurement *firmwareA1M;
Supla::Sensor::GeneralPurposeMeasurement *modbusCounter;
Supla::Sensor::GeneralPurposeMeasurement *systemType;

// Stan pompy
Supla::Sensor::VirtualBinary *hpRunning;


// =====================================================
// CZAS ODCZYTU MODBUS
// =====================================================

unsigned long lastModbusRead = 0;

const unsigned long MODBUS_READ_INTERVAL = 5000;


// =====================================================
// ODCZYT HOLDING REGISTER
// =====================================================

bool readHR(uint16_t address, uint16_t &value) {

  uint8_t result = node.readHoldingRegisters(address, 1);

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
// ODCZYT MODBUS -> SUPLA
// =====================================================

void readModbus() {

  uint16_t raw;


  // -------------------------------------------------
  // 99 - Outdoor Ambient Temperature
  // int16, x10
  // -------------------------------------------------

  if (readHR(99, raw)) {

    int16_t value = (int16_t)raw;

    double temp = value / 10.0;

    outdoorTemp->setValue(temp);

    Serial.print("Outdoor: ");
    Serial.print(temp);
    Serial.println(" C");
  }


  // -------------------------------------------------
  // 101 - Flow Temperature
  // int16, x100
  // -------------------------------------------------

  if (readHR(101, raw)) {

    int16_t value = (int16_t)raw;

    double temp = value / 100.0;

    flowTemp->setValue(temp);

    Serial.print("Flow: ");
    Serial.print(temp);
    Serial.println(" C");
  }


  // -------------------------------------------------
  // 103 - Return Temperature
  // int16, x100
  // -------------------------------------------------

  if (readHR(103, raw)) {

    int16_t value = (int16_t)raw;

    double temp = value / 100.0;

    returnTemp->setValue(temp);

    Serial.print("Return: ");
    Serial.print(temp);
    Serial.println(" C");
  }


  // -------------------------------------------------
  // 73 - Heat Pump Frequency
  // uint16, Hz
  // -------------------------------------------------

  if (readHR(73, raw)) {

    hpFrequency->setValue(raw);

    Serial.print("Frequency: ");
    Serial.print(raw);
    Serial.println(" Hz");
  }


  // -------------------------------------------------
  // 127 - Heat Pump RUN / STOP
  // 0 = STOP
  // 1 = RUN
  // -------------------------------------------------

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


  // -------------------------------------------------
  // 9 - Fault / Error Code
  //
  // 0x8000 = brak bledu
  // 0x6999 = problem komunikacji
  // -------------------------------------------------

  if (readHR(9, raw)) {

    faultCode->setValue(raw);

    Serial.print("Fault code: 0x");
    Serial.println(raw, HEX);
  }


  // -------------------------------------------------
  // 10 - Firmware MelcoBEMS
  // -------------------------------------------------

  if (readHR(10, raw)) {

    firmwareA1M->setValue(raw);

    Serial.print("Melco firmware raw: ");
    Serial.println(raw);
  }


  // -------------------------------------------------
  // 11 - Modbus communication counter
  // -------------------------------------------------

  if (readHR(11, raw)) {

    modbusCounter->setValue(raw);

    Serial.print("Modbus counter: ");
    Serial.println(raw);
  }


  // -------------------------------------------------
  // 13 - System Type
  //
  // 0   = ATA
  // 1   = ATW
  // 2   = Lossnay
  // 255 = undetermined
  // -------------------------------------------------

  if (readHR(13, raw)) {

    systemType->setValue(raw);

    Serial.print("System type: ");
    Serial.println(raw);
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
  // STRONA KONFIGURACYJNA SUPLA
  // -------------------------------------------------

  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;


  // -------------------------------------------------
  // TEMPERATURY
  // -------------------------------------------------

  outdoorTemp =
    new Supla::Sensor::VirtualThermometer();

  flowTemp =
    new Supla::Sensor::VirtualThermometer();

  returnTemp =
    new Supla::Sensor::VirtualThermometer();


  // -------------------------------------------------
  // CZESTOTLIWOSC
  // -------------------------------------------------

  hpFrequency =
    new Supla::Sensor::GeneralPurposeMeasurement();

  hpFrequency->setDefaultUnitAfterValue("Hz");
  hpFrequency->setDefaultValuePrecision(0);


  // -------------------------------------------------
  // RUN / STOP
  // -------------------------------------------------

  hpRunning =
    new Supla::Sensor::VirtualBinary();


  // -------------------------------------------------
  // FAULT CODE
  // -------------------------------------------------

  faultCode =
    new Supla::Sensor::GeneralPurposeMeasurement();

  faultCode->setDefaultValuePrecision(0);


  // -------------------------------------------------
  // MELCO FIRMWARE
  // -------------------------------------------------

  firmwareA1M =
    new Supla::Sensor::GeneralPurposeMeasurement();

  firmwareA1M->setDefaultValuePrecision(0);


  // -------------------------------------------------
  // MODBUS COUNTER
  // -------------------------------------------------

  modbusCounter =
    new Supla::Sensor::GeneralPurposeMeasurement();

  modbusCounter->setDefaultValuePrecision(0);


  // -------------------------------------------------
  // SYSTEM TYPE
  // -------------------------------------------------

  systemType =
    new Supla::Sensor::GeneralPurposeMeasurement();

  systemType->setDefaultValuePrecision(0);


  // -------------------------------------------------
  // DANE URZADZENIA
  // -------------------------------------------------

  SuplaDevice.setName("Mitsubishi MelcoBEMS");

  SuplaDevice.setSwVersion(FW_VERSION);


  // Przy pierwszym uruchomieniu / factory reset:
  // uruchomi AP i strone konfiguracji.
  //
  // Gdy konfiguracja jest juz zapisana w LittleFS:
  // normalnie polaczy sie z WiFi i SUPLA.

  SuplaDevice.setInitialMode(
    Supla::InitialMode::StartInCfgMode
  );


  // -------------------------------------------------
  // START SUPLA
  // -------------------------------------------------

  SuplaDevice.begin();

  Serial.println("SUPLA uruchomiona");
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // SUPLA musi byc wywolywana bardzo czesto
  SuplaDevice.iterate();


  // Odczyt Modbus co 5 sekund
  if (millis() - lastModbusRead >= MODBUS_READ_INTERVAL) {

    lastModbusRead = millis();

    readModbus();
  }
}