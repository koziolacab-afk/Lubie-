#include "config_after_migration.h"

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <esp_ota_ops.h>
#include <memory>
#include <new>
#include <string.h>
#include <supla-common/proto.h>

namespace {
// Byte-for-byte layout of the NVS "trial" record written by firmware 1.0.24.
struct ConfirmedMigration {
  uint8_t state;
  char ssid[MAX_SSID_SIZE];
  char password[MAX_WIFI_PASSWORD_SIZE];
  char server[SUPLA_SERVER_NAME_MAXSIZE];
  char email[SUPLA_EMAIL_MAXSIZE];
  char guid[SUPLA_GUID_SIZE];
  char authKey[SUPLA_AUTHKEY_SIZE];
};
static_assert(sizeof(ConfirmedMigration) == 451, "Keep the 1.0.24 NVS layout");
constexpr uint8_t ConfirmedState = 3;
constexpr const char *ImportedKey = "netMigrated";

template <size_t Size>
bool validText(const char (&value)[Size]) {
  return value[0] && strnlen(value, Size) < Size;
}

bool matchesFile(const char *path, const uint8_t *expected, size_t size) {
  File file = LittleFS.open(path, "r");
  if (!file || file.size() != size) return false;

  uint8_t chunk[256];
  for (size_t offset = 0; offset < size;) {
    const size_t count = size - offset < sizeof(chunk) ? size - offset : sizeof(chunk);
    if (file.read(chunk, count) != count || memcmp(chunk, expected + offset, count) != 0) {
      return false;
    }
    offset += count;
  }
  return true;
}
}

bool ConfigAfterMigration::init() {
  const bool loaded = LittleFsConfig::init();
  Preferences prefs;
  if (!prefs.begin("net-migration", true)) return loaded;

  ConfirmedMigration record{};
  const bool found = prefs.getBytesLength("trial") == sizeof(record) &&
    prefs.getBytes("trial", &record, sizeof(record)) == sizeof(record);
  prefs.end();
  if (!found || record.state != ConfirmedState) return loaded;

  uint8_t marker = 0;
  const bool imported = loaded && getUInt8(ImportedKey, &marker) && marker == 1;
  if (!imported) {
    const bool valid = validText(record.ssid) && validText(record.password) &&
      validText(record.server) && validText(record.email) && strchr(record.email, '@');
    const bool applied = valid &&
      setWiFiSSID(record.ssid) && setWiFiPassword(record.password) &&
      setAltWiFiSSID("") && setAltWiFiPassword("") &&
      setSuplaServer(record.server) && setSuplaServerPort(2016) &&
      setEmail(record.email) && setGUID(record.guid) && setAuthKey(record.authKey) &&
      setDeviceMode(Supla::DEVICE_MODE_NORMAL) && setUInt8(ImportedKey, 1);
    if (!applied) {
      Serial.println("Konfiguracja: nie udalo sie odczytac danych po migracji");
      esp_ota_mark_app_invalid_rollback_and_reboot();
      initResult = false;
      return false;
    }
  }

  // LittleFsConfig::commit has no return value. Check both files before consuming NVS.
  if (!imported || !verifyStoredConfig()) commit();
  if (!verifyStoredConfig()) {
    Serial.println("Konfiguracja: zapis niepotwierdzony, powrot do wersji migracyjnej");
    esp_ota_mark_app_invalid_rollback_and_reboot();
    initResult = false;
    return false;
  }

  // Retrying after an interrupted cleanup must preserve subsequent local edits.
  if (prefs.begin("net-migration", false)) {
    prefs.remove("trial");
    prefs.end();
  }
  Serial.println("Konfiguracja: nowe dane Wi-Fi i SUPLA zapisane w LittleFS");
  initResult = true;
  return true;
}

bool ConfigAfterMigration::verifyStoredConfig() {
  std::unique_ptr<uint8_t[]> expected(new (std::nothrow) uint8_t[configMaxSize]);
  if (!expected) return false;

  const size_t size = serializeToMemory(expected.get(), configMaxSize);
  if (!size || !LittleFS.begin(false)) return false;
  const bool verified = matchesFile("/supla-dev.cfg", expected.get(), size) &&
    matchesFile("/supla-dev.cfg.bak", expected.get(), size);
  LittleFS.end();
  return verified;
}
