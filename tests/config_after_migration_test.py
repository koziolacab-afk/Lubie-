"""Exercise the real config importer with interrupted writes and later local edits."""
from pathlib import Path
import subprocess
import tempfile


root = Path(__file__).resolve().parents[1]
headers = {
    "Arduino.h": r"""
#pragma once
#include <cstdint>
#include <cstring>
struct SerialStub { void println(const char *) {} };
static SerialStub Serial;
""",
    "LittleFS.h": r"""
#pragma once
#include <algorithm>
#include <cstring>
#include <map>
#include <string>
#include <vector>
using Bytes = std::vector<uint8_t>;
static std::map<std::string, Bytes> files;
class File {
 public:
  explicit File(const char *path) : path(path) {}
  explicit operator bool() const { return files.count(path) != 0; }
  size_t size() const { return files.at(path).size(); }
  size_t read(uint8_t *out, size_t count) {
    count = std::min(count, size() - offset);
    memcpy(out, files.at(path).data() + offset, count);
    offset += count;
    return count;
  }
 private:
  std::string path;
  size_t offset = 0;
};
struct FakeFS {
  bool begin(bool) { return true; }
  void end() {}
  File open(const char *path, const char *) { return File(path); }
};
static FakeFS LittleFS;
""",
    "Preferences.h": r"""
#pragma once
#include <LittleFS.h>
static Bytes trial;
static bool cleanupFails = false;
class Preferences {
 public:
  bool begin(const char *, bool) { return !trial.empty(); }
  size_t getBytesLength(const char *) { return trial.size(); }
  size_t getBytes(const char *, void *out, size_t size) {
    memcpy(out, trial.data(), size);
    return size;
  }
  void end() {}
  bool remove(const char *) {
    if (cleanupFails) return false;
    trial.clear();
    return true;
  }
};
""",
    "esp_ota_ops.h": r"""
#pragma once
struct Rollback {};
inline int esp_ota_mark_app_invalid_rollback_and_reboot() { throw Rollback{}; }
""",
    "supla-common/proto.h": r"""
#pragma once
#define SUPLA_GUID_SIZE 16
#define SUPLA_AUTHKEY_SIZE 16
#define SUPLA_SERVER_NAME_MAXSIZE 65
#define SUPLA_EMAIL_MAXSIZE 256
""",
    "supla/storage/littlefs_config.h": r"""
#pragma once
#include <LittleFS.h>
#define MAX_SSID_SIZE 33
#define MAX_WIFI_PASSWORD_SIZE 64
namespace Supla {
enum DeviceMode { DEVICE_MODE_NORMAL };
using Values = std::map<std::string, std::string>;
static Values disk;
static bool loaded = true;
static bool setterFails = false;
static bool commitFails = false;
static bool interruptCommit = false;
static int writes = 0;
struct PowerLoss {};
class LittleFsConfig {
 public:
  explicit LittleFsConfig(int size) : configMaxSize(size) {}
  virtual ~LittleFsConfig() {}
  virtual bool init() { values = disk; initResult = loaded; return loaded; }
  void commit() {
    ++writes;
    if (commitFails) return;
    uint8_t serialized[16384];
    const size_t size = serializeToMemory(serialized, sizeof(serialized));
    disk = values;
    files["/supla-dev.cfg"] = Bytes(serialized, serialized + size);
    if (interruptCommit) throw PowerLoss{};
    files["/supla-dev.cfg.bak"] = Bytes(serialized, serialized + size);
    loaded = true;
  }
  bool setString(const char *key, const char *value) {
    if (setterFails) return false;
    values[key] = value;
    return true;
  }
  bool setUInt8(const char *key, uint8_t value) {
    return setString(key, std::to_string(value).c_str());
  }
  bool getUInt8(const char *key, uint8_t *value) {
    if (!values.count(key)) return false;
    *value = static_cast<uint8_t>(std::stoi(values.at(key)));
    return true;
  }
  bool setWiFiSSID(const char *value) { return setString("ssid", value); }
  bool setWiFiPassword(const char *value) { return setString("password", value); }
  bool setAltWiFiSSID(const char *value) { return setString("altssid", value); }
  bool setAltWiFiPassword(const char *value) { return setString("altpassword", value); }
  bool setSuplaServer(const char *value) { return setString("server", value); }
  bool setSuplaServerPort(int value) { return setString("port", std::to_string(value).c_str()); }
  bool setEmail(const char *value) { return setString("email", value); }
  bool setGUID(const char *value) { values["guid"] = std::string(value, 16); return true; }
  bool setAuthKey(const char *value) { values["authkey"] = std::string(value, 16); return true; }
  bool setDeviceMode(DeviceMode) { return setString("mode", "normal"); }
  size_t serializeToMemory(uint8_t *out, size_t maxSize) {
    std::string serialized;
    for (const auto &entry : values) {
      serialized += entry.first + "=" + entry.second + ";";
    }
    if (serialized.size() > maxSize) return 0;
    memcpy(out, serialized.data(), serialized.size());
    return serialized.size();
  }
  Values values;
 protected:
  int configMaxSize;
  bool initResult = false;
};
}
""",
}
fixture = r"""
#include <cassert>
#include "config_after_migration.cpp"

void reset() {
  files.clear();
  trial.clear();
  cleanupFails = false;
  Supla::loaded = true;
  Supla::setterFails = false;
  Supla::commitFails = false;
  Supla::interruptCommit = false;
  Supla::writes = 0;
  Supla::disk = {
    {"ssid", "old-network"}, {"password", "old-password"},
    {"email", "old@example.test"}, {"server", "old.example.test"},
    {"guid", std::string(16, 'o')}, {"authkey", std::string(16, 'a')},
    {"channels", "keep-functions-and-captions"},
    {"thermostats", "keep-setpoints"}, {"history", std::string(800, 'h')}
  };
}

void confirmed(uint8_t state = 3) {
  ConfirmedMigration record{};
  record.state = state;
  strcpy(record.ssid, "new-network");
  strcpy(record.password, "new-password");
  strcpy(record.server, "new.example.test");
  strcpy(record.email, "new@example.test");
  memset(record.guid, 0x41, sizeof(record.guid));
  memset(record.authKey, 0x42, sizeof(record.authKey));
  const auto *start = reinterpret_cast<const uint8_t *>(&record);
  trial.assign(start, start + sizeof(record));
}

void checkNew(const ConfigAfterMigration &config) {
  const auto &v = config.values;
  assert(v.at("ssid") == "new-network" && v.at("password") == "new-password");
  assert(v.at("email") == "new@example.test" && v.at("server") == "new.example.test");
  assert(v.at("port") == "2016");
  assert(v.at("guid") == std::string(16, 0x41));
  assert(v.at("authkey") == std::string(16, 0x42));
  assert(v.at("altssid").empty() && v.at("altpassword").empty());
  assert(v.at("channels") == "keep-functions-and-captions");
  assert(v.at("thermostats") == "keep-setpoints");
  assert(v.at("history") == std::string(800, 'h'));
}

template<class Exception, class Action>
void expectException(Action action) {
  try { action(); }
  catch (const Exception &) { return; }
  assert(false);
}

int main() {
  reset();
  {
    ConfigAfterMigration ordinary(16384);
    assert(ordinary.init() && Supla::writes == 0);
    assert(ordinary.values.at("ssid") == "old-network");
  }
  for (uint8_t state : {uint8_t(1), uint8_t(2), uint8_t(4)}) {
    reset();
    confirmed(state);
    ConfigAfterMigration unconfirmed(16384);
    assert(unconfirmed.init() && Supla::writes == 0);
    assert(!trial.empty());
  }
  reset();
  confirmed();
  {
    ConfigAfterMigration imported(16384);
    assert(imported.init());
    checkNew(imported);
    assert(trial.empty() && Supla::writes == 1);
    ConfigAfterMigration rebooted(16384);
    assert(rebooted.init());
    checkNew(rebooted);
    assert(Supla::writes == 1);
    // A plain pre-migration storage class now sees the new identity as well.
    Supla::LittleFsConfig previousFirmware(16384);
    assert(previousFirmware.init());
    assert(previousFirmware.values == rebooted.values);
  }
  reset();
  confirmed();
  cleanupFails = true;
  {
    ConfigAfterMigration imported(16384);
    assert(imported.init());
    assert(!trial.empty());
    imported.setWiFiPassword("changed-on-site");
    imported.commit();
    cleanupFails = false;
    ConfigAfterMigration rebooted(16384);
    assert(rebooted.init());
    assert(rebooted.values.at("password") == "changed-on-site");
    assert(Supla::writes == 2 && trial.empty());
  }
  reset();
  confirmed();
  Supla::commitFails = true;
  {
    ConfigAfterMigration failed(16384);
    expectException<Rollback>([&] { failed.init(); });
    assert(!trial.empty());
    Supla::commitFails = false;
    ConfigAfterMigration retry(16384);
    assert(retry.init());
    checkNew(retry);
  }
  reset();
  confirmed();
  Supla::setterFails = true;
  {
    ConfigAfterMigration failed(16384);
    expectException<Rollback>([&] { failed.init(); });
    assert(!trial.empty() && Supla::writes == 0);
  }
  reset();
  confirmed();
  Supla::interruptCommit = true;
  {
    ConfigAfterMigration interrupted(16384);
    expectException<Supla::PowerLoss>([&] { interrupted.init(); });
    assert(!trial.empty());
    Supla::interruptCommit = false;
    ConfigAfterMigration retry(16384);
    assert(retry.init());
    checkNew(retry);
    assert(trial.empty() && Supla::writes == 2);
  }
  reset();
  confirmed();
  Supla::loaded = false;
  {
    ConfigAfterMigration recovered(16384);
    assert(recovered.init());
    checkNew(recovered);
  }
}
"""

with tempfile.TemporaryDirectory() as directory:
    base = Path(directory)
    for name, content in headers.items():
        path = base / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    source = base / "config_after_migration_test.cpp"
    binary = base / "config_after_migration_test"
    source.write_text(fixture, encoding="utf-8")
    subprocess.run([
        "g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
        "-I", str(base), "-I", str(root / "MelcoBEMS_SUPLA"),
        str(source), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
