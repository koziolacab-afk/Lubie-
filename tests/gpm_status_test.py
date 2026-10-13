"""Exercise actual fault publishing and cleanup/reconnect code from the sketch."""
from pathlib import Path
import re
import subprocess
import tempfile


source = (Path(__file__).resolve().parents[1] /
          "MelcoBEMS_SUPLA/MelcoBEMS_SUPLA.ino").read_text(encoding="utf-8")


def block(marker):
    start = source.index(marker)
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


compact = re.sub(r"\s+", "", source)
assert "constuint16_tFAULT_DECIMAL_REGISTER=12;" in compact
assert "{FAULT_DECIMAL_REGISTER,5000,{0},REG_FAULT,ALL_PUMPS}" in compact
assert "getFaultText" not in source and "updateStatusDescriptions" not in source

fixture = r"""
#include <cassert>
#include <cstdint>
#include <string>
struct Measurement {
  std::string unit;
  double value = -1;
  int changes = 0;
  void setValue(double input) { value = input; }
  void setUnitAfterValue(const char *input) {
    if (unit != input) { unit = input; ++changes; }
  }
};
struct PumpChannels { Measurement *defrost, *mode, *fault; };
const int PUMP_COUNT = 3, STATUS_REGISTERED_AND_READY = 1, REG_FAULT = 0;
Measurement measures[3][3];
PumpChannels pumpChannels[3];
unsigned long now = 0;
unsigned long millis() { return now; }
struct Device { int status = 0; int getCurrentStatus() { return status; } } SuplaDevice;
bool suplaWasReady = false, historyConfigured = false;
unsigned long suplaReadySince = 0, lastHistoryCheck = 0;
const unsigned long SUPLA_CONFIG_SETTLE_TIME = 5000;
int historyCalls = 0;
void configureGpmHistory(bool) { ++historyCalls; }
__CONFIG_FUNCTION__
void publishFault(PumpChannels &channels, uint16_t raw) {
  switch (REG_FAULT) { __FAULT_CASE__ }
}
int main() {
  for (int pump = 0; pump < 3; ++pump) {
    pumpChannels[pump] = {&measures[pump][0], &measures[pump][1], &measures[pump][2]};
    measures[pump][0].unit = "Defrost";
    measures[pump][1].unit = "Heating";
    measures[pump][2].unit = "OK";
    publishFault(pumpChannels[pump], 8000);
    assert(measures[pump][2].value == 0);
    for (uint16_t code : {uint16_t(6999), uint16_t(6607), uint16_t(5101)}) {
      publishFault(pumpChannels[pump], code);
      assert(measures[pump][2].value == code);
    }
  }
  handleGpmConfiguration();
  assert(historyCalls == 0 && !measures[0][0].unit.empty());
  SuplaDevice.status = STATUS_REGISTERED_AND_READY;
  now = 100;
  handleGpmConfiguration();
  now = 5099;
  handleGpmConfiguration();
  assert(historyCalls == 0);
  now = 5100;
  handleGpmConfiguration();
  assert(historyCalls == 1);
  for (auto &pump : measures) for (auto &measurement : pump) {
    assert(measurement.unit.empty() && measurement.changes == 1);
  }
  now += 60000;
  handleGpmConfiguration();
  for (auto &pump : measures) for (auto &measurement : pump) {
    assert(measurement.changes == 1);  // No repeated config writes once clean.
  }
  // A stale unit restored by Cloud during reconnect is removed again.
  SuplaDevice.status = 0;
  handleGpmConfiguration();
  measures[2][2].unit = "Fault";
  SuplaDevice.status = STATUS_REGISTERED_AND_READY;
  handleGpmConfiguration();
  now += 5000;
  handleGpmConfiguration();
  assert(measures[2][2].unit.empty() && measures[2][2].changes == 2);
}
""".replace("__CONFIG_FUNCTION__", block("void handleGpmConfiguration()"))
fixture = fixture.replace("__FAULT_CASE__", block("case REG_FAULT:"))
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "gpm_status.cpp"
    binary = Path(directory) / "gpm_status"
    cpp.write_text(fixture, encoding="utf-8")
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                    str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
