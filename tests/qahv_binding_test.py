"""Exercise the firmware's actual HVAC binding methods with a minimal base."""
from pathlib import Path
import subprocess
import tempfile


source = (Path(__file__).resolve().parents[1] /
          "MelcoBEMS_SUPLA/MelcoBEMS_SUPLA.ino").read_text(encoding="utf-8")
source = source.split("class QahvThermoOffControl", 1)[1].split(
    "QahvThermoOffControl *qahvThermoOffControl", 1)[0]


def method(signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


methods = "\n".join(method(signature) for signature in (
    "void onInit() override",
    "void iterateAlways() override",
    "bool applyAdditionalValidation(TChannelConfig_HVAC *configuration) override",
))
assert "parameterFlags.MainThermometerChannelNoReadonly = 1;" in source

fixture = r"""
#include <cassert>
const int QAHV_TH17_CHANNEL = 77;
struct TChannelConfig_HVAC { int MainThermometerChannelNo; };
namespace Supla { namespace Control {
class HvacBase {
 public:
  virtual void onInit() { ++initCalls; }
  virtual void iterateAlways() { ++iterateCalls; sampledChannel = thermometer; }
  virtual bool applyAdditionalValidation(TChannelConfig_HVAC *) { return false; }
  int getMainThermometerChannelNo() const { return thermometer; }
  bool setMainThermometerChannelNo(int value) {
    thermometer = value;
    ++writes;
    return true;
  }
  int thermometer = 45, sampledChannel = -1;
  int writes = 0, initCalls = 0, iterateCalls = 0;
};
} }
class Control : public Supla::Control::HvacBase {
 public:
__METHODS__
};
int main() {
  Control control;
  control.onInit();
  assert(control.initCalls == 1 && control.thermometer == 77);
  for (int incoming : {0, 45, 71, 76, 77}) {
    TChannelConfig_HVAC configuration{incoming};
    assert(control.applyAdditionalValidation(&configuration) == (incoming != 77));
    assert(configuration.MainThermometerChannelNo == 77);
  }
  assert(!control.applyAdditionalValidation(nullptr));
  // Simulate a saved/Cloud configuration applied after initialization.
  control.thermometer = 45;
  control.iterateAlways();
  assert(control.thermometer == 77 && control.sampledChannel == 77);
  int writes = control.writes;
  control.iterateAlways();
  assert(control.writes == writes);  // No config/flash churn in a normal loop.
}
""".replace("__METHODS__", methods)
fixture = "#include <initializer_list>\n" + fixture
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / "qahv_binding.cpp"
    binary = Path(directory) / "qahv_binding"
    cpp.write_text(fixture, encoding="utf-8")
    subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                    str(cpp), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
