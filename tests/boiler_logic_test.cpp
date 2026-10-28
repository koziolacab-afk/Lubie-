#include "../MelcoBEMS_SUPLA/boiler_logic.h"
#include <cassert>
#include <limits>

int main() {
  using namespace BoilerLogic;
  assert(!demand(false, true, true, -10, -1000));
  assert(demand(false, true, true, -10.1, -1000));
  assert(demand(true, true, true, -9, -1000));
  assert(!demand(true, true, true, -8, -1000));
  assert(!demand(true, false, true, -20, -1000));
  assert(!demand(true, true, false, -20, -1000));
  assert(!demand(false, true, true, -20, -3001));
  assert(!demand(false, true, true, std::numeric_limits<double>::quiet_NaN(), -1000));
  assert(demand(false, true, true, -5, 0));
  assert(!demand(true, true, true, -5, -2000));
  assert(fresh(true, true, 30000, 0));
  assert(!fresh(true, true, 30001, 0));
  assert(!fresh(false, true, 1, 0));
  assert(!fresh(true, false, 1, 0));
  assert(fresh(true, true, 10, UINT32_MAX - 10));
  // A charged buffer may stop both compressors (HR127 = 0), but the
  // installation must still circulate while either CAHV is in heating mode.
  assert(secondaryPump(false, true, 2, true, 0));
  assert(secondaryPump(false, true, 0, true, 7));
  assert(secondaryPump(false, true, 7, true, 2));
  assert(!secondaryPump(false, true, 0, true, 0));
  assert(secondaryPump(true, true, 0, true, 0));
  assert(secondaryPump(false, false, 0, true, 0));
  assert(secondaryPump(false, true, 0, false, UINT16_MAX));
  for (uint16_t mode1 = 0; mode1 <= 11; ++mode1) {
    for (uint16_t mode2 = 0; mode2 <= 11; ++mode2) {
      for (int bits = 0; bits < 8; ++bits) {
        bool co = bits & 1, f1 = bits & 2, f2 = bits & 4;
        bool expected = co || !f1 || !f2 || mode1 == 2 || mode1 == 7 ||
                        mode2 == 2 || mode2 == 7;
        assert(secondaryPump(co, f1, mode1, f2, mode2) == expected);
        bool coilOn = !secondaryPump(co, f1, mode1, f2, mode2);
        assert(!coilOn == expected);
      }
    }
  }
  // CO and CWU demands are independent; either must keep the boiler on.
  assert(demand(false, true, true, -15, -1000));
  assert(!demand(false, true, true, -15, -2000));
}
