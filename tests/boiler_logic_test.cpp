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
  for (int bits = 0; bits < 32; ++bits) {
    bool co = bits & 1, f1 = bits & 2, r1 = bits & 4;
    bool f2 = bits & 8, r2 = bits & 16;
    bool expected = co || !f1 || !f2 || r1 || r2;
    assert(secondaryPump(co, f1, r1, f2, r2) == expected);
    bool coilOn = !secondaryPump(co, f1, r1, f2, r2);
    assert(!coilOn == expected);
  }
  // CO and CWU demands are independent; either must keep the boiler on.
  assert(demand(false, true, true, -15, -1000));
  assert(!demand(false, true, true, -15, -2000));
}
