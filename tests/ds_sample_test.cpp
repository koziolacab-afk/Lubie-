#include "../MelcoBEMS_SUPLA/ds_sample.h"
#include <cassert>
#include <limits>

int main() {
  DsSample sample;
  assert(!sample.available(0));
  assert(!sample.accept(-127, 0));
  assert(!sample.accept(85, 0));
  assert(!sample.accept(std::numeric_limits<double>::quiet_NaN(), 0));
  assert(!sample.accept(126, 0));
  assert(sample.accept(52.5, 1000));
  assert(!sample.accept(-127, 16000));
  assert(sample.temperature == 52.5 && sample.sampleAt == 1000);
  assert(sample.available(60999));
  assert(!sample.available(61000));
  assert(sample.accept(53, 62000));
  assert(sample.available(62000) && sample.temperature == 53);
  assert(sample.accept(-55, UINT32_MAX - 10));
  assert(sample.available(10));
  assert(!sample.available(60000));
  assert(sample.accept(125, 70000));
  assert(sample.available(70000));
}
