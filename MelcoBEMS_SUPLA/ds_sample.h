#pragma once
#include <stdint.h>

struct DsSample {
  double temperature = 0;
  uint32_t sampleAt = 0;
  bool known = false;

  bool accept(double value, uint32_t now) {
    // 85 C is the power-on scratchpad value; preserve existing rejection.
    if (!(value >= -55.0 && value <= 125.0) || value == 85.0) return false;
    temperature = value;
    sampleAt = now;
    known = true;
    return true;
  }

  bool available(uint32_t now) const {
    return known && uint32_t(now - sampleAt) < 60000;
  }
};
