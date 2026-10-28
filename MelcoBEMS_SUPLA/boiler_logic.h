#pragma once

#include <stdint.h>

namespace BoilerLogic {
constexpr uint32_t MaxSampleAgeMs = 30000;
constexpr uint16_t HeatingMode = 2;
constexpr uint16_t HeatingEcoMode = 7;

inline bool fresh(bool online, bool known, uint32_t now, uint32_t sampleAt) {
  return online && known && uint32_t(now - sampleAt) <= MaxSampleAgeMs;
}

inline bool demand(bool previous, bool enabled, bool sampleFresh,
                   double outdoor, int threshold) {
  if (!enabled || !sampleFresh || threshold < -3000 || threshold > 2000)
    return false;
  if (previous) return outdoor < threshold / 100.0 + 2.0;
  return outdoor < threshold / 100.0;
}

inline bool heatingMode(uint16_t mode) {
  return mode == HeatingMode || mode == HeatingEcoMode;
}

inline bool secondaryPump(bool oilCo, bool fresh1, uint16_t mode1,
                          bool fresh2, uint16_t mode2) {
  return oilCo || !fresh1 || !fresh2 || heatingMode(mode1) || heatingMode(mode2);
}
}  // namespace BoilerLogic
