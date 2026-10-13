#pragma once

#include <stdint.h>

namespace BoilerLogic {
constexpr uint32_t MaxSampleAgeMs = 30000;

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

inline bool secondaryPump(bool oilCo, bool fresh1, bool running1,
                          bool fresh2, bool running2) {
  return oilCo || !fresh1 || !fresh2 || running1 || running2;
}
}  // namespace BoilerLogic
