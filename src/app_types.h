#pragma once

#include <Arduino.h>

struct MeterData {
  uint16_t Va, Vb, Vc;
  uint16_t Ia, Ib, Ic;
  int32_t Pa, Pb, Pc;
  bool valid;
  int failCount;
  uint32_t lastOkMs;
};
