#pragma once

#include <Arduino.h>
#include <Preferences.h>

class EnergyCounters {
public:
  void begin(Preferences &preferences);
  void update(float phase1W, float phase2W, float phase3W, uint8_t phaseRotation,
              uint32_t nowMs);
  void snapshot(uint64_t output[8]);

private:
  struct Data {
    uint64_t raw[6];
    uint64_t remainderWms[6];
    uint32_t lastUpdateMs;
    uint32_t lastSaveMs;
    bool dirty;
  } data_ = {};

  SemaphoreHandle_t mutex_ = nullptr;
  Preferences *preferences_ = nullptr;

  void saveIfDue(uint32_t nowMs);
};
