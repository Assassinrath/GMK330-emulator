#include "energy_counters.h"

static constexpr uint64_t ENERGY_RAW_UNIT_WMS = 36000000ULL;
static constexpr uint32_t ENERGY_SAVE_INTERVAL_MS = 900000UL;
static constexpr uint32_t ENERGY_PERSIST_VERSION = 1U;

struct PersistedEnergyData {
  uint32_t version;
  uint64_t raw[6];
  uint64_t remainderWms[6];
};

void EnergyCounters::begin(Preferences &preferences) {
  preferences_ = &preferences;
  mutex_ = xSemaphoreCreateMutex();
  PersistedEnergyData saved = {};
  if (preferences.getBytesLength("energy") == sizeof(saved) &&
      preferences.getBytes("energy", &saved, sizeof(saved)) == sizeof(saved) &&
      saved.version == ENERGY_PERSIST_VERSION) {
    for (size_t index = 0; index < 6; index++) {
      data_.raw[index] = saved.raw[index];
      data_.remainderWms[index] = saved.remainderWms[index] % ENERGY_RAW_UNIT_WMS;
    }
  }
  data_.lastSaveMs = millis();
}

void EnergyCounters::saveIfDue(uint32_t nowMs) {
  if (nowMs - data_.lastSaveMs < ENERGY_SAVE_INTERVAL_MS) return;

  PersistedEnergyData saved = { ENERGY_PERSIST_VERSION, {}, {} };
  xSemaphoreTake(mutex_, portMAX_DELAY);
  const bool dirty = data_.dirty;
  if (dirty) {
    memcpy(saved.raw, data_.raw, sizeof(saved.raw));
    memcpy(saved.remainderWms, data_.remainderWms, sizeof(saved.remainderWms));
    data_.dirty = false;
  }
  data_.lastSaveMs = nowMs;
  xSemaphoreGive(mutex_);

  if (dirty && preferences_->putBytes("energy", &saved, sizeof(saved)) != sizeof(saved)) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    data_.dirty = true;
    xSemaphoreGive(mutex_);
  }
}

void EnergyCounters::update(float phase1W, float phase2W, float phase3W,
                            uint8_t phaseRotation, uint32_t nowMs) {
  float phasePowerW[3];
  if (phaseRotation == 1) {
    phasePowerW[0] = phase2W; phasePowerW[1] = phase3W; phasePowerW[2] = phase1W;
  } else if (phaseRotation == 2) {
    phasePowerW[0] = phase3W; phasePowerW[1] = phase1W; phasePowerW[2] = phase2W;
  } else {
    phasePowerW[0] = phase1W; phasePowerW[1] = phase2W; phasePowerW[2] = phase3W;
  }

  xSemaphoreTake(mutex_, portMAX_DELAY);
  if (data_.lastUpdateMs != 0) {
    const uint32_t elapsedMs = nowMs - data_.lastUpdateMs;
    if (elapsedMs <= 2000UL) {
      for (size_t phase = 0; phase < 3; phase++) {
        const float powerW = phasePowerW[phase];
        if (powerW == 0.0f) continue;
        const size_t slot = powerW < 0.0f ? phase : phase + 3;
        const uint64_t deltaWms = (uint64_t)(fabsf(powerW) * elapsedMs + 0.5f);
        data_.remainderWms[slot] += deltaWms;
        data_.raw[slot] += data_.remainderWms[slot] / ENERGY_RAW_UNIT_WMS;
        data_.remainderWms[slot] %= ENERGY_RAW_UNIT_WMS;
        data_.dirty = true;
      }
    }
  }
  data_.lastUpdateMs = nowMs;
  xSemaphoreGive(mutex_);

  saveIfDue(nowMs);
}

void EnergyCounters::snapshot(uint64_t output[8]) {
  memset(output, 0, sizeof(uint64_t) * 8);
  xSemaphoreTake(mutex_, portMAX_DELAY);
  memcpy(output, data_.raw, sizeof(data_.raw));
  xSemaphoreGive(mutex_);
}
