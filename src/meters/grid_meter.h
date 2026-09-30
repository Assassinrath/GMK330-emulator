#pragma once

#include <Arduino.h>

enum class GridMeterType : uint8_t {
  HomeWizard = 0,
  Shelly3EM = 1,
  ShellyPro3EM = 2,
  ShellyEmMiniGen4 = 3
};

struct GridMeterSample {
  float voltageV[3];
  float currentA[3];
  float activePowerW[3];
  float totalActivePowerW;
  bool hasPerPhasePower;
};

struct GridMeterConnection {
  GridMeterType type;
  String ip;
};

const char *gridMeterTypeKey(GridMeterType type);
const char *gridMeterTypeName(GridMeterType type);
bool parseGridMeterType(const String &value, GridMeterType &type);
