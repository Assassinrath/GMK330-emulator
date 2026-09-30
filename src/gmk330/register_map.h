#pragma once

#include <Arduino.h>
#include "app_types.h"

uint16_t gmk330TranslateRegister(const MeterData &meter, uint16_t address);
uint16_t gmk330DiscoveryRegister(uint16_t address);
