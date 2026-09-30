#pragma once

#include "grid_meter.h"

bool fetchGridMeterSample(const GridMeterConnection &connection, uint16_t timeoutMs,
                          GridMeterSample &sample, int &httpStatus);
