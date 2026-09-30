#include "meter_clients.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>

const char *gridMeterTypeKey(GridMeterType type) {
  switch (type) {
    case GridMeterType::Shelly3EM: return "shelly3em";
    case GridMeterType::ShellyPro3EM: return "shellypro3em";
    case GridMeterType::ShellyEmMiniGen4: return "shellyemminigen4";
    default: return "homewizard";
  }
}

const char *gridMeterTypeName(GridMeterType type) {
  switch (type) {
    case GridMeterType::Shelly3EM: return "Shelly 3EM";
    case GridMeterType::ShellyPro3EM: return "Shelly Pro 3EM";
    case GridMeterType::ShellyEmMiniGen4: return "Shelly EM Mini Gen4";
    default: return "HomeWizard P1";
  }
}

bool parseGridMeterType(const String &value, GridMeterType &type) {
  if (value == "homewizard") type = GridMeterType::HomeWizard;
  else if (value == "shelly3em") type = GridMeterType::Shelly3EM;
  else if (value == "shellypro3em") type = GridMeterType::ShellyPro3EM;
  else if (value == "shellyemminigen4") type = GridMeterType::ShellyEmMiniGen4;
  else return false;
  return true;
}

static bool getJson(const GridMeterConnection &connection, const String &path,
                    uint16_t timeoutMs, JsonDocument &document, int &httpStatus) {
  HTTPClient http;
  http.begin("http://" + connection.ip + path);
  http.setTimeout(timeoutMs);
  httpStatus = http.GET();
  if (httpStatus != HTTP_CODE_OK) {
    http.end();
    return false;
  }
  const DeserializationError error = deserializeJson(document, http.getStream());
  http.end();
  return !error;
}

static bool fetchHomeWizard(const GridMeterConnection &connection, uint16_t timeoutMs,
                            GridMeterSample &sample, int &httpStatus) {
  JsonDocument filter;
  const char *fields[] = {
    "active_power_w",
    "active_power_l1_w", "active_power_l2_w", "active_power_l3_w",
    "active_voltage_l1_v", "active_voltage_l2_v", "active_voltage_l3_v",
    "active_current_l1_a", "active_current_l2_a", "active_current_l3_a",
  };
  for (const char *field : fields) filter[field] = true;

  HTTPClient http;
  http.begin("http://" + connection.ip + "/api/v1/data");
  http.setTimeout(timeoutMs);
  httpStatus = http.GET();
  if (httpStatus != HTTP_CODE_OK) {
    http.end();
    return false;
  }
  JsonDocument document;
  const DeserializationError error = deserializeJson(
    document, http.getStream(), DeserializationOption::Filter(filter));
  http.end();
  if (error) return false;

  const bool hasPhases = document["active_power_l1_w"].is<float>() ||
                         document["active_power_l2_w"].is<float>() ||
                         document["active_power_l3_w"].is<float>();
  const float totalPowerW = document["active_power_w"] | 0.0f;
  sample.totalActivePowerW = totalPowerW;
  sample.hasPerPhasePower = hasPhases;
  for (size_t phase = 0; phase < 3; phase++) {
    const String suffix = String(phase + 1);
    const String powerKey = "active_power_l" + suffix + "_w";
    const String voltageKey = "active_voltage_l" + suffix + "_v";
    const String currentKey = "active_current_l" + suffix + "_a";
    sample.activePowerW[phase] = hasPhases
      ? (document[powerKey] | 0.0f)
      : totalPowerW / 3.0f;
    sample.voltageV[phase] = document[voltageKey] | 230.0f;
    sample.currentA[phase] = document[currentKey].is<float>()
      ? fabsf((float)document[currentKey])
      : (sample.voltageV[phase] > 1.0f
          ? fabsf(sample.activePowerW[phase]) / sample.voltageV[phase]
          : 0.0f);
  }
  return true;
}

static bool fetchShelly3EM(const GridMeterConnection &connection, uint16_t timeoutMs,
                           GridMeterSample &sample, int &httpStatus) {
  JsonDocument document;
  if (!getJson(connection, "/status", timeoutMs, document, httpStatus)) return false;
  JsonArray phases = document["emeters"].as<JsonArray>();
  if (phases.size() < 3) return false;
  sample.totalActivePowerW = 0.0f;
  sample.hasPerPhasePower = true;
  for (size_t phase = 0; phase < 3; phase++) {
    JsonObject values = phases[phase];
    if (!values["power"].is<float>()) return false;
    sample.activePowerW[phase] = values["power"] | 0.0f;
    sample.totalActivePowerW += sample.activePowerW[phase];
    sample.voltageV[phase] = values["voltage"] | 230.0f;
    sample.currentA[phase] = values["current"].is<float>()
      ? fabsf((float)values["current"])
      : (sample.voltageV[phase] > 1.0f
          ? fabsf(sample.activePowerW[phase]) / sample.voltageV[phase]
          : 0.0f);
  }
  return true;
}

static bool fetchShellyPro3EM(const GridMeterConnection &connection, uint16_t timeoutMs,
                              GridMeterSample &sample, int &httpStatus) {
  JsonDocument document;
  if (!getJson(connection, "/rpc/EM.GetStatus?id=0", timeoutMs, document, httpStatus)) return false;
  const char *phaseNames[] = { "a", "b", "c" };
  sample.totalActivePowerW = 0.0f;
  sample.hasPerPhasePower = true;
  for (size_t phase = 0; phase < 3; phase++) {
    const String prefix = phaseNames[phase];
    const String powerKey = prefix + "_act_power";
    const String voltageKey = prefix + "_voltage";
    const String currentKey = prefix + "_current";
    if (!document[powerKey].is<float>()) return false;
    sample.activePowerW[phase] = document[powerKey] | 0.0f;
    sample.totalActivePowerW += sample.activePowerW[phase];
    sample.voltageV[phase] = document[voltageKey] | 230.0f;
    sample.currentA[phase] = document[currentKey].is<float>()
      ? fabsf((float)document[currentKey])
      : (sample.voltageV[phase] > 1.0f
          ? fabsf(sample.activePowerW[phase]) / sample.voltageV[phase]
          : 0.0f);
  }
  return true;
}

static bool fetchShellyEmMiniGen4(const GridMeterConnection &connection, uint16_t timeoutMs,
                                  GridMeterSample &sample, int &httpStatus) {
  JsonDocument document;
  if (!getJson(connection, "/rpc/Shelly.GetStatus", timeoutMs, document, httpStatus)) return false;
  JsonObject values = document["pm1:0"].as<JsonObject>();
  if (!values["apower"].is<float>()) return false;

  sample.totalActivePowerW = values["apower"] | 0.0f;
  sample.hasPerPhasePower = false;
  const float voltageV = values["voltage"] | 230.0f;
  const float currentA = values["current"].is<float>()
    ? fabsf((float)values["current"])
    : (voltageV > 1.0f ? fabsf(sample.totalActivePowerW) / voltageV : 0.0f);
  for (size_t phase = 0; phase < 3; phase++) {
    sample.activePowerW[phase] = sample.totalActivePowerW / 3.0f;
    sample.voltageV[phase] = voltageV;
    sample.currentA[phase] = currentA / 3.0f;
  }
  return true;
}

bool fetchGridMeterSample(const GridMeterConnection &connection, uint16_t timeoutMs,
                          GridMeterSample &sample, int &httpStatus) {
  switch (connection.type) {
    case GridMeterType::Shelly3EM:
      return fetchShelly3EM(connection, timeoutMs, sample, httpStatus);
    case GridMeterType::ShellyPro3EM:
      return fetchShellyPro3EM(connection, timeoutMs, sample, httpStatus);
    case GridMeterType::ShellyEmMiniGen4:
      return fetchShellyEmMiniGen4(connection, timeoutMs, sample, httpStatus);
    default:
      return fetchHomeWizard(connection, timeoutMs, sample, httpStatus);
  }
}
