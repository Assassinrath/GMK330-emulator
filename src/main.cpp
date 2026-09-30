/**
 * HomeWizard P1  ->  GoodWe GMK330 smart-meter emulator
 * Board: LilyGo T-CAN485 (ESP32, MAX13487E auto-direction RS485)
 *
 * The GoodWe GW15K-ET inverter is the Modbus RTU MASTER and polls its smart
 * meter (slave 0x03, 9600 8N1). This firmware becomes that meter: it reads the
 * HomeWizard P1 over WiFi and answers the inverter's Modbus polls with the
 * GMK330 register layout.
 *
 * Register map and Meter1 binding behavior are based on passive captures of
 * the real GMK330 installed on this inverter.
 *
 * Sign convention confirmed against paired P1 and real-GMK330 captures:
 *   HomeWizard active_power_w  > 0 = importing, < 0 = exporting
 *   GMK330 active-power register > 0 = exporting, < 0 = importing
 *
 * SAFETY: if the P1 cannot be read for P1_MAX_FAILS consecutive polls, the
 * emulator stops answering the inverter. Verify the inverter's configured
 * meter-loss behavior before relying on this fail-safe unattended.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>
#include <time.h>
#include "secrets.h"
#include "web_index.h"
#include "app_types.h"
#include "control/energy_counters.h"
#include "gmk330/register_map.h"
#include "meters/meter_clients.h"

// ───────────────────────── USER SETTINGS ─────────────────────────
#define FIRMWARE_VERSION "1.0.4"

// Default P1 IP, used only until a value is saved via the web config page.
#define P1_IP_DEFAULT    "192.168.1.252"

// The observed GMK330 normal operating address. Meter1 binding temporarily
// assigns 0x02, but the inverter resumes measurement polling only at 0x03.
#define GMK330_NORMAL_ADDR 0x03

// Inverter's own Modbus TCP server (Settings -> Modbus TCP on the inverter/SolarGo),
// polled read-only so the dashboard can show what the inverter itself sees for
// Meter1, independent of our RS485 TX and of the SolarGo app's display quirks.
#define INVERTER_IP_DEFAULT   "192.168.1.155"
#define INVERTER_MODBUS_PORT  502
#define INVERTER_UNIT_ID      0xF7   // GoodWe default for ET/EH/BT/BH family
#define INVERTER_POLL_MS      2000
#define INVERTER_TCP_TIMEOUT  800    // ms
#define INVERTER_STATUS_TIMEOUT_MS 6000

// HomeWizard publishes roughly once per second. After detecting changed data,
// wait near the next expected refresh, then probe rapidly until it appears.
#define P1_REFRESH_WAIT_MS  800
#define P1_PROBE_MS         100
#define P1_FAILURE_RETRY_MS 500
#define P1_HTTP_TIMEOUT     700
#define P1_MAX_FAILS     4            // stop answering inverter after N fails

// The GMK330 0x0501 block stores energy in 0.01 kWh increments. Persist at a
// low rate to retain totals without adding significant flash wear.
// Normal gain may be raised for testing, while transient gains remain bounded.
#define CONTROL_FEEDBACK_GAIN       0.33f
#define CONTROL_NORMAL_GAIN_MAX     1.0f

#define CONTROL_DEFAULT_STEP_GAIN       0.25f
#define CONTROL_DEFAULT_BRAKE_GAIN      0.15f
#define CONTROL_DEFAULT_HOLD_MS         100UL
#define CONTROL_DEFAULT_RAMP_MS         1000UL
#define CONTROL_CONFIG_VERSION          2U

// A slow, bounded correction removes the ~100-150 W export offset left by
// the stable 0.25 fast path. It acts only with fresh, valid inverter meter
// communication and freezes around large household load changes.
#define CONTROL_TARGET_IMPORT_W        0.0f
#define CONTROL_AVERAGE_TAU_MS        45000.0f
#define CONTROL_INTEGRAL_TIME_MS      900000.0f
#define CONTROL_BIAS_LIMIT_W          50.0f
#define CONTROL_INTEGRAL_WINDOW_W     400.0f
#define CONTROL_STEP_FREEZE_W         500.0f
#define CONTROL_STEP_FREEZE_MS        30000UL

#define TIME_ZONE "CET-1CEST,M3.5.0,M10.5.0/3"

// No valid frame from the inverter within this long = considered disconnected.
// The GW15K-ET polls roughly every 10s, so keep this comfortably above that.
#define RS485_LINK_TIMEOUT_MS 15000

// If your P1 only reports one phase (1-phase meter) but the inverter is
// 3-phase, set to 1 to spread total power/current evenly over L1/L2/L3.
#define SPLIT_TOTAL_OVER_3PHASE  0

// Paired P1/GMK330 captures use the native P1 phase order.
// 0 = A<-L1,B<-L2,C<-L3 (as-is). 1 = A<-L2,B<-L3,C<-L1.
#define PHASE_ROTATION 0

// Uncomment to log every Modbus frame in/out over USB serial (115200).
#define DEBUG_MODBUS

// ───────────────────────── T-CAN485 PINS ─────────────────────────
// From LilyGo's official example/config.h. All three must be driven HIGH or
// the RS485 transceiver is unpowered / disabled.
#define PIN_5V_EN        16          // boost converter that powers RS485+CAN
#define RS485_EN_PIN     17          // MAX13487 /RE
#define RS485_SE_PIN     19          // MAX13487 /SHDN
#define RS485_TX_PIN     22
#define RS485_RX_PIN     21
#define RS485_BAUD       9600

// Onboard TF card (official LilyGo T-CAN485 pin map).
#define SD_MISO_PIN      2
#define SD_MOSI_PIN      15
#define SD_SCLK_PIN      14
#define SD_CS_PIN        13
#define SD_SPI_HZ        25000000
#define SD_FLUSH_MS      10000
#define SD_LOG_QUEUE_LEN 24
#define SD_BATCH_SIZE    8192
#define SD_DOWNLOAD_BUFFER_SIZE 1360

#define FRAME_TIMEOUT_MS 4           // >3.5 char idle at 9600 delimits a frame
#define MAX_FRAME_LEN    256
#define BYTE_TIME_US     ((10UL * 1000000UL) / RS485_BAUD)

HardwareSerial rs485(2);
static SPIClass sdSpi(VSPI);
static volatile uint32_t lastRs485Ms = 0;   // millis() of last valid frame from the inverter

// Onboard WS2812: green/red flash = P1 read result, breathing blue = inverter absent.
#define WS2812_PIN       4
static Adafruit_NeoPixel statusLed(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
static portMUX_TYPE ledStateMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t ledFlashColor = 0;
static uint32_t ledFlashUntilMs = 0;

static void ledPulse(uint8_t r, uint8_t g, uint8_t b) {
  portENTER_CRITICAL(&ledStateMux);
  ledFlashColor = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  ledFlashUntilMs = millis() + 60;
  portEXIT_CRITICAL(&ledStateMux);
}

static void ledTask(void *) {
  for (;;) {
    const uint32_t nowMs = millis();
    uint32_t flashColor;
    uint32_t flashUntilMs;
    portENTER_CRITICAL(&ledStateMux);
    flashColor = ledFlashColor;
    flashUntilMs = ledFlashUntilMs;
    portEXIT_CRITICAL(&ledStateMux);

    if ((int32_t)(flashUntilMs - nowMs) > 0) {
      statusLed.setPixelColor(0, statusLed.Color(
        (flashColor >> 16) & 0xFF, (flashColor >> 8) & 0xFF, flashColor & 0xFF));
    } else {
      const uint32_t lastFrameMs = lastRs485Ms;
      const bool inverterMissing = lastFrameMs == 0 || nowMs - lastFrameMs > RS485_LINK_TIMEOUT_MS;
      if (inverterMissing) {
        const uint32_t phaseMs = nowMs % 3000UL;
        const uint32_t riseMs = phaseMs <= 1500UL ? phaseMs : 3000UL - phaseMs;
        const uint8_t blue = 12 + (riseMs * 108UL) / 1500UL;
        statusLed.setPixelColor(0, statusLed.Color(0, 0, blue));
      } else {
        statusLed.setPixelColor(0, 0);
      }
    }
    statusLed.show();
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// ───────────────────────── SHARED METER DATA ─────────────────────
static MeterData        meter      = {};
static SemaphoreHandle_t meterMutex = nullptr;
static volatile uint8_t activeMeterAddress = GMK330_NORMAL_ADDR;

static EnergyCounters energyCounters;

// Raw P1 readings (L1/L2/L3, pre-rotation, pre sign-flip) kept only for the /data debug view.
struct P1Debug {
  float v[3];   // V
  float i[3];   // A
  float p[3];   // W, P1's own sign (+import)
};
static P1Debug p1dbg = {};

// Inverter's own view of the grid meter, polled directly over Modbus TCP for
// a 3-way cross-check against p1dbg/meter. Reads BOTH register groups found
// in the GoodWe ET/EH "meter data" block (offset 0x8CA0/36000):
//  - active_power1/2/3/total (16-bit signed, 36005-36008) - the actual live
//    per-phase grid power the inverter's control loop appears to use.
//  - meter_active_power1/2/3/total (32-bit signed, 36019/21/23/25) - reads
//    zero in testing so far, kept for reference/comparison.
//  - meter_comm_status (36004) - 1=OK, 0=NotOK per GoodWe's own comment.
// Also polls the inverter's general "running data" block (offset 0x891C/
// 35100), which drives the inverter's own app/display and is expected to be
// reliably live (unlike the secondary meter-data block above):
//  - active_power (35140) - single combined grid import/export total.
//  - backup_p1/2/3 (35150/35156/35162) + backup_ptotal (35170) - power
//    actually flowing through the backup port per phase, which is what L3's
//    loads are wired to (see repo memory) - lets us separate backup-port
//    contamination from genuine grid readings.
//  - load_p1/2/3 (35164/35166/35168) + load_ptotal (35172) - house load.
struct InverterMeterDebug {
  int16_t  ap[4];      // W, signed: active_power L1, L2, L3, Total (36005-36008)
  int32_t  p[4];       // W, signed: meter_active_power L1, L2, L3, Total (36019-36026)
  int16_t  commStatus; // meter_comm_status (36004): 1 OK, 0 NotOK
  bool     valid;
  uint32_t lastOkMs;

  int16_t  gridActivePower;  // W, signed: active_power (35140), running-data block
  int16_t  backupP[4];       // W: backup_p1/2/3 (35150/35156/35162), backup_ptotal (35170)
  int16_t  loadP[4];         // W: load_p1/2/3 (35164/35166/35168), load_ptotal (35172)
  int32_t  pvP[4];           // W: ppv1/2/3/4 (35105/35109/35113/35117)
  int32_t  inverterAcP;      // W: total_inverter_power (35138)
  int32_t  batteryP;         // W: pbattery1 (35182)
  uint16_t batteryMode;      // battery_mode (35184): 2=discharge, 3=charge
  bool     runValid;
  uint32_t runLastOkMs;

  uint16_t meterRaw[45];     // raw registers 36000-36044 (meter-data block), for /invraw
  uint16_t runRaw[125];      // raw registers 35100-35224 (running-data block), for /invraw
};
static InverterMeterDebug invdbg    = {};
static SemaphoreHandle_t  invMutex  = nullptr;

// One row is queued after each successful P1 fetch. SD I/O runs in a
// low-priority task on core 0 and never blocks the RS485 responder on core 1.
struct SdLogSample {
  uint32_t uptimeMs;
  uint32_t unixTime;
  float rawP[3];
  float rawI[3];
  uint16_t voltage[3];
  int32_t meterP[3];
  uint16_t meterI[3];
  int16_t invActiveP[4];
  int32_t invMeterP[4];
  int16_t invCommStatus;
  int16_t invGridP;
  int16_t invBackupP[4];
  int16_t invLoadP[4];
  int32_t invPvP[4];
  int32_t invAcP;
  int32_t invBatteryP;
  uint32_t invAgeMs;
  uint32_t runAgeMs;
  uint32_t rs485AgeMs;
  uint8_t meterAddress;
  bool invValid;
  bool runValid;
  float controlAverageRawW;
  float controlBiasW;
  float effectiveFeedbackGain;
  float normalGain;
  float stepGain;
  float brakeGain;
  float stepThresholdW;
  float targetImportW;
  uint32_t holdMs;
  uint32_t rampMs;
  uint32_t integralFreezeMs;
  uint8_t transientState;
  bool controlHeld;
};

static QueueHandle_t sdLogQueue = nullptr;
static SemaphoreHandle_t sdMutex = nullptr;
static File          sdLogFile;
static String        sdLogPath;
static volatile bool sdReady = false;
static volatile uint32_t sdDroppedRows = 0;
static volatile uint32_t sdWrittenBytes = 0;
static volatile uint32_t sdLoggedRows = 0;
static volatile uint32_t sdFirstUptimeMs = 0;
static volatile uint32_t sdLatestUptimeMs = 0;
static volatile bool maintenancePaused = false;
static volatile bool sdPauseAcknowledged = false;
static bool otaUploadAuthorized = false;
static bool otaUploadFailed = false;
static bool otaUploadComplete = false;
static bool otaResumeAfterFailure = false;
static String otaError;
static uint32_t otaRestartAtMs = 0;

struct ControlConfig {
  float normalGain;
  float stepGain;
  float brakeGain;
  float stepThresholdW;
  float targetImportW;
  uint32_t holdMs;
  uint32_t rampMs;
  uint32_t integralFreezeMs;
};

static ControlConfig controlConfig = {
  CONTROL_FEEDBACK_GAIN,
  CONTROL_DEFAULT_STEP_GAIN,
  CONTROL_DEFAULT_BRAKE_GAIN,
  CONTROL_STEP_FREEZE_W,
  CONTROL_TARGET_IMPORT_W,
  CONTROL_DEFAULT_HOLD_MS,
  CONTROL_DEFAULT_RAMP_MS,
  CONTROL_STEP_FREEZE_MS
};
static SemaphoreHandle_t controlMutex = nullptr;

static float controlAverageRawW = 0.0f;
static float controlBiasW = 0.0f;
static float controlPreviousRawW = 0.0f;
static float controlEffectiveGain = CONTROL_FEEDBACK_GAIN;
static uint32_t controlLastUpdateMs = 0;
static uint32_t controlFreezeUntilMs = 0;
static uint32_t controlTransientStartMs = 0;
static uint32_t controlCrossedAtMs = 0;
static int8_t controlStepErrorSign = 0;
static uint8_t controlTransientState = 0;  // 0=normal, 1=step hold, 2=brake, 3=ramp
static bool controlInitialized = false;

// ───────────────────────── NETWORK CONFIG ────────────────────────
static Preferences        prefs;
static SemaphoreHandle_t  ipMutex = nullptr;
static String             wifiSsid;
static String             wifiPassword;
static String             configUser;
static String             configPassword;
static String             p1Ip;
static GridMeterType      gridMeterType = GridMeterType::HomeWizard;
static volatile uint32_t  gridMeterGeneration = 0;
static String             inverterIp;
static bool               inverterModbusEnabled = true;
static WebServer          server(80);

// ───────────────────────── SERIAL LOG BUFFER ─────────────────────
// Mirrors everything printed to USB serial into a RAM ring buffer so the
// web config page can show the same log without a second serial connection.
// True circular buffer (head/count indices): appends cost O(bytes written),
// never O(buffer size), so it's safe to call from the timing-critical RS485 path.
#define LOG_BUF_SIZE 8192
static char              logBuf[LOG_BUF_SIZE];
static size_t            logHead  = 0;   // index of oldest byte
static size_t            logCount = 0;   // valid bytes currently stored
static SemaphoreHandle_t logMutex = nullptr;

static void logAppend(const char *s) {
  size_t n = strlen(s);
  if (n == 0) return;
  if (n > LOG_BUF_SIZE) { s += n - LOG_BUF_SIZE; n = LOG_BUF_SIZE; }   // keep only the tail
  xSemaphoreTake(logMutex, portMAX_DELAY);
  size_t pos = (logHead + logCount) % LOG_BUF_SIZE;
  for (size_t i = 0; i < n; i++) {
    logBuf[pos] = s[i];
    pos = (pos + 1) % LOG_BUF_SIZE;
  }
  size_t total = logCount + n;
  if (total > LOG_BUF_SIZE) {
    logHead  = (logHead + (total - LOG_BUF_SIZE)) % LOG_BUF_SIZE;
    logCount = LOG_BUF_SIZE;
  } else {
    logCount = total;
  }
  xSemaphoreGive(logMutex);
}

static void logPrintf(const char *fmt, ...) {
  char tmp[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);
  Serial.print(tmp);
  logAppend(tmp);
}

static void logPrintln(const String &s = "") {
  Serial.println(s);
  logAppend(s.c_str());
  logAppend("\n");
}

// ───────────────────────── CRC-16 / MODBUS ───────────────────────
static uint16_t crc16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001u : (crc >> 1);
  }
  return crc;
}

static bool crcOk(const uint8_t *f, size_t len) {
  if (len < 4) return false;
  uint16_t calc = crc16(f, len - 2);
  uint16_t recv = (uint16_t)f[len - 2] | ((uint16_t)f[len - 1] << 8);
  return calc == recv;
}

static void appendCRC(uint8_t *buf, size_t len) {
  uint16_t crc = crc16(buf, len);
  buf[len]     = crc & 0xFF;
  buf[len + 1] = (crc >> 8) & 0xFF;
}

// ───────────────────────── RS485 TX ──────────────────────────────
static void rs485Send(const uint8_t *buf, size_t len) {
#ifdef DEBUG_MODBUS
  logPrintf("[TX] %u:", (unsigned)len);
  for (size_t i = 0; i < len; i++) logPrintf(" %02X", buf[i]);
  logPrintln();
#endif
  rs485.write(buf, len);
  rs485.flush();                       // block until last bit is on the wire
  // Auto-direction transceiver echoes our own frame back onto RX; discard it.
  delayMicroseconds(2UL * BYTE_TIME_US);
  while (rs485.available()) rs485.read();
  delayMicroseconds(4UL * BYTE_TIME_US);   // Modbus inter-frame guard
}

static void sendReadResponse(uint8_t addr, uint8_t fc, uint16_t start, uint16_t count) {
  xSemaphoreTake(meterMutex, portMAX_DELAY);
  bool       ok   = meter.valid;
  MeterData  snap = meter;
  xSemaphoreGive(meterMutex);

  uint64_t energyRaw[8] = {};
  energyCounters.snapshot(energyRaw);

  if (!ok) return;                     // safety hold: let the inverter time out

  if (count == 0 || count > 125) {
    uint8_t ex[5] = { addr, (uint8_t)(fc | 0x80), 0x03 };
    appendCRC(ex, 3);
    rs485Send(ex, 5);
    return;
  }

  uint8_t buf[3 + 125 * 2 + 2];
  size_t  idx = 0;
  buf[idx++] = addr;
  buf[idx++] = fc;
  buf[idx++] = (uint8_t)(count * 2);
  for (uint16_t i = 0; i < count; i++) {
    const uint16_t addr = start + i;
    uint16_t val;
    if (addr >= 0x0501 && addr <= 0x0520) {
      const uint16_t offset = addr - 0x0501;
      const uint8_t slot = offset / 4;
      const uint8_t word = offset % 4;
      val = (uint16_t)(energyRaw[slot] >> ((3 - word) * 16));
    } else if (addr >= 0x0132 && addr <= 0x0157) {
      val = gmk330TranslateRegister(snap, addr);
    } else {
      val = gmk330DiscoveryRegister(addr);
    }
    buf[idx++] = (val >> 8) & 0xFF;
    buf[idx++] =  val       & 0xFF;
  }
  appendCRC(buf, idx);
  idx += 2;
  rs485Send(buf, idx);
}

// Ack a Write Multiple Registers request (FC 0x10) - GoodWe's app writes a
// config register to the meter as part of "Add external meter"/binding. We
// don't know what the register means, but echoing a normal Modbus write
// confirmation (slave/fc/start/qty) lets that handshake step appear to
// succeed instead of silently timing out.
static void sendWriteAck(uint8_t addr, uint16_t start, uint16_t qty) {
  uint8_t buf[8];
  buf[0] = addr;
  buf[1] = 0x10;
  buf[2] = (start >> 8) & 0xFF;
  buf[3] = start & 0xFF;
  buf[4] = (qty >> 8) & 0xFF;
  buf[5] = qty & 0xFF;
  appendCRC(buf, 6);
  rs485Send(buf, 8);
}

// GoodWe's proprietary AA-55 protocol shares the RS485 bus with Modbus RTU;
// the WiFi dongle uses it to read the meter's identity string, and the
// SolarGo app's "Meter Binding" screen uses it to push/confirm a serial
// number. Frame: AA 55 [src] [dst] [ctrl] [fn] [len] [data] [chk_hi chk_lo],
// checksum = 16-bit sum of every preceding byte (magic included).
#define AA55_DISCOVERY_ADDR 0x01
// Installation-specific identity observed during the GMK330 binding exchange.
static const uint8_t AA55_GMK330_ID[] = GMK330_ID;
static_assert(sizeof(AA55_GMK330_ID) == 17, "GMK330_ID must contain exactly 16 characters");

static void aa55Send(uint8_t src, uint8_t dst, uint8_t ctrl, uint8_t fn,
                     const uint8_t *data, uint8_t dlen) {
  uint8_t buf[7 + 32 + 2];
  size_t  idx = 0;
  buf[idx++] = 0xAA;
  buf[idx++] = 0x55;
  buf[idx++] = src;
  buf[idx++] = dst;
  buf[idx++] = ctrl;
  buf[idx++] = fn;
  buf[idx++] = dlen;
  for (uint8_t i = 0; i < dlen; i++) buf[idx++] = data[i];
  uint16_t sum = 0;
  for (size_t i = 0; i < idx; i++) sum += buf[i];
  buf[idx++] = (uint8_t)(sum >> 8);
  buf[idx++] = (uint8_t)(sum & 0xFF);
  rs485Send(buf, idx);
}

// Returns true if the frame was AA-55 proprietary traffic (handled or ignored here).
static bool processAA55Frame(const uint8_t *f, size_t len) {
  if (len < 9 || f[0] != 0xAA || f[1] != 0x55) return false;
  uint8_t src = f[2], dst = f[3], fn = f[5], dlen = f[6];
  const size_t expectedLen = 7 + dlen + 2;
  if (len != expectedLen) return true;
  uint16_t sum = 0;
  for (size_t i = 0; i < expectedLen - 2; i++) sum += f[i];
  const uint16_t received = ((uint16_t)f[expectedLen - 2] << 8) | f[expectedLen - 1];
  if (sum != received) return true;

  if (fn == 0x83 && dst == AA55_DISCOVERY_ADDR) {
#ifdef DEBUG_MODBUS
    logPrintf("[AA55] fn=%02X dlen=%u\n", fn, dlen);
#endif
    aa55Send(AA55_DISCOVERY_ADDR, src, 0x04, fn, AA55_GMK330_ID, 16);
  } else if (fn == 0x85 && dst == 0x7F && dlen == 17 &&
             memcmp(f + 7, AA55_GMK330_ID, 16) == 0 &&
             (f[7 + 16] == 0x02 || f[7 + 16] == GMK330_NORMAL_ADDR)) {
    // The binding request carries the GMK330 identity followed by the target
    // Modbus address. The real meter replies with an empty function-0x85 frame.
    const uint8_t target = f[7 + 16];
    aa55Send(target, src, 0x04, fn, nullptr, 0);
  }
  return true;
}

static void processFrame(const uint8_t *frame, size_t len) {
  if (maintenancePaused) return;
  if (processAA55Frame(frame, len)) return;
  if (!crcOk(frame, len)) return;
  uint8_t addr = frame[0];
  const uint8_t fc = frame[1];
  const bool isBindingWrite = fc == 0x10 && len == 11 &&
                              (((uint16_t)frame[2] << 8) | frame[3]) == 0x0200 &&
                              frame[4] == 0x00 && frame[5] == 0x01 &&
                              frame[6] == 0x02 && frame[7] == 0x00 &&
                              frame[8] == addr &&
                              (addr == 0x02 || addr == GMK330_NORMAL_ADDR);
  if (addr != activeMeterAddress && !isBindingWrite) return;

  lastRs485Ms = millis();   // a genuine frame addressed to us -> inverter is on the bus

#ifdef DEBUG_MODBUS
  logPrintf("[RX] %u:", (unsigned)len);
  for (size_t i = 0; i < len; i++) logPrintf(" %02X", frame[i]);
  logPrintln();
#endif

  if ((fc == 0x03 || fc == 0x04) && len == 8) {
    uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t count = ((uint16_t)frame[4] << 8) | frame[5];
    sendReadResponse(addr, fc, start, count);
  } else if (isBindingWrite) {
    uint16_t start = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t qty   = ((uint16_t)frame[4] << 8) | frame[5];
    uint8_t  byteCount = frame[6];
    if (len == (size_t)(7 + byteCount + 2)) {
      activeMeterAddress = addr;
      sendWriteAck(addr, start, qty);
    }
  }
}

// ───────────────────────── P1 FETCH ──────────────────────────────
static uint16_t toVoltReg(float v) {
  int32_t r = (int32_t)lroundf(v * 10.0f);
  return (uint16_t)(r < 0 ? 0 : (r > 65535 ? 65535 : r));
}
static uint16_t toCurrReg(float a) {
  int32_t r = (int32_t)lroundf(fabsf(a) * 100.0f);
  return (uint16_t)(r > 65535 ? 65535 : r);
}
static int32_t toPowerReg(float w) {
  return (int32_t)lroundf(w);
}

static void markFail() {
  ledPulse(40, 0, 0);
  xSemaphoreTake(meterMutex, portMAX_DELAY);
  meter.failCount++;
  if (meter.failCount >= P1_MAX_FAILS) meter.valid = false;
  xSemaphoreGive(meterMutex);
}

static String getP1Ip() {
  xSemaphoreTake(ipMutex, portMAX_DELAY);
  String ip = p1Ip;
  xSemaphoreGive(ipMutex);
  return ip;
}

static GridMeterConnection getGridMeterConnection() {
  xSemaphoreTake(ipMutex, portMAX_DELAY);
  GridMeterConnection connection = { gridMeterType, p1Ip };
  xSemaphoreGive(ipMutex);
  return connection;
}

static void setP1Ip(const String &ip) {
  xSemaphoreTake(ipMutex, portMAX_DELAY);
  p1Ip = ip;
  xSemaphoreGive(ipMutex);
  prefs.putString("p1ip", ip);
}

static void setGridMeterConnection(const GridMeterConnection &connection) {
  xSemaphoreTake(ipMutex, portMAX_DELAY);
  const bool sourceChanged = gridMeterType != connection.type || p1Ip != connection.ip;
  gridMeterType = connection.type;
  p1Ip = connection.ip;
  if (sourceChanged) gridMeterGeneration = gridMeterGeneration + 1;
  xSemaphoreGive(ipMutex);

  prefs.putString("meterType", gridMeterTypeKey(connection.type));
  prefs.putString("p1ip", connection.ip);

  if (sourceChanged) {
    xSemaphoreTake(meterMutex, portMAX_DELAY);
    meter.valid = false;
    meter.failCount = 0;
    meter.lastOkMs = 0;
    xSemaphoreGive(meterMutex);
  }
}

struct InverterConnectionConfig {
  String ip;
  bool enabled;
};

static InverterConnectionConfig getInverterConnectionConfig() {
  xSemaphoreTake(ipMutex, portMAX_DELAY);
  InverterConnectionConfig config = {inverterIp, inverterModbusEnabled};
  xSemaphoreGive(ipMutex);
  return config;
}

static void setInverterConnectionConfig(const String &ip, bool enabled) {
  xSemaphoreTake(ipMutex, portMAX_DELAY);
  inverterIp = ip;
  inverterModbusEnabled = enabled;
  xSemaphoreGive(ipMutex);
  prefs.putString("invip", ip);
  prefs.putBool("inven", enabled);
}

static bool isValidIp(const String &s) {
  IPAddress ip;
  return ip.fromString(s);
}

static ControlConfig getControlConfig() {
  xSemaphoreTake(controlMutex, portMAX_DELAY);
  ControlConfig config = controlConfig;
  xSemaphoreGive(controlMutex);
  return config;
}

static int8_t controlSign(float value) {
  return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0);
}

enum class P1FetchResult : uint8_t {
  Failed,
  Unchanged,
  Changed
};

static P1FetchResult fetchP1() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    markFail();
    return P1FetchResult::Failed;
  }

  const GridMeterConnection connection = getGridMeterConnection();
  GridMeterSample sourceSample = {};
  int code = 0;
  if (!fetchGridMeterSample(connection, P1_HTTP_TIMEOUT, sourceSample, code)) {
    markFail();
#ifdef DEBUG_MODBUS
    logPrintf("[GRID] %s HTTP/JSON failure (%d)\n", gridMeterTypeName(connection.type), code);
#endif
    return P1FetchResult::Failed;
  }

  float p1 = sourceSample.activePowerW[0];
  float p2 = sourceSample.activePowerW[1];
  float p3 = sourceSample.activePowerW[2];
  if (!sourceSample.hasPerPhasePower || SPLIT_TOTAL_OVER_3PHASE) {
    p1 = p2 = p3 = sourceSample.totalActivePowerW / 3.0f;
  }

  float p1raw = p1, p2raw = p2, p3raw = p3;   // pre-negation, for the /data debug view
  static bool previousReadingValid = false;
  static float previousPowerW[3] = {};
  static float previousVoltageV[3] = {};
  static float previousCurrentA[3] = {};
  static uint32_t previousGeneration = UINT32_MAX;

  float v1 = sourceSample.voltageV[0];
  float v2 = sourceSample.voltageV[1];
  float v3 = sourceSample.voltageV[2];
  float i1raw = sourceSample.currentA[0];
  float i2raw = sourceSample.currentA[1];
  float i3raw = sourceSample.currentA[2];
  const float powerW[3] = { p1raw, p2raw, p3raw };
  const float voltageV[3] = { v1, v2, v3 };
  const float currentA[3] = { i1raw, i2raw, i3raw };
  const uint32_t currentGeneration = gridMeterGeneration;
  bool readingChanged = !previousReadingValid || previousGeneration != currentGeneration;
  for (size_t phase = 0; phase < 3 && !readingChanged; phase++) {
    readingChanged = powerW[phase] != previousPowerW[phase] ||
                     voltageV[phase] != previousVoltageV[phase] ||
                     currentA[phase] != previousCurrentA[phase];
  }
  memcpy(previousPowerW, powerW, sizeof(powerW));
  memcpy(previousVoltageV, voltageV, sizeof(voltageV));
  memcpy(previousCurrentA, currentA, sizeof(currentA));
  previousReadingValid = true;
  previousGeneration = currentGeneration;

  const uint32_t nowMs = millis();
  energyCounters.update(p1raw, p2raw, p3raw, PHASE_ROTATION, nowMs);
  const float rawTotalW = p1raw + p2raw + p3raw;
  const ControlConfig config = getControlConfig();
  uint32_t controlDtMs = controlLastUpdateMs ? nowMs - controlLastUpdateMs : P1_FAILURE_RETRY_MS;
  if (controlDtMs > 2000) controlDtMs = P1_FAILURE_RETRY_MS;
  controlLastUpdateMs = nowMs;

  if (!controlInitialized) {
    controlAverageRawW = rawTotalW;
    controlPreviousRawW = rawTotalW;
    controlInitialized = true;
  }
  if (fabsf(rawTotalW - controlPreviousRawW) > config.stepThresholdW) {
    controlFreezeUntilMs = nowMs + config.integralFreezeMs;
    controlTransientStartMs = nowMs;
    controlCrossedAtMs = 0;
    controlStepErrorSign = controlSign(rawTotalW - config.targetImportW);
    controlAverageRawW = rawTotalW;
  } else {
    const float alpha = controlDtMs / (CONTROL_AVERAGE_TAU_MS + controlDtMs);
    controlAverageRawW += alpha * (rawTotalW - controlAverageRawW);
  }
  controlPreviousRawW = rawTotalW;

  controlEffectiveGain = config.normalGain;
  controlTransientState = 0;
  if (controlTransientStartMs) {
    const uint32_t transientAgeMs = nowMs - controlTransientStartMs;
    if (!controlCrossedAtMs && transientAgeMs >= config.holdMs) {
      const int8_t currentErrorSign = controlSign(rawTotalW - config.targetImportW);
      if (currentErrorSign && controlStepErrorSign && currentErrorSign != controlStepErrorSign) {
        controlCrossedAtMs = nowMs;
      }
    }

    if (transientAgeMs < config.holdMs) {
      controlEffectiveGain = config.stepGain;
      controlTransientState = 1;
    } else {
      const bool braking = controlCrossedAtMs != 0;
      const uint32_t rampAgeMs = braking ? nowMs - controlCrossedAtMs
                                         : transientAgeMs - config.holdMs;
      const float ramp = min(1.0f, (float)rampAgeMs / (float)config.rampMs);
      const float startGain = braking ? config.brakeGain : config.stepGain;
      controlEffectiveGain = startGain + ramp * (config.normalGain - startGain);
      controlTransientState = braking && rampAgeMs < controlDtMs ? 2 : 3;
      if (ramp >= 1.0f) {
        controlTransientStartMs = 0;
        controlCrossedAtMs = 0;
        controlTransientState = 0;
      }
    }
  }

  const uint32_t rs485AgeMs = lastRs485Ms ? nowMs - lastRs485Ms : UINT32_MAX;
  const bool rs485ControlValid = rs485AgeMs <= RS485_LINK_TIMEOUT_MS;
  const bool stepFreezeActive = (int32_t)(controlFreezeUntilMs - nowMs) > 0;
  const bool outsideIntegralWindow =
    fabsf(controlAverageRawW - config.targetImportW) > CONTROL_INTEGRAL_WINDOW_W;
  const bool controlHeld = !rs485ControlValid || stepFreezeActive ||
                           outsideIntegralWindow || maintenancePaused;
  if (!controlHeld) {
    const float errorW = config.targetImportW - controlAverageRawW;
    controlBiasW += errorW * ((float)controlDtMs / CONTROL_INTEGRAL_TIME_MS);
    controlBiasW = constrain(controlBiasW, -CONTROL_BIAS_LIMIT_W, CONTROL_BIAS_LIMIT_W);
  }
  const float phaseBiasW = controlBiasW / 3.0f;

  // Confirmed by live test: with P1's own sign (positive=import) sent as-is,
  // the inverter discharged the battery at max power while the home was
  // actually EXPORTING ~12kW - i.e. it read our export as a large import
  // demand. This GW15K-ET "External meter" role expects the opposite
  // convention from the GW25K-MT reference (positive=export, negative=
  // import), so negate here.
  p1 = -p1 * controlEffectiveGain + phaseBiasW;
  p2 = -p2 * controlEffectiveGain + phaseBiasW;
  p3 = -p3 * controlEffectiveGain + phaseBiasW;

  // Voltages default to 230 V so the inverter never sees a "dead phase".
  // Currents come from P1 when available and otherwise use |P| / V.
  float i1 = i1raw * controlEffectiveGain;
  float i2 = i2raw * controlEffectiveGain;
  float i3 = i3raw * controlEffectiveGain;
  if (v1 > 1) i1 = max(i1, fabsf(p1) / v1);
  if (v2 > 1) i2 = max(i2, fabsf(p2) / v2);
  if (v3 > 1) i3 = max(i3, fabsf(p3) / v3);

#if PHASE_ROTATION == 1
  float av = v2, bv = v3, cv = v1;
  float ai = i2, bi = i3, ci = i1;
  float ap = p2, bp = p3, cp = p1;
#elif PHASE_ROTATION == 2
  float av = v3, bv = v1, cv = v2;
  float ai = i3, bi = i1, ci = i2;
  float ap = p3, bp = p1, cp = p2;
#else
  float av = v1, bv = v2, cv = v3;
  float ai = i1, bi = i2, ci = i3;
  float ap = p1, bp = p2, cp = p3;
#endif

  xSemaphoreTake(meterMutex, portMAX_DELAY);
  meter.Va = toVoltReg(av);  meter.Vb = toVoltReg(bv);  meter.Vc = toVoltReg(cv);
  meter.Ia = toCurrReg(ai);  meter.Ib = toCurrReg(bi);  meter.Ic = toCurrReg(ci);
  meter.Pa = toPowerReg(ap); meter.Pb = toPowerReg(bp); meter.Pc = toPowerReg(cp);
  meter.failCount = 0;
  meter.valid     = true;
  meter.lastOkMs  = millis();
  p1dbg.v[0] = v1;    p1dbg.v[1] = v2;    p1dbg.v[2] = v3;
  p1dbg.i[0] = i1raw; p1dbg.i[1] = i2raw; p1dbg.i[2] = i3raw;
  p1dbg.p[0] = p1raw; p1dbg.p[1] = p2raw; p1dbg.p[2] = p3raw;
  xSemaphoreGive(meterMutex);

  static uint32_t lastCsvSampleMs = 0;
  const bool csvSampleDue = readingChanged || nowMs - lastCsvSampleMs >= 1000UL;
  if (csvSampleDue && sdLogQueue && sdReady) {
    SdLogSample sample = {};
    sample.uptimeMs = millis();
    const time_t currentTime = time(nullptr);
    sample.unixTime = currentTime > 1000000000 ? (uint32_t)currentTime : 0;
    sample.rawP[0] = p1raw; sample.rawP[1] = p2raw; sample.rawP[2] = p3raw;
    sample.rawI[0] = i1raw; sample.rawI[1] = i2raw; sample.rawI[2] = i3raw;
    sample.voltage[0] = toVoltReg(v1); sample.voltage[1] = toVoltReg(v2); sample.voltage[2] = toVoltReg(v3);
    sample.meterP[0] = toPowerReg(ap); sample.meterP[1] = toPowerReg(bp); sample.meterP[2] = toPowerReg(cp);
    sample.meterI[0] = toCurrReg(ai); sample.meterI[1] = toCurrReg(bi); sample.meterI[2] = toCurrReg(ci);

    xSemaphoreTake(invMutex, portMAX_DELAY);
    for (int k = 0; k < 4; k++) {
      sample.invActiveP[k] = invdbg.ap[k];
      sample.invMeterP[k] = invdbg.p[k];
      sample.invBackupP[k] = invdbg.backupP[k];
      sample.invLoadP[k] = invdbg.loadP[k];
      sample.invPvP[k] = invdbg.pvP[k];
    }
    sample.invCommStatus = invdbg.commStatus;
    sample.invGridP = invdbg.gridActivePower;
    sample.invAcP = invdbg.inverterAcP;
    sample.invBatteryP = invdbg.batteryP;
    sample.invValid = invdbg.valid;
    sample.runValid = invdbg.runValid;
    sample.invAgeMs = invdbg.lastOkMs ? sample.uptimeMs - invdbg.lastOkMs : UINT32_MAX;
    sample.runAgeMs = invdbg.runLastOkMs ? sample.uptimeMs - invdbg.runLastOkMs : UINT32_MAX;
    xSemaphoreGive(invMutex);
    sample.rs485AgeMs = lastRs485Ms ? sample.uptimeMs - lastRs485Ms : UINT32_MAX;
    sample.meterAddress = activeMeterAddress;
    sample.controlAverageRawW = controlAverageRawW;
    sample.controlBiasW = controlBiasW;
    sample.effectiveFeedbackGain = controlEffectiveGain;
    sample.normalGain = config.normalGain;
    sample.stepGain = config.stepGain;
    sample.brakeGain = config.brakeGain;
    sample.stepThresholdW = config.stepThresholdW;
    sample.targetImportW = config.targetImportW;
    sample.holdMs = config.holdMs;
    sample.rampMs = config.rampMs;
    sample.integralFreezeMs = config.integralFreezeMs;
    sample.transientState = controlTransientState;
    sample.controlHeld = controlHeld;

    if (xQueueSend(sdLogQueue, &sample, 0) != pdTRUE) {
      sdDroppedRows = sdDroppedRows + 1;
    } else {
      lastCsvSampleMs = nowMs;
    }
  }

  if (readingChanged) ledPulse(0, 40, 0);

#ifdef DEBUG_MODBUS
  // Values here use the same (negated) sign convention as what's sent to the
  // inverter, so the log matches what it actually sees.
  if (readingChanged) {
    logPrintf("[P1] P=%d/%d/%d W  V=%.1f/%.1f/%.1f  total=%.0f W\n",
              (int)meter.Pa, (int)meter.Pb, (int)meter.Pc, v1, v2, v3,
              (float)(meter.Pa + meter.Pb + meter.Pc));
  }
#endif
  return readingChanged ? P1FetchResult::Changed : P1FetchResult::Unchanged;
}

// Reads the inverter's own grid-meter view over Modbus TCP: both
// active_power1/2/3/total (16-bit, 36005-36008) and meter_active_power1/2/3/
// total (32-bit, 36019-36026), plus meter_comm_status (36004). Ground truth
// for what the inverter's control loop actually sees, independent of our
// RS485 TX and of the SolarGo app's display quirks.
static bool readInverterMeter(const String &ip, int16_t ap[4], int32_t p[4],
                              int16_t *commStatus, uint16_t raw[45]) {
  WiFiClient client;
  if (!client.connect(ip.c_str(), INVERTER_MODBUS_PORT, INVERTER_TCP_TIMEOUT)) return false;

  // The inverter only answers its documented "READ_METER_DATA" block read
  // (start 0x8CA0/36000, count 0x2D/45 regs) - a sub-range starting mid-block
  // (e.g. straight at 36019) gets a valid-looking but all-zero response.
  const uint8_t req[12] = {
    0x00, 0x01,                          // transaction id
    0x00, 0x00,                          // protocol id (Modbus)
    0x00, 0x06,                          // remaining length (unit id + PDU)
    INVERTER_UNIT_ID,
    0x03,                                // function: read holding registers
    0x8C, 0xA0,                          // start address 36000
    0x00, 0x2D,                          // quantity: 45 registers
  };
  client.write(req, sizeof(req));

  uint8_t       resp[128];
  size_t        n        = 0;
  const size_t  expected = 9 + 45 * 2;   // MBAP(7)+func(1)+bytecount(1) + 90 data bytes
  uint32_t      t0       = millis();
  while (millis() - t0 < INVERTER_TCP_TIMEOUT && n < expected) {
    if (client.available()) {
      int b = client.read();
      if (b >= 0 && n < sizeof(resp)) resp[n++] = (uint8_t)b;
    } else {
      delay(2);
    }
  }
  client.stop();

  if (n < expected || resp[7] != 0x03) return false;   // short frame or Modbus exception
  const uint8_t *d = resp + 9;                         // skip MBAP(7) + func(1) + byte count(1)

  auto reg16 = [&](int regAddr) -> int16_t {
    size_t b = (regAddr - 36000) * 2;
    return (int16_t)(((uint16_t)d[b] << 8) | d[b + 1]);
  };
  auto reg32 = [&](int regAddr) -> int32_t {
    size_t b = (regAddr - 36000) * 2;
    return (int32_t)(((uint32_t)d[b] << 24) | ((uint32_t)d[b + 1] << 16) |
                      ((uint32_t)d[b + 2] << 8) | d[b + 3]);
  };

  *commStatus = reg16(36004);
  const int apReg[4] = {36005, 36006, 36007, 36008};
  const int pReg[4]  = {36019, 36021, 36023, 36025};
  for (int k = 0; k < 4; k++) { ap[k] = reg16(apReg[k]); p[k] = reg32(pReg[k]); }
  for (int k = 0; k < 45; k++) raw[k] = (uint16_t)((d[k * 2] << 8) | d[k * 2 + 1]);
  return true;
}

// Reads the inverter's general "running data" block (start 0x891C/35100,
// count 0x7D/125 regs) - the same data source that drives the inverter's own
// app/display, so it should be reliably live even if the meter-data block
// above reads all-zero/NotOK.
static bool readInverterRunningData(const String &ip, int16_t *gridActivePower,
                                    int16_t backupP[4], int16_t loadP[4],
                                    int32_t pvP[4], int32_t *inverterAcP,
                                    int32_t *batteryP, uint16_t *batteryMode,
                                    uint16_t raw[125]) {
  WiFiClient client;
  if (!client.connect(ip.c_str(), INVERTER_MODBUS_PORT, INVERTER_TCP_TIMEOUT)) return false;

  const uint8_t req[12] = {
    0x00, 0x02,                          // transaction id
    0x00, 0x00,                          // protocol id (Modbus)
    0x00, 0x06,                          // remaining length (unit id + PDU)
    INVERTER_UNIT_ID,
    0x03,                                // function: read holding registers
    0x89, 0x1C,                          // start address 35100
    0x00, 0x7D,                          // quantity: 125 registers
  };
  client.write(req, sizeof(req));

  uint8_t       resp[280];
  size_t        n        = 0;
  const size_t  expected = 9 + 125 * 2;  // MBAP(7)+func(1)+bytecount(1) + 250 data bytes
  uint32_t      t0       = millis();
  while (millis() - t0 < INVERTER_TCP_TIMEOUT && n < expected) {
    if (client.available()) {
      int b = client.read();
      if (b >= 0 && n < sizeof(resp)) resp[n++] = (uint8_t)b;
    } else {
      delay(2);
    }
  }
  client.stop();

  if (n < expected || resp[7] != 0x03) return false;   // short frame or Modbus exception
  const uint8_t *d = resp + 9;                         // skip MBAP(7) + func(1) + byte count(1)

  auto reg16 = [&](int regAddr) -> int16_t {
    size_t b = (regAddr - 35100) * 2;
    return (int16_t)(((uint16_t)d[b] << 8) | d[b + 1]);
  };
  auto reg32 = [&](int regAddr) -> int32_t {
    size_t b = (regAddr - 35100) * 2;
    return (int32_t)(((uint32_t)d[b] << 24) | ((uint32_t)d[b + 1] << 16) |
                     ((uint32_t)d[b + 2] << 8) | d[b + 3]);
  };

  *gridActivePower = reg16(35140);
  const int backupReg[4] = {35150, 35156, 35162, 35170};
  const int loadReg[4]   = {35164, 35166, 35168, 35172};
  for (int k = 0; k < 4; k++) { backupP[k] = reg16(backupReg[k]); loadP[k] = reg16(loadReg[k]); }
  const int pvReg[4] = {35105, 35109, 35113, 35117};
  for (int k = 0; k < 4; k++) pvP[k] = reg32(pvReg[k]);
  *inverterAcP = reg16(35138);
  *batteryP = reg32(35182);
  *batteryMode = (uint16_t)reg16(35184);
  for (int k = 0; k < 125; k++) raw[k] = (uint16_t)((d[k * 2] << 8) | d[k * 2 + 1]);
  return true;
}

static void fetchInverterMeter() {
  const InverterConnectionConfig config = getInverterConnectionConfig();
  if (!config.enabled) return;

  int16_t ap[4]; int32_t p[4]; int16_t commStatus = 0; uint16_t meterRaw[45];
  bool ok = (WiFi.status() == WL_CONNECTED) &&
            readInverterMeter(config.ip, ap, p, &commStatus, meterRaw);

  int16_t gridActivePower = 0; int16_t backupP[4]; int16_t loadP[4];
  int32_t pvP[4]; int32_t inverterAcP = 0; int32_t batteryP = 0;
  uint16_t batteryMode = 0; uint16_t runRaw[125];
  bool runOk = (WiFi.status() == WL_CONNECTED) &&
               readInverterRunningData(config.ip, &gridActivePower, backupP, loadP, pvP,
                                       &inverterAcP, &batteryP, &batteryMode, runRaw);

  const InverterConnectionConfig currentConfig = getInverterConnectionConfig();
  if (!currentConfig.enabled || currentConfig.ip != config.ip) return;

  xSemaphoreTake(invMutex, portMAX_DELAY);
  invdbg.valid = ok;
  if (ok) {
    for (int k = 0; k < 4; k++) { invdbg.ap[k] = ap[k]; invdbg.p[k] = p[k]; }
    invdbg.commStatus = commStatus;
    invdbg.lastOkMs = millis();
    memcpy(invdbg.meterRaw, meterRaw, sizeof(meterRaw));
  }
  invdbg.runValid = runOk;
  if (runOk) {
    invdbg.gridActivePower = gridActivePower;
    for (int k = 0; k < 4; k++) {
      invdbg.backupP[k] = backupP[k]; invdbg.loadP[k] = loadP[k]; invdbg.pvP[k] = pvP[k];
    }
    invdbg.inverterAcP = inverterAcP;
    invdbg.batteryP = batteryP;
    invdbg.batteryMode = batteryMode;
    invdbg.runLastOkMs = millis();
    memcpy(invdbg.runRaw, runRaw, sizeof(runRaw));
  }
  xSemaphoreGive(invMutex);

#ifdef DEBUG_MODBUS
  if (ok) {
    logPrintf("[INV] commStatus=%d activeP=%d/%d/%d/%d W meterP=%d/%d/%d/%d W\n",
              commStatus, ap[0], ap[1], ap[2], ap[3], (int)p[0], (int)p[1], (int)p[2], (int)p[3]);
  } else {
    logPrintf("[INV] Modbus TCP read failed (%s:%d)\n", config.ip.c_str(), INVERTER_MODBUS_PORT);
  }
  if (runOk) {
    logPrintf("[INV-RUN] grid=%d W backup=%d/%d/%d/%d W load=%d/%d/%d/%d W\n",
              gridActivePower, backupP[0], backupP[1], backupP[2], backupP[3],
              loadP[0], loadP[1], loadP[2], loadP[3]);
  } else {
    logPrintf("[INV-RUN] Modbus TCP running-data read failed (%s:%d)\n", config.ip.c_str(), INVERTER_MODBUS_PORT);
  }
#endif
}

// ───────────────────────── WEB CONFIG ────────────────────────────
static bool checkAuth() {
  if (!server.authenticate(configUser.c_str(), configPassword.c_str())) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

static void failOtaUpload(const String &message) {
  otaUploadFailed = true;
  otaError = message;
  logPrintln("[OTA] " + message);
}

static void handleOtaUpload() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    otaUploadAuthorized = server.authenticate(configUser.c_str(), configPassword.c_str());
    otaUploadFailed = false;
    otaUploadComplete = false;
    otaError = "";
    if (!otaUploadAuthorized) return;

    otaResumeAfterFailure = !maintenancePaused;
    maintenancePaused = true;
    const uint32_t pauseStartedMs = millis();
    while (sdReady && !sdPauseAcknowledged && millis() - pauseStartedMs < 5000UL) delay(20);
    if (sdReady && !sdPauseAcknowledged) {
      failOtaUpload("Could not pause SD logging");
      return;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      failOtaUpload("Could not start firmware update (error " + String(Update.getError()) + ")");
      return;
    }
    logPrintf("[OTA] Receiving %s\n", upload.filename.c_str());
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!otaUploadAuthorized || otaUploadFailed) return;
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      failOtaUpload("Firmware write failed (error " + String(Update.getError()) + ")");
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!otaUploadAuthorized) return;
    if (!otaUploadFailed && !Update.end(true)) {
      failOtaUpload("Firmware validation failed (error " + String(Update.getError()) + ")");
    } else if (!otaUploadFailed) {
      otaUploadComplete = true;
      logPrintf("[OTA] Firmware accepted: %u bytes\n", (unsigned)upload.totalSize);
    }
    if (otaUploadFailed) Update.abort();
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (!otaUploadAuthorized) return;
    Update.abort();
    failOtaUpload("Firmware upload was aborted");
  }
}

static void handleOtaComplete() {
  if (!checkAuth()) return;

  if (!otaUploadAuthorized || !otaUploadComplete || otaUploadFailed || Update.hasError()) {
    if (otaResumeAfterFailure) maintenancePaused = false;
    const String message = otaError.length() ? otaError : "Firmware update failed";
    server.send(500, "text/plain", message);
    return;
  }

  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", "Firmware installed. Restarting controller...");
  otaRestartAtMs = millis() + 1000UL;
}

static void handleRoot() {
  if (!checkAuth()) return;

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html", (PGM_P)WEB_INDEX_HTML_GZ, WEB_INDEX_HTML_GZ_LEN);

}

static void handleLog() {
  if (!checkAuth()) return;
  static char snap[LOG_BUF_SIZE + 1];   // scratch buffer, only used here (single caller)
  xSemaphoreTake(logMutex, portMAX_DELAY);
  size_t n     = logCount;
  size_t first = min(n, LOG_BUF_SIZE - logHead);
  memcpy(snap, logBuf + logHead, first);
  if (n > first) memcpy(snap + first, logBuf, n - first);
  xSemaphoreGive(logMutex);
  snap[n] = '\0';
  server.send(200, "text/plain", snap);
}

// Register names below are from the GoodWe ET/EH Modbus TCP map (marcelblijleven/goodwe
// OSS library, et.py). "(hi)"/"(lo)" mark the two registers of a 32-bit value.
static const char *const meterRegNames[45] = {
  "commode", "rssi", "manufacture_code", "meter_test_status", "meter_comm_status",
  "active_power1 (L1)", "active_power2 (L2)", "active_power3 (L3)", "active_power_total",
  "reactive_power_total", "meter_power_factor1", "meter_power_factor2", "meter_power_factor3",
  "meter_power_factor_total", "meter_freq",
  "meter_e_total_exp (hi)", "meter_e_total_exp (lo)", "meter_e_total_imp (hi)", "meter_e_total_imp (lo)",
  "meter_active_power1 (hi)", "meter_active_power1 (lo)", "meter_active_power2 (hi)", "meter_active_power2 (lo)",
  "meter_active_power3 (hi)", "meter_active_power3 (lo)", "meter_active_power_total (hi)", "meter_active_power_total (lo)",
  "meter_reactive_power1 (hi)", "meter_reactive_power1 (lo)", "meter_reactive_power2 (hi)", "meter_reactive_power2 (lo)",
  "meter_reactive_power3 (hi)", "meter_reactive_power3 (lo)", "meter_reactive_power_total (hi)", "meter_reactive_power_total (lo)",
  "meter_apparent_power1 (hi)", "meter_apparent_power1 (lo)", "meter_apparent_power2 (hi)", "meter_apparent_power2 (lo)",
  "meter_apparent_power3 (hi)", "meter_apparent_power3 (lo)", "meter_apparent_power_total (hi)", "meter_apparent_power_total (lo)",
  "meter_type", "meter_sw_version"
};

static const char *const runRegNames[125] = {
  "timestamp (yy/mm)", "timestamp (dd/hh)", "timestamp (mm/ss)",
  "vpv1", "ipv1", "ppv1 (hi)", "ppv1 (lo)",
  "vpv2", "ipv2", "ppv2 (hi)", "ppv2 (lo)",
  "vpv3", "ipv3", "ppv3 (hi)", "ppv3 (lo)",
  "vpv4", "ipv4", "ppv4 (hi)", "ppv4 (lo)",
  "pv4_mode/pv3_mode", "pv2_mode/pv1_mode",
  "vgrid (L1)", "igrid (L1)", "fgrid (L1)", "reserved", "pgrid (L1)",
  "vgrid2 (L2)", "igrid2 (L2)", "fgrid2 (L2)", "reserved", "pgrid2 (L2)",
  "vgrid3 (L3)", "igrid3 (L3)", "fgrid3 (L3)", "reserved", "pgrid3 (L3)",
  "grid_mode", "reserved", "total_inverter_power", "reserved",
  "active_power (Grid)", "reserved", "reactive_power", "reserved", "apparent_power",
  "backup_v1", "backup_i1", "backup_f1", "load_mode1", "reserved", "backup_p1",
  "backup_v2", "backup_i2", "backup_f2", "load_mode2", "reserved", "backup_p2",
  "backup_v3", "backup_i3", "backup_f3", "load_mode3", "reserved", "backup_p3",
  "reserved", "load_p1", "reserved", "load_p2", "reserved", "load_p3", "reserved",
  "backup_ptotal", "reserved", "load_ptotal",
  "ups_load", "temperature_air", "temperature_module", "temperature", "function_bit",
  "bus_voltage", "nbus_voltage", "vbattery1", "ibattery1", "pbattery1 (hi)", "pbattery1 (lo)",
  "battery_mode", "warning_code", "safety_country", "work_mode", "operation_mode",
  "error_codes (hi)", "error_codes (lo)",
  "e_total (hi)", "e_total (lo)", "e_day (hi)", "e_day (lo)",
  "e_total_exp (hi)", "e_total_exp (lo)", "h_total (hi)", "h_total (lo)", "e_day_exp",
  "e_total_imp (hi)", "e_total_imp (lo)", "e_day_imp",
  "e_load_total (hi)", "e_load_total (lo)", "e_load_day",
  "e_bat_charge_total (hi)", "e_bat_charge_total (lo)", "e_bat_charge_day",
  "e_bat_discharge_total (hi)", "e_bat_discharge_total (lo)", "e_bat_discharge_day",
  "reserved/unknown", "reserved/unknown", "reserved/unknown", "reserved/unknown",
  "reserved/unknown", "reserved/unknown", "reserved/unknown", "reserved/unknown",
  "diagnose_result (hi)", "diagnose_result (lo)",
  "reserved/unknown", "reserved/unknown", "reserved/unknown"
};

// Plain-text dump of every raw register from both Modbus TCP blocks we poll,
// for manual inspection of what the inverter actually exposes.
static void handleInvRaw() {
  if (!checkAuth()) return;
  xSemaphoreTake(invMutex, portMAX_DELAY);
  InverterMeterDebug inv = invdbg;
  xSemaphoreGive(invMutex);

  String out;
  out.reserve(20000);
  char line[96];

  snprintf(line, sizeof(line), "Meter-data block (0x8CA0/36000, 45 regs) - valid=%s\n",
           inv.valid ? "true" : "false");
  out += line;
  out += "reg    hex     u16    s16    name\n";
  for (int k = 0; k < 45; k++) {
    uint16_t v = inv.meterRaw[k];
    snprintf(line, sizeof(line), "%5d  0x%04X  %6u  %6d    %s\n",
             36000 + k, v, v, (int16_t)v, meterRegNames[k]);
    out += line;
  }

  snprintf(line, sizeof(line), "\nRunning-data block (0x891C/35100, 125 regs) - valid=%s\n",
           inv.runValid ? "true" : "false");
  out += line;
  out += "reg    hex     u16    s16    name\n";
  for (int k = 0; k < 125; k++) {
    uint16_t v = inv.runRaw[k];
    snprintf(line, sizeof(line), "%5d  0x%04X  %6u  %6d    %s\n",
             35100 + k, v, v, (int16_t)v, runRegNames[k]);
    out += line;
  }

  server.send(200, "text/plain", out);
}

static void handleData() {
  if (!checkAuth()) return;
  xSemaphoreTake(meterMutex, portMAX_DELAY);
  P1Debug d = p1dbg;
  MeterData m = meter;
  xSemaphoreGive(meterMutex);

  xSemaphoreTake(invMutex, portMAX_DELAY);
  InverterMeterDebug inv = invdbg;
  xSemaphoreGive(invMutex);
  uint32_t invAge = inv.lastOkMs ? (millis() - inv.lastOkMs) / 1000UL : 0;
  uint32_t runAge = inv.runLastOkMs ? (millis() - inv.runLastOkMs) / 1000UL : 0;

  char buf[720];
  snprintf(buf, sizeof(buf),
    "{\"p1\":{\"v\":[%.1f,%.1f,%.1f],\"i\":[%.2f,%.2f,%.2f],\"p\":[%d,%d,%d]},"
    "\"reg\":{\"v\":[%.1f,%.1f,%.1f],\"i\":[%.2f,%.2f,%.2f],\"p\":[%d,%d,%d]},"
    "\"inv\":{\"ap\":[%d,%d,%d,%d],\"p\":[%d,%d,%d,%d],\"comm\":%d,\"valid\":%s,\"age\":%lu},"
    "\"run\":{\"grid\":%d,\"backup\":[%d,%d,%d,%d],\"load\":[%d,%d,%d,%d],\"valid\":%s,\"age\":%lu}}",
    d.v[0], d.v[1], d.v[2], d.i[0], d.i[1], d.i[2],
    (int)lroundf(d.p[0]), (int)lroundf(d.p[1]), (int)lroundf(d.p[2]),
    m.Va / 10.0f, m.Vb / 10.0f, m.Vc / 10.0f, m.Ia / 100.0f, m.Ib / 100.0f, m.Ic / 100.0f,
    (int)m.Pa, (int)m.Pb, (int)m.Pc,
    (int)inv.ap[0], (int)inv.ap[1], (int)inv.ap[2], (int)inv.ap[3],
    (int)inv.p[0], (int)inv.p[1], (int)inv.p[2], (int)inv.p[3],
    (int)inv.commStatus, inv.valid ? "true" : "false", (unsigned long)invAge,
    (int)inv.gridActivePower,
    (int)inv.backupP[0], (int)inv.backupP[1], (int)inv.backupP[2], (int)inv.backupP[3],
    (int)inv.loadP[0], (int)inv.loadP[1], (int)inv.loadP[2], (int)inv.loadP[3],
    inv.runValid ? "true" : "false", (unsigned long)runAge);
  server.send(200, "application/json", buf);
}

static void handleSave() {
  if (!checkAuth()) return;
  String ip = server.arg("ip");
  ip.trim();
  if (!isValidIp(ip)) {
    server.send(400, "text/plain", "Invalid IP address");
    return;
  }
  setP1Ip(ip);
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleP1Ip() {
  if (!checkAuth()) return;
  String ip = server.arg("ip");
  ip.trim();
  if (!isValidIp(ip)) {
    server.send(400, "text/plain", "Invalid IP address");
    return;
  }
  setP1Ip(ip);
  server.send(200, "text/plain", "Grid Meter (P1) IP changed to " + ip);
}

static void handleInverterConfig() {
  if (!checkAuth()) return;
  String ip = server.arg("ip");
  ip.trim();
  if (!isValidIp(ip)) {
    server.send(400, "text/plain", "Invalid inverter IP address");
    return;
  }
  const bool enabled = server.arg("enabled") == "true" || server.arg("enabled") == "1";
  setInverterConnectionConfig(ip, enabled);
  xSemaphoreTake(invMutex, portMAX_DELAY);
  invdbg.valid = false;
  invdbg.runValid = false;
  invdbg.lastOkMs = 0;
  invdbg.runLastOkMs = 0;
  xSemaphoreGive(invMutex);
  server.send(200, "text/plain", enabled
    ? "Inverter Modbus TCP enabled at " + ip
    : "Inverter Modbus TCP disabled");
}

static bool parseGridMeterConnectionArgs(GridMeterConnection &connection) {
  if (!server.hasArg("meterType") || !parseGridMeterType(server.arg("meterType"), connection.type)) {
    return false;
  }
  connection.ip = server.arg("meterIp");
  connection.ip.trim();
  if (!isValidIp(connection.ip)) return false;
  return true;
}

static void clearInverterDebugState() {
  xSemaphoreTake(invMutex, portMAX_DELAY);
  invdbg.valid = false;
  invdbg.runValid = false;
  invdbg.lastOkMs = 0;
  invdbg.runLastOkMs = 0;
  xSemaphoreGive(invMutex);
}

static void handleSettings() {
  if (!checkAuth()) return;
  GridMeterConnection meterConnection;
  if (!parseGridMeterConnectionArgs(meterConnection)) {
    server.send(400, "text/plain", "Invalid grid meter type or IP address");
    return;
  }
  String inverterAddress = server.arg("inverterIp");
  inverterAddress.trim();
  if (!isValidIp(inverterAddress)) {
    server.send(400, "text/plain", "Invalid inverter IP address");
    return;
  }
  const bool inverterEnabled = server.arg("inverterEnabled") == "true" ||
                               server.arg("inverterEnabled") == "1";
  setGridMeterConnection(meterConnection);
  setInverterConnectionConfig(inverterAddress, inverterEnabled);
  clearInverterDebugState();
  server.send(200, "text/plain", String(gridMeterTypeName(meterConnection.type)) +
              " selected; waiting for a valid reading");
}

static void handleTestGridMeter() {
  if (!checkAuth()) return;
  GridMeterConnection connection;
  if (!parseGridMeterConnectionArgs(connection)) {
    server.send(400, "text/plain", "Invalid grid meter type or IP address");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "text/plain", "Wi-Fi is not connected");
    return;
  }
  GridMeterSample sample = {};
  int httpStatus = 0;
  if (!fetchGridMeterSample(connection, P1_HTTP_TIMEOUT, sample, httpStatus)) {
    server.send(502, "text/plain", "Grid meter test failed (HTTP " + String(httpStatus) + ")");
    return;
  }
  server.send(200, "text/plain", String(gridMeterTypeName(connection.type)) +
              " connected: " + String(lroundf(sample.totalActivePowerW)) + " W total");
}

static void handleReset() {
  if (!checkAuth()) return;
  setP1Ip(P1_IP_DEFAULT);
  server.sendHeader("Location", "/");
  server.send(303);
}

static bool parseFloatArg(const char *name, float minimum, float maximum, float &value) {
  if (!server.hasArg(name)) return false;
  const String text = server.arg(name);
  char *end = nullptr;
  value = strtof(text.c_str(), &end);
  return end && end != text.c_str() && *end == '\0' && isfinite(value) &&
         value >= minimum && value <= maximum;
}

static bool parseUintArg(const char *name, uint32_t minimum, uint32_t maximum, uint32_t &value) {
  if (!server.hasArg(name)) return false;
  const String text = server.arg(name);
  char *end = nullptr;
  const unsigned long parsed = strtoul(text.c_str(), &end, 10);
  if (!end || end == text.c_str() || *end != '\0' || parsed < minimum || parsed > maximum) return false;
  value = (uint32_t)parsed;
  return true;
}

static void handleControl() {
  if (!checkAuth()) return;
  ControlConfig next;
  if (!parseFloatArg("normalGain", 0.0f, CONTROL_NORMAL_GAIN_MAX, next.normalGain) ||
      !parseFloatArg("stepGain", 0.0f, CONTROL_FEEDBACK_GAIN, next.stepGain) ||
      !parseFloatArg("brakeGain", 0.0f, CONTROL_FEEDBACK_GAIN, next.brakeGain) ||
      !parseFloatArg("stepThreshold", 50.0f, 5000.0f, next.stepThresholdW) ||
      !parseFloatArg("targetImport", -500.0f, 500.0f, next.targetImportW) ||
      !parseUintArg("holdMs", 0, 15000, next.holdMs) ||
      !parseUintArg("rampMs", 1000, 60000, next.rampMs) ||
      !parseUintArg("freezeMs", 0, 120000, next.integralFreezeMs)) {
    server.send(400, "text/plain", "Invalid settings. Normal gain must be 0.00-1.00; Step and Brake gains must be 0.00-0.33.");
    return;
  }

  xSemaphoreTake(controlMutex, portMAX_DELAY);
  controlConfig = next;
  xSemaphoreGive(controlMutex);
  prefs.putFloat("normGain", next.normalGain);
  prefs.putFloat("stepGain", next.stepGain);
  prefs.putFloat("brakeGain", next.brakeGain);
  prefs.putFloat("stepW", next.stepThresholdW);
  prefs.putFloat("targetW", next.targetImportW);
  prefs.putULong("holdMs", next.holdMs);
  prefs.putULong("rampMs", next.rampMs);
  prefs.putULong("freezeMs", next.integralFreezeMs);
  server.send(200, "text/plain", "Settings applied and saved");
}

static void handlePause() {
  if (!checkAuth()) return;
  maintenancePaused = true;
  uint32_t started = millis();
  while (sdReady && !sdPauseAcknowledged && millis() - started < 5000) delay(20);
  if (sdReady && !sdPauseAcknowledged) {
    server.send(500, "text/plain", "Could not pause SD logging; emulator remains paused");
    return;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleResume() {
  if (!checkAuth()) return;
  maintenancePaused = false;
  server.sendHeader("Location", "/");
  server.send(303);
}

static void handleState() {
  if (!checkAuth()) return;
  xSemaphoreTake(meterMutex, portMAX_DELAY);
  const bool p1Reachable = meter.valid;
  const uint32_t p1Age = meter.lastOkMs ? millis() - meter.lastOkMs : UINT32_MAX;
  xSemaphoreGive(meterMutex);
  const uint32_t rs485Age = lastRs485Ms ? millis() - lastRs485Ms : UINT32_MAX;
  const bool inverterListening = rs485Age <= RS485_LINK_TIMEOUT_MS;
  const InverterConnectionConfig inverterConfig = getInverterConnectionConfig();
  xSemaphoreTake(invMutex, portMAX_DELAY);
  const uint32_t inverterLastOkMs = max(invdbg.lastOkMs, invdbg.runLastOkMs);
  const bool inverterReadValid = invdbg.valid || invdbg.runValid;
  xSemaphoreGive(invMutex);
  const uint32_t inverterModbusAge = inverterLastOkMs ? millis() - inverterLastOkMs : UINT32_MAX;
  const bool inverterModbusOk = inverterConfig.enabled && inverterReadValid &&
                                 inverterModbusAge <= INVERTER_STATUS_TIMEOUT_MS;
  const ControlConfig config = getControlConfig();
  const char *transientName = controlTransientState == 1 ? "step hold" :
                              controlTransientState == 2 ? "crossing brake" :
                              controlTransientState == 3 ? "gain ramp" : "normal";
  const GridMeterConnection meterConnection = getGridMeterConnection();
  char state[1200];
  snprintf(state, sizeof(state),
           "{\"firmwareVersion\":\"%s\",\"paused\":%s,\"downloadReady\":%s,\"sd\":%s,\"p1Reachable\":%s,"
           "\"p1AgeMs\":%lu,\"p1Ip\":\"%s\",\"gridMeterType\":\"%s\","
           "\"gridMeterName\":\"%s\","
           "\"inverterListening\":%s,"
           "\"inverterModbusEnabled\":%s,\"inverterModbusOk\":%s,"
           "\"inverterModbusAgeMs\":%lu,\"inverterIp\":\"%s\","
           "\"bias\":%.2f,\"averageRaw\":%.1f,\"effectiveGain\":%.3f,\"transient\":\"%s\","
           "\"config\":{\"normalGain\":%.3f,\"stepGain\":%.3f,\"brakeGain\":%.3f,"
           "\"stepThreshold\":%.0f,\"holdMs\":%lu,\"rampMs\":%lu,\"freezeMs\":%lu,\"targetImport\":%.0f},"
           "\"rs485AgeMs\":%lu,\"meterAddress\":%u,"
           "\"csv\":\"%s\",\"csvBytes\":%lu,\"csvRows\":%lu,"
           "\"csvFirstUptimeMs\":%lu,\"csvLatestUptimeMs\":%lu}",
           FIRMWARE_VERSION, maintenancePaused ? "true" : "false",
           sdPauseAcknowledged ? "true" : "false",
           sdReady ? "true" : "false", p1Reachable ? "true" : "false",
           (unsigned long)p1Age, meterConnection.ip.c_str(), gridMeterTypeKey(meterConnection.type),
           gridMeterTypeName(meterConnection.type), inverterListening ? "true" : "false",
           inverterConfig.enabled ? "true" : "false", inverterModbusOk ? "true" : "false",
           (unsigned long)inverterModbusAge, inverterConfig.ip.c_str(),
           controlBiasW, controlAverageRawW,
           controlEffectiveGain, transientName,
           config.normalGain, config.stepGain, config.brakeGain, config.stepThresholdW,
           (unsigned long)config.holdMs, (unsigned long)config.rampMs,
           (unsigned long)config.integralFreezeMs, config.targetImportW,
           (unsigned long)rs485Age, activeMeterAddress,
           sdLogPath.c_str(), (unsigned long)sdWrittenBytes, (unsigned long)sdLoggedRows,
           (unsigned long)sdFirstUptimeMs, (unsigned long)sdLatestUptimeMs);
  server.send(200, "application/json", state);
}

static void handleInverterStatus() {
  if (!checkAuth()) return;
  const InverterConnectionConfig config = getInverterConnectionConfig();
  xSemaphoreTake(invMutex, portMAX_DELAY);
  const InverterMeterDebug inv = invdbg;
  xSemaphoreGive(invMutex);

  const uint32_t nowMs = millis();
  const uint32_t meterAgeMs = inv.lastOkMs ? nowMs - inv.lastOkMs : UINT32_MAX;
  const uint32_t runAgeMs = inv.runLastOkMs ? nowMs - inv.runLastOkMs : UINT32_MAX;
  const bool meterOk = config.enabled && inv.valid && meterAgeMs <= INVERTER_STATUS_TIMEOUT_MS;
  const bool runOk = config.enabled && inv.runValid && runAgeMs <= INVERTER_STATUS_TIMEOUT_MS;

  JsonDocument doc;
  doc["enabled"] = config.enabled;
  doc["connected"] = meterOk || runOk;
  doc["ip"] = config.ip;
  doc["port"] = INVERTER_MODBUS_PORT;
  doc["unitId"] = INVERTER_UNIT_ID;
  doc["meterBlockOk"] = meterOk;
  doc["runningBlockOk"] = runOk;
  doc["meterAgeMs"] = meterAgeMs;
  doc["runningAgeMs"] = runAgeMs;
  doc["meterCommStatus"] = inv.commStatus;
  doc["gridPowerW"] = inv.gridActivePower;
  doc["inverterAcPowerW"] = inv.inverterAcP;
  doc["batteryPowerW"] = inv.batteryP;
  doc["batteryMode"] = inv.batteryMode;
  const char *batteryState = inv.batteryMode == 0 ? "No battery" :
                             inv.batteryMode == 1 ? "Standby" :
                             inv.batteryMode == 2 ? "Discharging" :
                             inv.batteryMode == 3 ? "Charging" :
                             inv.batteryMode == 4 ? "Waiting to charge" : "Unknown";
  doc["batteryState"] = batteryState;
  JsonArray pv = doc["pvPowerW"].to<JsonArray>();
  JsonArray backup = doc["backupPowerW"].to<JsonArray>();
  JsonArray load = doc["loadPowerW"].to<JsonArray>();
  int64_t pvTotal = 0;
  for (int index = 0; index < 4; index++) {
    pv.add(inv.pvP[index]);
    backup.add(inv.backupP[index]);
    load.add(inv.loadP[index]);
    pvTotal += inv.pvP[index];
  }
  doc["pvTotalW"] = pvTotal;

  String response;
  response.reserve(640);
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

static void handleStartLogging() {
  if (!checkAuth()) return;
  if (sdReady) {
    server.send(200, "text/plain", "CSV logging is already active");
    return;
  }

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  SD.end();
  const bool cardReady = SD.begin(SD_CS_PIN, sdSpi, SD_SPI_HZ);
  if (!cardReady || SD.cardType() == CARD_NONE) {
    xSemaphoreGive(sdMutex);
    server.send(503, "text/plain", "No SD card detected. Insert a FAT32 card and try again.");
    return;
  }

  sdLogPath = "";
  for (int index = 0; index < 1000; index++) {
    char path[16];
    snprintf(path, sizeof(path), "/p1gw%03d.csv", index);
    if (!SD.exists(path)) { sdLogPath = path; break; }
  }
  if (sdLogPath.isEmpty()) {
    xSemaphoreGive(sdMutex);
    server.send(507, "text/plain", "No free CSV filename is available on the SD card");
    return;
  }

  sdLogFile = SD.open(sdLogPath, FILE_WRITE);
  if (!sdLogFile) {
    xSemaphoreGive(sdMutex);
    server.send(500, "text/plain", "SD card detected, but the CSV file could not be created");
    return;
  }
  sdWrittenBytes = sdLogFile.println(
    "uptime_ms,wall_time,p1_raw_l1_w,p1_raw_l2_w,p1_raw_l3_w,p1_raw_total_w,"
    "p1_raw_l1_a,p1_raw_l2_a,p1_raw_l3_a,p1_l1_v,p1_l2_v,p1_l3_v,"
    "meter_l1_w,meter_l2_w,meter_l3_w,meter_total_w,meter_l1_a,meter_l2_a,meter_l3_a,"
    "inv_active_l1_w,inv_active_l2_w,inv_active_l3_w,inv_active_total_w,"
    "inv_meter_l1_w,inv_meter_l2_w,inv_meter_l3_w,inv_meter_total_w,inv_comm_status,"
    "inv_grid_w,backup_l1_w,backup_l2_w,backup_l3_w,backup_total_w,"
    "load_l1_w,load_l2_w,load_l3_w,load_total_w,inv_age_ms,run_age_ms,inv_valid,run_valid,dropped_rows,"
    "control_average_raw_w,control_bias_w,control_held,rs485_age_ms,active_meter_address,"
    "pv1_w,pv2_w,pv3_w,pv4_w,pv_total_w,inverter_ac_w,battery_w,"
    "effective_feedback_gain,transient_state,normal_gain,step_gain,brake_gain,step_threshold_w,"
    "hold_ms,ramp_ms,integral_freeze_ms,target_import_w,average_tau_ms,integral_time_ms,bias_limit_w,phase_rotation");
  sdLogFile.flush();
  sdDroppedRows = 0;
  sdLoggedRows = 0;
  sdFirstUptimeMs = 0;
  sdLatestUptimeMs = 0;
  xQueueReset(sdLogQueue);
  sdReady = true;
  xSemaphoreGive(sdMutex);
  logPrintf("SD logging started: %s\n", sdLogPath.c_str());
  server.send(200, "text/plain", "CSV logging started: " + sdLogPath);
}

static bool normalizeLogFilename(String &name) {
  name.trim();
  name.toLowerCase();
  if (name.startsWith("/")) name.remove(0, 1);
  return name.length() == 11 && name.startsWith("p1gw") &&
         isDigit(name[4]) && isDigit(name[5]) && isDigit(name[6]) &&
         name.substring(7) == ".csv";
}

static void handleLogs() {
  if (!checkAuth()) return;
  if (!sdReady) {
    server.send(503, "text/plain", "SD logging is unavailable");
    return;
  }

  String html =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>CSV logs</title><style>body{font-family:sans-serif;max-width:620px;"
    "margin:40px auto;padding:0 16px;color:#222}table{width:100%;border-collapse:collapse}"
    "th,td{text-align:left;padding:9px;border-bottom:1px solid #ddd}td:nth-child(2){text-align:right}"
    "a{color:#1769aa}.current{font-weight:bold}</style></head><body>"
    "<h2>CSV logs</h2><p><a href='/'>Back to dashboard</a></p>"
    "<table><tr><th>File</th><th>Size</th></tr>";

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    xSemaphoreGive(sdMutex);
    server.send(500, "text/plain", "Could not read the SD directory");
    return;
  }

  for (;;) {
    File entry = root.openNextFile();
    if (!entry) break;

    String name = entry.name();
    const size_t size = entry.size();
    const bool isDirectory = entry.isDirectory();
    entry.close();
    if (isDirectory || !normalizeLogFilename(name)) continue;

    const String path = "/" + name;
    String row = "<tr";
    if (path == sdLogPath) row += " class='current'";
    row += "><td><a href='/sdlog?file=" + name + "'>" + name + "</a>";
    if (path == sdLogPath) row += " (current)";
    row += "</td><td>" + String((unsigned long)size) + " bytes</td></tr>";
    html += row;
  }
  root.close();
  xSemaphoreGive(sdMutex);
  html += "</table></body></html>";
  server.send(200, "text/html", html);
}

static void handleSdLog() {
  if (!checkAuth()) return;
  if (!sdReady || sdLogPath.isEmpty()) {
    server.send(503, "text/plain", "SD logging is unavailable");
    return;
  }

  String filename = server.hasArg("file") ? server.arg("file") : sdLogPath;
  if (!normalizeLogFilename(filename)) {
    server.send(400, "text/plain", "Invalid CSV filename");
    return;
  }
  const String path = "/" + filename;
  const bool resumeAfterDownload = !maintenancePaused;

  if (!sdPauseAcknowledged) {
    maintenancePaused = true;
    uint32_t started = millis();
    while (!sdPauseAcknowledged && millis() - started < 5000) delay(20);
    if (!sdPauseAcknowledged) {
      if (resumeAfterDownload) maintenancePaused = false;
      server.send(500, "text/plain", "Could not stop SD logging for download");
      return;
    }
  }

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File download = SD.open(path, FILE_READ);
  if (!download) {
    xSemaphoreGive(sdMutex);
    if (resumeAfterDownload) maintenancePaused = false;
    server.send(404, "text/plain", "CSV log not found");
    return;
  }
  const size_t downloadSize = download.size();
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
  server.setContentLength(downloadSize);
  server.send(200, "text/csv", "");

  NetworkClient &client = server.client();
  client.setTimeout(3000);
  client.setNoDelay(true);
  uint8_t *buffer = (uint8_t *)malloc(SD_DOWNLOAD_BUFFER_SIZE);
  size_t sent = 0;
  const uint32_t downloadStartedMs = millis();
  if (buffer) {
    while (sent < downloadSize && client.connected()) {
      const size_t wanted = min((size_t)SD_DOWNLOAD_BUFFER_SIZE, downloadSize - sent);
      const size_t readBytes = download.read(buffer, wanted);
      if (!readBytes) break;
      const size_t written = client.write(buffer, readBytes);
      sent += written;
      if (written != readBytes) break;
      yield();
    }
    free(buffer);
  }
  download.close();
  xSemaphoreGive(sdMutex);
  if (resumeAfterDownload) maintenancePaused = false;
  logPrintf("[SD] Download %s: %lu/%lu bytes in %lu ms\n", filename.c_str(),
            (unsigned long)sent, (unsigned long)downloadSize,
            (unsigned long)(millis() - downloadStartedMs));
}

// ───────────────────────── TASKS ─────────────────────────────────
static void sdLogTask(void *) {
  char *batch = (char *)malloc(SD_BATCH_SIZE);
  if (!batch) {
    sdReady = false;
    logPrintln("SD logging stopped: could not allocate batch buffer.");
    vTaskDelete(nullptr);
  }

  SdLogSample sample;
  size_t batchLen = 0;
  uint32_t lastFlush = millis();
  for (;;) {
    if (!sdReady) {
      batchLen = 0;
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (maintenancePaused) {
      if (!sdPauseAcknowledged) {
        xSemaphoreTake(sdMutex, portMAX_DELAY);
        size_t written = batchLen ? sdLogFile.write((const uint8_t *)batch, batchLen) : 0;
        sdWrittenBytes = sdWrittenBytes + written;
        sdLogFile.flush();
        sdLogFile.close();
        xSemaphoreGive(sdMutex);
        if (batchLen && written != batchLen) {
          sdReady = false;
          logPrintln("SD logging stopped: pause flush failed.");
        }
        batchLen = 0;
        xQueueReset(sdLogQueue);
        sdPauseAcknowledged = true;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    if (sdPauseAcknowledged) {
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      sdLogFile = SD.open(sdLogPath, FILE_APPEND);
      xSemaphoreGive(sdMutex);
      if (!sdLogFile) {
        sdReady = false;
        logPrintln("SD logging stopped: could not reopen log after maintenance.");
        sdPauseAcknowledged = false;
        batchLen = 0;
        xQueueReset(sdLogQueue);
        continue;
      }
      sdPauseAcknowledged = false;
      lastFlush = millis();
    }
    if (xQueueReceive(sdLogQueue, &sample, pdMS_TO_TICKS(1000)) == pdTRUE && sdReady) {
      char wallTime[24] = "unsynced";
      if (sample.unixTime) {
        const time_t epoch = sample.unixTime;
        struct tm localTime;
        localtime_r(&epoch, &localTime);
        strftime(wallTime, sizeof(wallTime), "%Y-%m-%dT%H:%M:%S", &localTime);
      }
      const int32_t pvTotal = sample.invPvP[0] + sample.invPvP[1] +
                              sample.invPvP[2] + sample.invPvP[3];
      char line[768];
      int lineLen = snprintf(line, sizeof(line),
        "%lu,%s,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,"
        "%ld,%ld,%ld,%ld,%.2f,%.2f,%.2f,"
        "%d,%d,%d,%d,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
        "%lu,%lu,%u,%u,%lu,%.1f,%.2f,%u,%lu,%u,"
        "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.3f,%u,%.3f,%.3f,%.3f,%.0f,%lu,%lu,%lu,%.1f,%.1f,%.1f,%.1f,%d\n",
        (unsigned long)sample.uptimeMs,
        wallTime,
        sample.rawP[0], sample.rawP[1], sample.rawP[2], sample.rawP[0] + sample.rawP[1] + sample.rawP[2],
        sample.rawI[0], sample.rawI[1], sample.rawI[2],
        sample.voltage[0] / 10.0f, sample.voltage[1] / 10.0f, sample.voltage[2] / 10.0f,
        (long)sample.meterP[0], (long)sample.meterP[1], (long)sample.meterP[2],
        (long)(sample.meterP[0] + sample.meterP[1] + sample.meterP[2]),
        sample.meterI[0] / 100.0f, sample.meterI[1] / 100.0f, sample.meterI[2] / 100.0f,
        sample.invActiveP[0], sample.invActiveP[1], sample.invActiveP[2], sample.invActiveP[3],
        (long)sample.invMeterP[0], (long)sample.invMeterP[1], (long)sample.invMeterP[2], (long)sample.invMeterP[3],
        sample.invCommStatus, sample.invGridP,
        sample.invBackupP[0], sample.invBackupP[1], sample.invBackupP[2], sample.invBackupP[3],
        sample.invLoadP[0], sample.invLoadP[1], sample.invLoadP[2], sample.invLoadP[3],
        (unsigned long)sample.invAgeMs, (unsigned long)sample.runAgeMs,
        sample.invValid ? 1 : 0, sample.runValid ? 1 : 0, (unsigned long)sdDroppedRows,
        sample.controlAverageRawW, sample.controlBiasW, sample.controlHeld ? 1 : 0,
        (unsigned long)sample.rs485AgeMs, sample.meterAddress,
        (long)sample.invPvP[0], (long)sample.invPvP[1], (long)sample.invPvP[2],
        (long)sample.invPvP[3], (long)pvTotal, (long)sample.invAcP, (long)sample.invBatteryP,
        sample.effectiveFeedbackGain, sample.transientState,
        sample.normalGain, sample.stepGain, sample.brakeGain, sample.stepThresholdW,
        (unsigned long)sample.holdMs, (unsigned long)sample.rampMs,
        (unsigned long)sample.integralFreezeMs, sample.targetImportW, CONTROL_AVERAGE_TAU_MS,
        CONTROL_INTEGRAL_TIME_MS, CONTROL_BIAS_LIMIT_W, PHASE_ROTATION);
      if (lineLen <= 0 || (size_t)lineLen >= sizeof(line)) {
        sdDroppedRows = sdDroppedRows + 1;
      } else {
        if (batchLen + (size_t)lineLen > SD_BATCH_SIZE) {
          xSemaphoreTake(sdMutex, portMAX_DELAY);
          size_t written = sdLogFile.write((const uint8_t *)batch, batchLen);
          sdWrittenBytes = sdWrittenBytes + written;
          sdLogFile.flush();
          xSemaphoreGive(sdMutex);
          if (written != batchLen) {
            sdReady = false;
            logPrintln("SD logging stopped: card write failed.");
            batchLen = 0;
            xQueueReset(sdLogQueue);
            continue;
          }
          batchLen = 0;
          lastFlush = millis();
        }
        memcpy(batch + batchLen, line, lineLen);
        batchLen += lineLen;
        if (sdLoggedRows == 0) sdFirstUptimeMs = sample.uptimeMs;
        sdLatestUptimeMs = sample.uptimeMs;
        sdLoggedRows = sdLoggedRows + 1;
      }
    }
    if (sdReady && batchLen > 0 && millis() - lastFlush >= SD_FLUSH_MS) {
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      size_t written = sdLogFile.write((const uint8_t *)batch, batchLen);
      sdWrittenBytes = sdWrittenBytes + written;
      sdLogFile.flush();
      xSemaphoreGive(sdMutex);
      if (written != batchLen) {
        sdReady = false;
        logPrintln("SD logging stopped: card write failed.");
        batchLen = 0;
        xQueueReset(sdLogQueue);
        continue;
      }
      batchLen = 0;
      lastFlush = millis();
    }
  }
}

static void p1Task(void *) {
  for (;;) {
    if (maintenancePaused) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    const uint32_t requestStartedMs = millis();
    const P1FetchResult result = fetchP1();
    const uint32_t intervalMs = result == P1FetchResult::Changed
                                  ? P1_REFRESH_WAIT_MS
                                  : result == P1FetchResult::Unchanged
                                    ? P1_PROBE_MS
                                    : P1_FAILURE_RETRY_MS;
    const uint32_t elapsedMs = millis() - requestStartedMs;
    if (elapsedMs < intervalMs) vTaskDelay(pdMS_TO_TICKS(intervalMs - elapsedMs));
  }
}

static void invTask(void *) {
  for (;;) {
    if (maintenancePaused) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    fetchInverterMeter();
    vTaskDelay(pdMS_TO_TICKS(INVERTER_POLL_MS));
  }
}

static void modbusTask(void *) {
  uint8_t  buf[MAX_FRAME_LEN];
  size_t   len   = 0;
  uint32_t lastB = 0;
  for (;;) {
    if (rs485.available()) {
      int b = rs485.read();
      if (b >= 0 && len < MAX_FRAME_LEN) { buf[len++] = (uint8_t)b; lastB = millis(); }
    } else {
      if (len > 0 && (millis() - lastB) >= FRAME_TIMEOUT_MS) {
#ifdef DEBUG_MODBUS
        // Logged pre-filter: proves whether anything is on the wire at all,
        // independent of CRC/slave-ID matching done later in processFrame().
        logPrintf("[RAW] %u:", (unsigned)len);
        for (size_t i = 0; i < len; i++) logPrintf(" %02X", buf[i]);
        logPrintln();
#endif
        processFrame(buf, len);
        len = 0;
      }
      vTaskDelay(1);
    }
  }
}

// ───────────────────────── SETUP / LOOP ──────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  logMutex = xSemaphoreCreateMutex();
  logPrintln("\n=== P1 -> GoodWe GMK330 emulator ===");

  // Power and enable the RS485 transceiver (order matters on T-CAN485).
  pinMode(PIN_5V_EN,    OUTPUT); digitalWrite(PIN_5V_EN,    HIGH);
  pinMode(RS485_EN_PIN, OUTPUT); digitalWrite(RS485_EN_PIN, HIGH);
  pinMode(RS485_SE_PIN, OUTPUT); digitalWrite(RS485_SE_PIN, HIGH);
  rs485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  statusLed.begin();
  statusLed.setBrightness(60);
  statusLed.clear();
  statusLed.show();
  xTaskCreatePinnedToCore(ledTask, "led", 2048, nullptr, 1, nullptr, 0);

  meterMutex = xSemaphoreCreateMutex();
  ipMutex     = xSemaphoreCreateMutex();
  invMutex    = xSemaphoreCreateMutex();
  sdMutex     = xSemaphoreCreateMutex();
  controlMutex = xSemaphoreCreateMutex();
  sdLogQueue  = xQueueCreate(SD_LOG_QUEUE_LEN, sizeof(SdLogSample));
  sdSpi.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  prefs.begin("p1cfg", false);
  prefs.remove("meterUser");
  prefs.remove("meterPass");
  if (!prefs.isKey("wifiSsid")) prefs.putString("wifiSsid", WIFI_SSID);
  if (!prefs.isKey("wifiPass")) prefs.putString("wifiPass", WIFI_PASS);
  if (!prefs.isKey("webUser")) prefs.putString("webUser", CONFIG_USER);
  if (!prefs.isKey("webPass")) prefs.putString("webPass", CONFIG_PASS);
  wifiSsid = prefs.getString("wifiSsid", WIFI_SSID);
  wifiPassword = prefs.getString("wifiPass", WIFI_PASS);
  configUser = prefs.getString("webUser", CONFIG_USER);
  configPassword = prefs.getString("webPass", CONFIG_PASS);
  energyCounters.begin(prefs);
  p1Ip = prefs.getString("p1ip", P1_IP_DEFAULT);
  GridMeterType savedMeterType;
  if (parseGridMeterType(prefs.getString("meterType", "homewizard"), savedMeterType)) {
    gridMeterType = savedMeterType;
  }
  inverterIp = prefs.getString("invip", INVERTER_IP_DEFAULT);
  inverterModbusEnabled = prefs.getBool("inven", true);
  if (prefs.getUChar("ctrlVer", 0) < CONTROL_CONFIG_VERSION) {
    prefs.putFloat("normGain", CONTROL_FEEDBACK_GAIN);
    prefs.putFloat("stepGain", CONTROL_DEFAULT_STEP_GAIN);
    prefs.putFloat("brakeGain", CONTROL_DEFAULT_BRAKE_GAIN);
    prefs.putFloat("stepW", CONTROL_STEP_FREEZE_W);
    prefs.putFloat("targetW", CONTROL_TARGET_IMPORT_W);
    prefs.putULong("holdMs", CONTROL_DEFAULT_HOLD_MS);
    prefs.putULong("rampMs", CONTROL_DEFAULT_RAMP_MS);
    prefs.putULong("freezeMs", CONTROL_STEP_FREEZE_MS);
    prefs.putUChar("ctrlVer", CONTROL_CONFIG_VERSION);
  }
  controlConfig.normalGain = constrain(prefs.getFloat("normGain", CONTROL_FEEDBACK_GAIN), 0.0f, CONTROL_NORMAL_GAIN_MAX);
  controlConfig.stepGain = constrain(prefs.getFloat("stepGain", CONTROL_DEFAULT_STEP_GAIN), 0.0f, CONTROL_FEEDBACK_GAIN);
  controlConfig.brakeGain = constrain(prefs.getFloat("brakeGain", CONTROL_DEFAULT_BRAKE_GAIN), 0.0f, CONTROL_FEEDBACK_GAIN);
  controlConfig.stepThresholdW = constrain(prefs.getFloat("stepW", CONTROL_STEP_FREEZE_W), 50.0f, 5000.0f);
  controlConfig.targetImportW = constrain(prefs.getFloat("targetW", CONTROL_TARGET_IMPORT_W), -500.0f, 500.0f);
  controlConfig.holdMs = constrain(prefs.getULong("holdMs", CONTROL_DEFAULT_HOLD_MS), 0UL, 15000UL);
  controlConfig.rampMs = constrain(prefs.getULong("rampMs", CONTROL_DEFAULT_RAMP_MS), 1000UL, 60000UL);
  controlConfig.integralFreezeMs = constrain(prefs.getULong("freezeMs", CONTROL_STEP_FREEZE_MS), 0UL, 120000UL);
  controlEffectiveGain = controlConfig.normalGain;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                // lower, steadier HTTP latency
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  logPrintf("WiFi \"%s\" ", wifiSsid.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    logPrintf(".");
  }
  logPrintln(WiFi.status() == WL_CONNECTED
             ? "\nWiFi connected: " + WiFi.localIP().toString()
             : "\nWiFi not connected (will retry).");
  configTzTime(TIME_ZONE, "pool.ntp.org", "time.nist.gov");

  if (MDNS.begin("gmk330emulator")) {
    logPrintln("Config page: http://GMK330emulator.local/");
  }
  server.on("/",      HTTP_GET,  handleRoot);
  server.on("/save",  HTTP_POST, handleSave);
  server.on("/p1ip", HTTP_POST, handleP1Ip);
  server.on("/inverter/config", HTTP_POST, handleInverterConfig);
  server.on("/settings", HTTP_POST, handleSettings);
  server.on("/settings/test-meter", HTTP_POST, handleTestGridMeter);
  server.on("/inverter/status", HTTP_GET, handleInverterStatus);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/pause", HTTP_POST, handlePause);
  server.on("/resume", HTTP_POST, handleResume);
  server.on("/state", HTTP_GET, handleState);
  server.on("/log",   HTTP_GET,  handleLog);
  server.on("/data",  HTTP_GET,  handleData);
  server.on("/invraw", HTTP_GET, handleInvRaw);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/sdlog", HTTP_GET, handleSdLog);
  server.on("/logging/start", HTTP_POST, handleStartLogging);
  server.on("/update", HTTP_POST, handleOtaComplete, handleOtaUpload);
  server.begin();

  // P1 polling and inverter Modbus TCP polling on core 0, Modbus responder on core 1.
  xTaskCreatePinnedToCore(p1Task,     "p1",     8192, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(invTask,    "inv",    4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(sdLogTask,  "sdlog",  4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(modbusTask, "modbus", 4096, nullptr, 3, nullptr, 1);
  logPrintln("Ready.");
}

void loop() {
  server.handleClient();
  if (otaRestartAtMs && (int32_t)(millis() - otaRestartAtMs) >= 0) {
    delay(100);
    ESP.restart();
  }
  static uint32_t last = 0;
  if (millis() - last > 10000) {
    last = millis();
    xSemaphoreTake(meterMutex, portMAX_DELAY);
    bool v = meter.valid; int fc = meter.failCount;
    xSemaphoreGive(meterMutex);
    logPrintf("[STATUS] meter=%s fails=%d up=%lus ip=%s\n",
              v ? "OK" : "HOLD", fc, millis() / 1000UL,
              WiFi.localIP().toString().c_str());
  }
  delay(1);
}
