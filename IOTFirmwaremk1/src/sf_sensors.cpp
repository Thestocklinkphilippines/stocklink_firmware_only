#include "sf_sensors.h"

#include <time.h>

#include <WiFi.h>

#include "sf_adc.h"
#include "sf_config.h"
#include "sf_debug.h"
#include "keypad_calc.h"
#include "sf_globals.h"
#include "sf_network.h"
#include "sf_pins.h"
#include "sf_storage.h"
#include "sf_utils.h"

namespace {
static const uint8_t kKeyEventQueueSize = 8;
char gKeyEventQueue[kKeyEventQueueSize] = {'\0'};
uint8_t gKeyEventHead = 0;
uint8_t gKeyEventTail = 0;
uint8_t gKeyEventCount = 0;

static const char kKeypadKeys[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'};

KeypadAdcRange gKeypadRanges[16] = {
    {30, 90}, {170, 230}, {325, 385}, {480, 540},
    {645, 705}, {820, 880}, {1000, 1060}, {1185, 1245},
    {1380, 1440}, {1590, 1650}, {1815, 1875}, {2050, 2110},
    {2310, 2370}, {2600, 2660}, {2910, 2970}, {3245, 3305}};

int gKeypadIdleMin = KEYPAD_ADC_NO_KEY_MIN - KEYPAD_IDLE_BAND_ADC;
int gKeypadIdleMax = KEYPAD_ADC_NO_KEY_MIN + KEYPAD_IDLE_BAND_ADC;
bool gKeypadCalibrationLoaded = false;
unsigned long gLastKeypadCalibrationLoadMs = 0;
bool gKeypadInputEnabled = SF_ENABLE_KEYPAD_INPUT != 0;
String gLoadedKeypadCalibrationJson = "";
bool gBatteryReadAttempted = false;
unsigned long gLastBatteryReadMs = 0;
float gLastBatteryVoltageV = -1.0f;

struct FeederLoadCellSample {
  long rawA;
  long rawB;
  unsigned long sampledMs;
};

FeederLoadCellSample gFeedLcSamples[FEED_LC_MAX_AVERAGE_SAMPLES] = {};
uint8_t gFeedLcSampleHead = 0;
uint8_t gFeedLcSampleCount = 0;
bool gFeedLcInitialized = false;
bool gFeedLcHasLatest = false;
long gFeedLcLatestRawA = 0;
long gFeedLcLatestRawB = 0;
unsigned long gFeedLcLatestMs = 0;
bool gFeedLcHasAverage = false;
long gFeedLcAverageRawA = 0;
long gFeedLcAverageRawB = 0;
uint8_t gFeedLcAverageSamples = 0;
unsigned long gFeedLcRateWindowStartedMs = 0;
uint16_t gFeedLcRateWindowSamples = 0;
float gFeedLcObservedSps = 0.0f;

static KeypadAdcTuning buildDefaultKeypadTuning() {
  KeypadAdcTuning tuning;
  tuning.idleBandAdc = KEYPAD_IDLE_BAND_ADC;
  tuning.adcOffset = KEYPAD_ADC_OFFSET;
  tuning.samples = KEYPAD_RUNTIME_SAMPLES;
  tuning.minEventPolls = KEYPAD_EVENT_MIN_POLLS;
  tuning.minMatchedPolls = KEYPAD_EVENT_MIN_MATCHED_POLLS;
  tuning.minConfidencePct = KEYPAD_EVENT_MIN_CONFIDENCE_PCT;
  return tuning;
}

KeypadAdcTuning gKeypadTuning = buildDefaultKeypadTuning();

unsigned long gLastKeyEventMs = 0;
}

static void enqueueKeyEvent(char key) {
  if (key == '\0') return;

  if (gKeyEventCount >= kKeyEventQueueSize) {
    gKeyEventHead = (uint8_t)((gKeyEventHead + 1U) % kKeyEventQueueSize);
    gKeyEventCount--;
  }

  gKeyEventQueue[gKeyEventTail] = key;
  gKeyEventTail = (uint8_t)((gKeyEventTail + 1U) % kKeyEventQueueSize);
  gKeyEventCount++;
}

// Adapter used by the extracted keypad calculator: returns a single fresh ADC reading.
static int keypadAdcReader() {
  return readAdcFast(PIN_KEYPAD_ADC);
}

static long absLong(long value) {
  return value < 0 ? -value : value;
}

static bool requiredFeedLoadCellConstantsPresent() {
  return FEED_LC_A_ZERO_RAW != 0L &&
         FEED_LC_B_ZERO_RAW != 0L &&
         FEED_LC_A_SPAN_RAW != 0L &&
         FEED_LC_B_SPAN_RAW != 0L &&
         FEED_LC_SPAN_KG > 0.0f;
}

static long signExtendHx71124(unsigned long value) {
  if ((value & 0x800000UL) != 0UL) {
    value |= 0xFF000000UL;
  }
  return (long)value;
}

static bool feederLoadCellsReady() {
  if (!gFeedLcInitialized) return false;
  return digitalRead(PIN_FEED_LC_A_DOUT) == LOW &&
         digitalRead(PIN_FEED_LC_B_DOUT) == LOW;
}

static void pulseFeedLoadCellClock() {
  digitalWrite(PIN_FEED_LC_SCK, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_FEED_LC_SCK, LOW);
  delayMicroseconds(1);
}

static bool readFeederLoadCellsRaw(long& rawA, long& rawB) {
  if (!gFeedLcInitialized) initFeederLoadCells();
  if (!feederLoadCellsReady()) return false;

  unsigned long valueA = 0UL;
  unsigned long valueB = 0UL;

  noInterrupts();
  for (uint8_t bit = 0; bit < 24U; ++bit) {
    digitalWrite(PIN_FEED_LC_SCK, HIGH);
    delayMicroseconds(1);
    valueA = (valueA << 1) | (digitalRead(PIN_FEED_LC_A_DOUT) == HIGH ? 1UL : 0UL);
    valueB = (valueB << 1) | (digitalRead(PIN_FEED_LC_B_DOUT) == HIGH ? 1UL : 0UL);
    digitalWrite(PIN_FEED_LC_SCK, LOW);
    delayMicroseconds(1);
  }

  // One extra pulse selects channel A at gain 128 for the next conversion.
  pulseFeedLoadCellClock();
  interrupts();

  rawA = signExtendHx71124(valueA);
  rawB = signExtendHx71124(valueB);
  return true;
}

static void pushFeederLoadCellSample(long rawA, long rawB, unsigned long sampledMs) {
  gFeedLcSamples[gFeedLcSampleHead].rawA = rawA;
  gFeedLcSamples[gFeedLcSampleHead].rawB = rawB;
  gFeedLcSamples[gFeedLcSampleHead].sampledMs = sampledMs;
  gFeedLcSampleHead = (uint8_t)((gFeedLcSampleHead + 1U) % FEED_LC_MAX_AVERAGE_SAMPLES);
  if (gFeedLcSampleCount < FEED_LC_MAX_AVERAGE_SAMPLES) {
    gFeedLcSampleCount++;
  }

  gFeedLcLatestRawA = rawA;
  gFeedLcLatestRawB = rawB;
  gFeedLcLatestMs = sampledMs;
  gFeedLcHasLatest = true;

  if (gFeedLcRateWindowStartedMs == 0UL) {
    gFeedLcRateWindowStartedMs = sampledMs;
    gFeedLcRateWindowSamples = 0U;
  }
  gFeedLcRateWindowSamples++;
  unsigned long elapsedMs = sampledMs - gFeedLcRateWindowStartedMs;
  if (elapsedMs >= FEED_LC_RATE_LOG_INTERVAL_MS) {
    gFeedLcObservedSps = ((float)gFeedLcRateWindowSamples * 1000.0f) / (float)elapsedMs;
    if (gFeedLcObservedSps < (float)FEED_LC_MIN_OBSERVED_SAMPLE_RATE_SPS) {
      LOG_WARN("Feeder HX711 observed %.1f SPS below min %u SPS; set HX711 RATE pin high for 80 SPS",
               gFeedLcObservedSps,
               (unsigned int)FEED_LC_MIN_OBSERVED_SAMPLE_RATE_SPS);
    } else {
      LOG_DEBUG("Feeder HX711 observed %.1f SPS target=%u avg_window_samples=%u",
                gFeedLcObservedSps,
                (unsigned int)FEED_LC_TARGET_SAMPLE_RATE_SPS,
                (unsigned int)gFeedLcAverageSamples);
    }
    gFeedLcRateWindowStartedMs = sampledMs;
    gFeedLcRateWindowSamples = 0U;
  }
}

static bool refreshFeederLoadCellAverage() {
  if (gFeedLcSampleCount == 0U) {
    gFeedLcHasAverage = false;
    gFeedLcAverageSamples = 0U;
    return false;
  }

  unsigned long nowMs = millis();
  long long sumA = 0;
  long long sumB = 0;
  uint8_t count = 0U;

  for (uint8_t i = 0; i < gFeedLcSampleCount; ++i) {
    const FeederLoadCellSample& sample = gFeedLcSamples[i];
    if (nowMs - sample.sampledMs > FEED_LC_AVERAGE_WINDOW_MS) continue;
    sumA += sample.rawA;
    sumB += sample.rawB;
    count++;
  }

  if (count == 0U) {
    gFeedLcHasAverage = false;
    gFeedLcAverageSamples = 0U;
    return false;
  }

  gFeedLcAverageRawA = (long)(sumA / count);
  gFeedLcAverageRawB = (long)(sumB / count);
  gFeedLcAverageSamples = count;
  gFeedLcHasAverage = true;
  return true;
}

static bool getFeederLoadCellCachedKg(float& outKg) {
  if (!isFeederLoadCellCalibrationValid()) return false;
  if (!refreshFeederLoadCellAverage()) return false;
  if (!computeFeederLoadCellKgFromCalibration(gFeedLcAverageRawA, gFeedLcAverageRawB, outKg)) {
    return false;
  }
  if (outKg < 0.0f) outKg = 0.0f;
  return true;
}

static float getFeederMaxCapacityKg(JsonVariant cfg) {
  float maxCap = getConfigOrDefault(cfg, "max_feeds_capacity_kg", DEFAULT_MAX_FEEDS_CAPACITY_KG);
  if (maxCap <= 0.0f) maxCap = DEFAULT_MAX_FEEDS_CAPACITY_KG;
  return maxCap;
}

static float feederKgToPct(float kg, JsonVariant cfg) {
  float maxCap = getFeederMaxCapacityKg(cfg);
  return clampf((kg / maxCap) * 100.0f, 0.0f, 100.0f);
}

static float getStoredFeederLevelPct(JsonVariant cfg) {
  float maxCap = getFeederMaxCapacityKg(cfg);
  float storedKg = readRemainingKg();
  return clampf((storedKg / maxCap) * 100.0f, 0.0f, 100.0f);
}

static bool idleRangeOverlapsKeys(int idleMin, int idleMax, const KeypadAdcRange ranges[16]) {
  for (int i = 0; i < 16; i++) {
    if (idleMin <= ranges[i].maxAdc && ranges[i].minAdc <= idleMax) return true;
  }
  return false;
}

void reloadKeypadCalibration() {
  gKeypadCalibrationLoaded = true;
  gLastKeypadCalibrationLoadMs = millis();

  String cfg = loadLocalConfig();
  DynamicJsonDocument d(8192);
  DeserializationError err = deserializeJson(d, cfg);
  if (err) {
    LOG_KEYPAD_WARN("Keypad input flag reload skipped; config parse error: %s", err.c_str());
  } else {
    JsonVariant root = d.as<JsonVariant>();
    JsonVariant cfgRoot = root;
    if (!root.containsKey("keypad_input_enabled")) {
      JsonVariant nestedCfg = root["config"];
      if (!nestedCfg.isNull()) cfgRoot = nestedCfg;
    }
    if (cfgRoot.isNull()) cfgRoot = root;
    bool enabled = cfgRoot["keypad_input_enabled"] | gKeypadInputEnabled;
    if (enabled != gKeypadInputEnabled) resetKeypadCalcState();
    gKeypadInputEnabled = enabled;
  }

  String calibrationJson = loadLocalKeypadCalibration();
  if (calibrationJson == gLoadedKeypadCalibrationJson) return;

  DynamicJsonDocument calibration(3072);
  DeserializationError calibrationErr = deserializeJson(calibration, calibrationJson);
  if (calibrationErr) {
    LOG_KEYPAD_WARN("Keypad local calibration reload skipped; parse error: %s", calibrationErr.c_str());
    resetKeypadCalcState();
    return;
  }

  int version = calibration["version"] | 0;
  int idleMin = calibration["idle_min"] | -1;
  int idleMax = calibration["idle_max"] | -1;
  JsonArray keys = calibration["keys"].as<JsonArray>();
  if (version != 2 || keys.isNull() || keys.size() != 16U ||
      idleMin < 0 || idleMax > 4095 || idleMin > idleMax) {
    LOG_KEYPAD_WARN("Keypad local calibration reload skipped; invalid range schema");
    resetKeypadCalcState();
    return;
  }

  KeypadAdcRange loaded[16];
  for (int i = 0; i < 16; i++) {
    JsonVariant entry = keys[i];
    const char* storedKey = entry["key"] | "";
    if (storedKey[0] != kKeypadKeys[i] || storedKey[1] != '\0') {
      LOG_KEYPAD_WARN("Keypad local calibration reload skipped; key order mismatch at index=%d", i);
      resetKeypadCalcState();
      return;
    }
    loaded[i].minAdc = entry["min"] | -1;
    loaded[i].maxAdc = entry["max"] | -1;
  }

  if (!hasValidKeypadRanges(loaded) || idleRangeOverlapsKeys(idleMin, idleMax, loaded)) {
    LOG_KEYPAD_WARN("Keypad local calibration reload skipped; ranges overlap or are invalid");
    resetKeypadCalcState();
    return;
  }

  for (int i = 0; i < 16; i++) gKeypadRanges[i] = loaded[i];
  gKeypadIdleMin = idleMin;
  gKeypadIdleMax = idleMax;
  gLoadedKeypadCalibrationJson = calibrationJson;
  resetKeypadCalcState();
  LOG_KEYPAD_INFO("Keypad config reload enabled=%d local_cal_version=%d idle=%d..%d",
                  (int)gKeypadInputEnabled,
                  version,
                  gKeypadIdleMin,
                  gKeypadIdleMax);
}

bool isKeypadInputEnabled() {
  if (!gKeypadCalibrationLoaded || (millis() - gLastKeypadCalibrationLoadMs) > 5000UL) {
    reloadKeypadCalibration();
  }
  return gKeypadInputEnabled;
}

void setKeypadInputEnabled(bool enabled) {
  if (gKeypadInputEnabled != enabled) resetKeypadCalcState();
  gKeypadInputEnabled = enabled;
}

void initFeederLoadCells() {
  if (gFeedLcInitialized) return;
  pinMode(PIN_FEED_LC_SCK, OUTPUT);
  digitalWrite(PIN_FEED_LC_SCK, LOW);
  pinMode(PIN_FEED_LC_A_DOUT, INPUT);
  pinMode(PIN_FEED_LC_B_DOUT, INPUT);
  gFeedLcInitialized = true;
  LOG_INFO("Feeder HX711 pins initialized sck=%d dout_a=%d dout_b=%d",
           PIN_FEED_LC_SCK,
           PIN_FEED_LC_A_DOUT,
           PIN_FEED_LC_B_DOUT);
}

long computeFeederLoadCellSpanDelta(long zeroA, long zeroB, long spanA, long spanB) {
  return (spanA - zeroA) + (spanB - zeroB);
}

bool isFeederLoadCellCalibrationValid() {
  if (!requiredFeedLoadCellConstantsPresent()) return false;
  long spanDelta = computeFeederLoadCellSpanDelta(
      FEED_LC_A_ZERO_RAW,
      FEED_LC_B_ZERO_RAW,
      FEED_LC_A_SPAN_RAW,
      FEED_LC_B_SPAN_RAW);
  return absLong(spanDelta) >= FEED_LC_MIN_SPAN_DELTA_RAW;
}

bool computeFeederLoadCellKgFromCalibration(long rawA, long rawB, float& outKg) {
  if (!isFeederLoadCellCalibrationValid()) return false;

  long spanDelta = computeFeederLoadCellSpanDelta(
      FEED_LC_A_ZERO_RAW,
      FEED_LC_B_ZERO_RAW,
      FEED_LC_A_SPAN_RAW,
      FEED_LC_B_SPAN_RAW);
  long rawDelta = computeFeederLoadCellSpanDelta(
      FEED_LC_A_ZERO_RAW,
      FEED_LC_B_ZERO_RAW,
      rawA,
      rawB);
  float countsPerKg = (float)spanDelta / FEED_LC_SPAN_KG;
  if (countsPerKg == 0.0f) return false;

  outKg = (float)rawDelta / countsPerKg;
  return true;
}

void serviceFeederLoadCells() {
  if (!gFeedLcInitialized) initFeederLoadCells();

  long rawA = 0;
  long rawB = 0;
  if (!readFeederLoadCellsRaw(rawA, rawB)) return;

  pushFeederLoadCellSample(rawA, rawB, millis());
  refreshFeederLoadCellAverage();
}

bool getFeederLoadCellLatestRaw(long& rawA, long& rawB, unsigned long& ageMs) {
  if (!gFeedLcHasLatest) return false;
  rawA = gFeedLcLatestRawA;
  rawB = gFeedLcLatestRawB;
  ageMs = millis() - gFeedLcLatestMs;
  return true;
}

bool getFeederLoadCellAveragedRaw(long& rawA, long& rawB, uint8_t& sampleCount) {
  if (!refreshFeederLoadCellAverage()) return false;
  rawA = gFeedLcAverageRawA;
  rawB = gFeedLcAverageRawB;
  sampleCount = gFeedLcAverageSamples;
  return true;
}

float getFeederLoadCellObservedSampleRateSps() {
  return gFeedLcObservedSps;
}

float getConfigOrDefault(JsonVariant cfg, const char* key, float fallback) {
  if (cfg.isNull() || !cfg.containsKey(key)) return fallback;
  return cfg[key].as<float>();
}

float measureDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) {
    LOG_WARN("Ultrasonic timeout trig=%d echo=%d", trigPin, echoPin);
    return -1.0f;
  }
  return (duration * 0.0343f) / 2.0f;
}

float distanceToLevelPct(float distanceCm, float tankDepthCm) {
  return distanceToLevelPct(distanceCm, 0.0f, tankDepthCm);
}

static float waterHeightCmToLiters(float waterHeightCm, float fullHeightCm) {
  if (fullHeightCm <= 0.0f || WATER_BOTTOM_RADIUS_CM < 0.0f || WATER_FULL_RADIUS_CM < 0.0f) {
    return 0.0f;
  }

  const float boundedHeightCm = clampf(waterHeightCm, 0.0f, fullHeightCm);
  if (boundedHeightCm <= 0.0f) return 0.0f;

  const float radiusAtSurfaceCm = WATER_BOTTOM_RADIUS_CM +
                                  ((WATER_FULL_RADIUS_CM - WATER_BOTTOM_RADIUS_CM) *
                                   (boundedHeightCm / fullHeightCm));
  const float volumeCm3 = (WATER_TANK_PI * boundedHeightCm / 3.0f) *
                          ((WATER_BOTTOM_RADIUS_CM * WATER_BOTTOM_RADIUS_CM) +
                           (WATER_BOTTOM_RADIUS_CM * radiusAtSurfaceCm) +
                           (radiusAtSurfaceCm * radiusAtSurfaceCm));
  return volumeCm3 / 1000.0f;
}

float distanceToLevelPct(float distanceCm, float fullDistanceCm, float emptyDistanceCm) {
  if (distanceCm < 0.0f) return 0.0f;
  float usableSpanCm = emptyDistanceCm - fullDistanceCm;
  if (usableSpanCm <= 0.0f) return 0.0f;

  float boundedDistance = clampf(distanceCm, fullDistanceCm, emptyDistanceCm);
  float waterHeightCm = emptyDistanceCm - boundedDistance;
  float fullLiters = waterHeightCmToLiters(usableSpanCm, usableSpanCm);
  if (fullLiters <= 0.0f) return 0.0f;

  float currentLiters = waterHeightCmToLiters(waterHeightCm, usableSpanCm);
  float levelPct = (currentLiters / fullLiters) * 100.0f;
  return clampf(levelPct, 0.0f, 100.0f);
}

float waterLevelPctToLiters(float waterPct) {
  const float fullHeightCm = WATER_EMPTY_DISTANCE_CM - WATER_FULL_DISTANCE_CM;
  const float fullLiters = waterHeightCmToLiters(fullHeightCm, fullHeightCm);
  if (fullLiters <= 0.0f) return 0.0f;
  const float boundedPct = clampf(waterPct, 0.0f, 100.0f);
  return fullLiters * (boundedPct / 100.0f);
}

static bool measureFeederLevelPct(JsonVariant cfg, float& outPct) {
  float kg = -1.0f;
  if (!getFeederLoadCellCachedKg(kg)) {
    return false;
  }

  float maxCap = getFeederMaxCapacityKg(cfg);
  kg = clampf(kg, 0.0f, maxCap);
  float pct = feederKgToPct(kg, cfg);
  state.lastFeederLevelPct = pct;
  outPct = pct;
  SFMultiConsole.waterPrintf("[DBG ] Feeder load cells avg_a=%ld avg_b=%ld samples=%u kg=%.3f pct=%.1f\n",
                             gFeedLcAverageRawA,
                             gFeedLcAverageRawB,
                             (unsigned int)gFeedLcAverageSamples,
                             kg,
                             pct);
  return true;
}

float getFeederLevelPct(JsonVariant cfg) {
  float pct = 0.0f;
  if (!measureFeederLevelPct(cfg, pct)) {
    pct = getStoredFeederLevelPct(cfg);
    state.lastFeederLevelPct = pct;
    return pct;
  }
  return pct;
}

float getFeederRemainingKg(JsonVariant cfg) {
  float kg = -1.0f;
  if (!getFeederLoadCellCachedKg(kg)) return -1.0f;

  float maxCap = getFeederMaxCapacityKg(cfg);
  kg = clampf(kg, 0.0f, maxCap);
  state.lastFeederLevelPct = feederKgToPct(kg, cfg);
  return kg;
}

static bool parseScheduleTimeMinutes(const char* timeStr, int& outMinutes) {
  if (timeStr == nullptr || strlen(timeStr) < 5 || timeStr[2] != ':') return false;
  if (timeStr[0] < '0' || timeStr[0] > '9' ||
      timeStr[1] < '0' || timeStr[1] > '9' ||
      timeStr[3] < '0' || timeStr[3] > '9' ||
      timeStr[4] < '0' || timeStr[4] > '9') {
    return false;
  }

  int hour = ((timeStr[0] - '0') * 10) + (timeStr[1] - '0');
  int minute = ((timeStr[3] - '0') * 10) + (timeStr[4] - '0');
  if (hour < 0 || hour >= 24 || minute < 0 || minute >= 60) return false;

  outMinutes = (hour * 60) + minute;
  return true;
}

static bool scheduleDaysContain(JsonArrayConst days, const char* dayName) {
  if (dayName == nullptr) return false;
  for (JsonVariantConst day : days) {
    const char* candidate = day | "";
    if (strcmp(candidate, dayName) == 0) return true;
  }
  return false;
}

static float computeRequiredNextFeedKg(JsonVariant cfg) {
  if (cfg.isNull() || !cfg.containsKey("schedules")) return 0.0f;

  JsonArray schedules = cfg["schedules"].as<JsonArray>();
  if (schedules.isNull() || schedules.size() == 0) return 0.0f;

  static const char* kWeekdays[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  const int kMinPerDay = 24 * 60;
  int bestOffsetMin = 8 * kMinPerDay;
  float bestAmountKg = 0.0f;

  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  int nowMin = (timeinfo.tm_hour * 60) + timeinfo.tm_min;
  int nowWday = timeinfo.tm_wday;

  for (JsonVariant schedule : schedules) {
    if (!(schedule["enabled"] | false)) continue;

    float amountKg = schedule["feeding_amount_kg"] | 0.0f;
    if (amountKg <= 0.0f) continue;

    int scheduleMin = 0;
    if (!parseScheduleTimeMinutes(schedule["time"] | "", scheduleMin)) continue;

    JsonArrayConst days = schedule["days"].as<JsonArrayConst>();
    bool hasDays = !days.isNull() && days.size() > 0;

    if (!hasDays) {
      int offset = scheduleMin - nowMin;
      if (offset < 0) offset += kMinPerDay;
      if (offset < bestOffsetMin) {
        bestOffsetMin = offset;
        bestAmountKg = amountKg;
      }
      continue;
    }

    for (int dayOffset = 0; dayOffset < 7; ++dayOffset) {
      int weekdayIdx = (nowWday + dayOffset) % 7;
      if (!scheduleDaysContain(days, kWeekdays[weekdayIdx])) continue;

      int offset = (dayOffset * kMinPerDay) + (scheduleMin - nowMin);
      if (offset < 0) continue;

      if (offset < bestOffsetMin) {
        bestOffsetMin = offset;
        bestAmountKg = amountKg;
      }
      break;
    }
  }

  return bestAmountKg;
}

float getWaterLevelPct(JsonVariant cfg) {
  (void)cfg;
  float fullDistance = WATER_FULL_DISTANCE_CM;
  float emptyDistance = WATER_EMPTY_DISTANCE_CM;
  float d = measureDistanceCm(PIN_WATER_TRIG, PIN_WATER_ECHO);
  if (d < MIN_VALID_DISTANCE_CM || d > MAX_VALID_DISTANCE_CM) {
    LOG_WARN("Water ultrasonic out-of-range %.2fcm", d);
    state.lastWaterLevelPct = 0.0f;
    return 0.0f;
  }
  float usableSpanCm = emptyDistance - fullDistance;
  float boundedDistance = clampf(d, fullDistance, emptyDistance);
  float heightPct = usableSpanCm > 0.0f ? ((emptyDistance - boundedDistance) / usableSpanCm) * 100.0f : 0.0f;
  float pct = distanceToLevelPct(d, fullDistance, emptyDistance);
  float liters = waterLevelPctToLiters(pct);
  state.lastWaterLevelPct = pct;
  SFMultiConsole.waterPrintf("[DBG ] Water ultrasonic dist=%.2fcm full=%.2fcm empty=%.2fcm height_pct=%.1f volume_pct=%.1f liters=%.3f\n",
                             d,
                             fullDistance,
                             emptyDistance,
                             clampf(heightPct, 0.0f, 100.0f),
                             pct,
                             liters);
  return pct;
}

float getBatteryVoltageV(JsonVariant cfg) {
  bool enabled = cfg.isNull() ? true : (cfg["battery_sense_enabled"] | true);
  if (!enabled) {
    gBatteryReadAttempted = false;
    gLastBatteryVoltageV = -1.0f;
    return -1.0f;
  }

  unsigned long nowMs = millis();
  if (gBatteryReadAttempted && nowMs - gLastBatteryReadMs < BATTERY_READ_INTERVAL_MS) {
    return gLastBatteryVoltageV;
  }
  gBatteryReadAttempted = true;
  gLastBatteryReadMs = nowMs;

  int adcPin = BATTERY_ADC_PIN;
  float dividerTop = BATTERY_DIVIDER_TOP_OHMS;
  float dividerBottom = BATTERY_DIVIDER_BOTTOM_OHMS;
  float adcRefV = BATTERY_ADC_REFERENCE_V;
  float gainCorrection = BATTERY_ADC_GAIN_CORRECTION;

  if (dividerTop <= 0.0f || dividerBottom <= 0.0f || adcRefV <= 0.0f || gainCorrection <= 0.0f) {
    gLastBatteryVoltageV = -1.0f;
    LOG_WARN("Battery sense config invalid top=%.1f bottom=%.1f ref=%.2f gain=%.3f",
             dividerTop,
             dividerBottom,
             adcRefV,
             gainCorrection);
    return -1.0f;
  }

  int raw = readAdcReliable(adcPin, ADC_PRIORITY_NORMAL);
  if (raw < 0) {
    gLastBatteryVoltageV = -1.0f;
    LOG_WARN("Battery ADC read failed pin=%d", adcPin);
    return -1.0f;
  }

  float adcVoltage = ((float)raw / 4095.0f) * adcRefV;
  float batteryVoltage = adcVoltage * ((dividerTop + dividerBottom) / dividerBottom) * gainCorrection;
  gLastBatteryVoltageV = batteryVoltage;
  LOG_DEBUG("Battery sense pin=%d raw=%d adcV=%.3f battV=%.3f gain=%.3f",
            adcPin,
            raw,
            adcVoltage,
            batteryVoltage,
            gainCorrection);
  return batteryVoltage;
}

char decodeKeypadAnalog(int adc) {
  if (!gKeypadCalibrationLoaded) reloadKeypadCalibration();

  int adjustedAdc = adc + gKeypadTuning.adcOffset;
  if (adjustedAdc < 0) adjustedAdc = 0;
  if (adjustedAdc > 4095) adjustedAdc = 4095;

  int idleMin = gKeypadIdleMin - gKeypadTuning.idleBandAdc;
  int idleMax = gKeypadIdleMax + gKeypadTuning.idleBandAdc;
  if (adjustedAdc >= idleMin && adjustedAdc <= idleMax) return '\0';
  return decodeKeypadRangeSample(adjustedAdc, gKeypadRanges);
}

void pollKeypad() {
  if (!gKeypadInputEnabled) return;

  if ((millis() - gLastKeypadCalibrationLoadMs) > 5000UL) {
    reloadKeypadCalibration();
  }

  KeypadEventStats eventStats;
  char candidate = calculateKeypadKey(keypadAdcReader,
                                      gKeypadRanges,
                                      gKeypadIdleMin,
                                      gKeypadIdleMax,
                                      gKeypadTuning,
                                      &eventStats);

  char leader = eventStats.winningKeyIndex >= 0 ? kKeypadKeys[eventStats.winningKeyIndex] : '-';
  char result = candidate == '\0' ? '-' : candidate;
  LOG_KEYPAD_INFO("poll raw=%d adjusted=%d active=%d event_done=%d polls=%u matched=%u leader=%c votes=%u confidence=%u result=%c",
                  eventStats.pollRawAdc,
                  eventStats.pollAdjustedAdc,
                  eventStats.pressActive ? 1 : 0,
                  eventStats.eventCompleted ? 1 : 0,
                  (unsigned int)eventStats.eventPolls,
                  (unsigned int)eventStats.matchedPolls,
                  leader,
                  (unsigned int)eventStats.winningVotes,
                  (unsigned int)eventStats.confidencePct,
                  result);

  if (!eventStats.eventCompleted) return;
  if (candidate == '\0') {
    LOG_KEYPAD_WARN("Keypad event rejected range=%d..%d polls=%u matched=%u leader=%c confidence=%u",
                    eventStats.eventMinAdc,
                    eventStats.eventMaxAdc,
                    (unsigned int)eventStats.eventPolls,
                    (unsigned int)eventStats.matchedPolls,
                    leader,
                    (unsigned int)eventStats.confidencePct);
    return;
  }

  SFMultiConsole.writeKeyCandidate(candidate);
  unsigned long nowMs = millis();
  if ((nowMs - gLastKeyEventMs) < KEYPAD_EVENT_DEBOUNCE_MS) return;
  gLastKeyEventMs = nowMs;

  LOG_KEYPAD_INFO("Keypad key=%c event_range=%d..%d polls=%u matched=%u votes=%u confidence=%u raw_release=%d",
                  candidate,
                  eventStats.eventMinAdc,
                  eventStats.eventMaxAdc,
                  (unsigned int)eventStats.eventPolls,
                  (unsigned int)eventStats.matchedPolls,
                  (unsigned int)eventStats.winningVotes,
                  (unsigned int)eventStats.confidencePct,
                  eventStats.pollRawAdc);
  enqueueKeyEvent(candidate);
  if (candidate == 'A' && SF_SEND_KEYPAD_LOGS) {
    StaticJsonDocument<64> p;
    p["source"] = "keypad";
    p["key"] = "A";
    sendLog("ui", p.as<JsonVariant>());
  }
}

char consumeKeypadKeyEvent() {
  if (gKeyEventCount == 0) return '\0';
  char out = gKeyEventQueue[gKeyEventHead];
  gKeyEventHead = (uint8_t)((gKeyEventHead + 1U) % kKeyEventQueueSize);
  gKeyEventCount--;
  return out;
}

bool readMainsPowerPresent() {
  int raw = digitalRead(PIN_MAINS_SENSE_ADC);
  bool mainsLoss = MAINS_LOSS_SIGNAL_ACTIVE_HIGH ? (raw == HIGH) : (raw == LOW);
  bool present = !mainsLoss;
  LOG_DEBUG("Mains digital=%d loss=%d present=%d", raw, mainsLoss ? 1 : 0, present ? 1 : 0);
  return present;
}

static bool loadPersistedMainsPowerPresent(bool& present) {
  if (!prefs.begin(PREF_NAMESPACE, true)) {
    LOG_ERROR("Preferences begin(read mains state) failed");
    return false;
  }

  bool hasKey = prefs.isKey(PREF_MAINS_POWER_PRESENT);
  present = prefs.getBool(PREF_MAINS_POWER_PRESENT, false);
  prefs.end();
  return hasKey;
}

static void savePersistedMainsPowerPresent(bool present) {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(write mains state) failed");
    return;
  }

  prefs.putBool(PREF_MAINS_POWER_PRESENT, present);
  prefs.end();
}

void reconcilePowerAlertStateOnBoot() {
  bool currentPresent = readMainsPowerPresent();
  bool persistedPresent = currentPresent;
  bool hadPersistedState = loadPersistedMainsPowerPresent(persistedPresent);

  state.mainsPowerPresent = currentPresent;
  state.pendingPowerOutageAlert = false;
  state.pendingPowerRestoredAlert = false;

  if (!hadPersistedState) {
    savePersistedMainsPowerPresent(currentPresent);
    LOG_INFO("Boot power state initialized present=%d", currentPresent ? 1 : 0);
    return;
  }

  if (persistedPresent == currentPresent) {
    LOG_INFO("Boot power state unchanged present=%d", currentPresent ? 1 : 0);
    return;
  }

  savePersistedMainsPowerPresent(currentPresent);

  if (currentPresent) {
    LOG_INFO("Boot detected mains restored after outage");
    sendAlert("power_restored");
    StaticJsonDocument<128> p;
    p["event"] = "mains_restored";
    sendLog("power", p.as<JsonVariant>());
  } else {
    LOG_WARN("Boot detected mains outage while device restarted on battery");
    sendAlert("power_outage");
  }
}

static void trySendPendingPowerAlerts() {
  if (!WiFi.isConnected()) return;

  if (state.pendingPowerOutageAlert) {
    sendAlert("power_outage");
    state.pendingPowerOutageAlert = false;
  }

  if (state.pendingPowerRestoredAlert) {
    sendAlert("power_restored");
    state.pendingPowerRestoredAlert = false;
  }
}

void handlePowerFailMonitoring() {
  bool nowPresent = readMainsPowerPresent();
  trySendPendingPowerAlerts();

  if (nowPresent != state.mainsPowerPresent) {
    state.mainsPowerPresent = nowPresent;
    savePersistedMainsPowerPresent(nowPresent);
    if (!nowPresent) {
      LOG_WARN("Power outage detected (UPS active)");
      tone(PIN_BUZZER, 2500, 250);
      state.pendingPowerOutageAlert = true;
      state.pendingPowerRestoredAlert = false;
      trySendPendingPowerAlerts();
    } else {
      LOG_INFO("Mains power restored");
      state.pendingPowerOutageAlert = false;
      state.pendingPowerRestoredAlert = true;
      trySendPendingPowerAlerts();
      StaticJsonDocument<128> p;
      p["event"] = "mains_restored";
      sendLog("power", p.as<JsonVariant>());
    }
  }
}

void reportSensorLevels(JsonVariant cfg) {
  float feederLevel = getFeederLevelPct(cfg);
  float waterLevel = getWaterLevelPct(cfg);
  float waterLiters = waterLevelPctToLiters(waterLevel);
  float batteryVoltage = getBatteryVoltageV(cfg);
  float liveFeedKg = getFeederRemainingKg(cfg);
  float feedCurrentKg = liveFeedKg >= 0.0f ? liveFeedKg : readRemainingKg();
  float maxCap = getConfigOrDefault(cfg, "max_feeds_capacity_kg", DEFAULT_MAX_FEEDS_CAPACITY_KG);
  if (maxCap <= 0.0f) maxCap = DEFAULT_MAX_FEEDS_CAPACITY_KG;
  feedCurrentKg = clampf(feedCurrentKg, 0.0f, maxCap);
  float feedRequiredNextKg = computeRequiredNextFeedKg(cfg);
  bool feedSufficient = feedRequiredNextKg <= 0.0f || feedCurrentKg >= feedRequiredNextKg;

  StaticJsonDocument<640> doc;
  doc["feeder_level_pct"] = feederLevel;
  doc["water_level_pct"] = waterLevel;
  doc["water_current_liters"] = waterLiters;
  if (batteryVoltage >= 0.0f) {
    doc["battery_voltage_v"] = batteryVoltage;
  }
  doc["feed_sufficient"] = feedSufficient;
  doc["feed_current_kg"] = feedCurrentKg;
  doc["feed_required_next_kg"] = feedRequiredNextKg;
  doc["mains_power_present"] = state.mainsPowerPresent;
  doc["timestamp"] = getUtcIsoNow();

  String body;
  serializeJson(doc, body);
  bool accepted = sendSensorState(body);
  LOG_INFO("Sensor report queued=%d feeder=%.1f water=%.1f waterL=%.3f batt=%.2f feedKg=%.3f requiredNext=%.3f sufficient=%d",
           accepted ? 1 : 0,
           feederLevel,
           waterLevel,
           waterLiters,
           batteryVoltage,
           feedCurrentKg,
           feedRequiredNextKg,
           feedSufficient ? 1 : 0);
}
