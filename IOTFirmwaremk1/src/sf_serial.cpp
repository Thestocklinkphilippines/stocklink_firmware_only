#include "sf_serial.h"

#include <WiFi.h>
#include <stdlib.h>

#include "keypad_calc.h"
#include "sf_config.h"
#include "sf_actuators.h"
#include "sf_debug.h"
#include "sf_globals.h"
#include "sf_network.h"
#include "sf_pins.h"
#include "sf_sensors.h"
#include "sf_shutdown.h"
#include "sf_storage.h"
#include "sf_utils.h"

// Keep large JSON buffers off the loopTask stack.
static DynamicJsonDocument gSerialCfgDoc(8192);

namespace {

static const char kCalKeys[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'};

struct KeypadCalibrationSample {
  int minAdc;
  int maxAdc;
  int averageAdc;
  int observedMinAdc;
  int observedMaxAdc;
};

struct KeypadCalibrationSession {
  bool active;
  int step;
  int trend;
  KeypadAdcRange ranges[16];
  int idleMinAdc;
  int idleMaxAdc;
};

struct LoadCellCalibrationSession {
  bool active;
  bool zeroCaptured;
  bool spanCaptured;
  bool knownMassSet;
  float knownKg;
  long zeroA;
  long zeroB;
  long spanA;
  long spanB;
  unsigned long lastReportMs;
  char reason[48];
};

KeypadCalibrationSession gKeypadCal = {};
LoadCellCalibrationSession gLoadCellCal = {};

void keypadConsolePrint(const char* text) {
  if (text == nullptr) return;
  Serial.println(text);
  SFMultiConsole.keypadPrintf("%s\n", text);
}

template <typename... Args>
void keypadConsolePrintf(const char* format, Args... args) {
  char buffer[256];
  int len = snprintf(buffer, sizeof(buffer), format, args...);
  if (len < 0) return;
  Serial.print(buffer);
  SFMultiConsole.keypadPrintf("%s", buffer);
}

void loadCellConsolePrint(const char* text) {
  if (text == nullptr) return;
  Serial.println(text);
}

template <typename... Args>
void loadCellConsolePrintf(const char* format, Args... args) {
  char buffer[384];
  int len = snprintf(buffer, sizeof(buffer), format, args...);
  if (len < 0) return;
  Serial.print(buffer);
}

bool tryParseFloat(const String& token, float& outValue) {
  if (token.length() == 0) return false;
  char buf[32];
  token.toCharArray(buf, sizeof(buf));
  char* endPtr = nullptr;
  float v = strtof(buf, &endPtr);
  if (endPtr == buf || *endPtr != '\0') return false;
  outValue = v;
  return true;
}

bool tryParseInt(const String& token, int& outValue) {
  if (token.length() == 0) return false;
  char buf[16];
  token.toCharArray(buf, sizeof(buf));
  char* endPtr = nullptr;
  long v = strtol(buf, &endPtr, 10);
  if (endPtr == buf || *endPtr != '\0') return false;
  outValue = (int)v;
  return true;
}

static int clampKeypadAdc(int value) {
  if (value < 0) return 0;
  if (value > 4095) return 4095;
  return value;
}

static int sampleKeypadRuntimeWindow() {
  const int sampleCount = KEYPAD_RUNTIME_SAMPLES > 0 ? KEYPAD_RUNTIME_SAMPLES : 1;
  long total = 0;
  int minSample = 4095;
  int maxSample = 0;
  for (int i = 0; i < sampleCount; i++) {
    int sample = clampKeypadAdc(analogRead(PIN_KEYPAD_ADC));
    total += sample;
    if (sample < minSample) minSample = sample;
    if (sample > maxSample) maxSample = sample;
    delayMicroseconds(500);
  }
  if (sampleCount >= 3) {
    total -= minSample;
    total -= maxSample;
    return clampKeypadAdc((int)(total / (sampleCount - 2)) + KEYPAD_ADC_OFFSET);
  }
  return clampKeypadAdc((int)(total / sampleCount) + KEYPAD_ADC_OFFSET);
}

static KeypadCalibrationSample sampleKeypadCalibrationRange() {
  int values[KEYPAD_CALIBRATION_WINDOWS];
  long total = 0;
  int observedMin = 4095;
  int observedMax = 0;

  for (int i = 0; i < KEYPAD_CALIBRATION_WINDOWS; i++) {
    int value = sampleKeypadRuntimeWindow();
    values[i] = value;
    total += value;
    if (value < observedMin) observedMin = value;
    if (value > observedMax) observedMax = value;
    delay(KEYPAD_CALIBRATION_WINDOW_SPACING_MS);
  }

  for (int i = 1; i < KEYPAD_CALIBRATION_WINDOWS; i++) {
    int value = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = value;
  }

  int trimCount = (KEYPAD_CALIBRATION_WINDOWS * KEYPAD_CALIBRATION_TRIM_PERCENT) / 100;
  if (trimCount < 0) trimCount = 0;
  if (trimCount * 2 >= KEYPAD_CALIBRATION_WINDOWS) trimCount = 0;
  KeypadCalibrationSample sample;
  sample.minAdc = clampKeypadAdc(values[trimCount] - KEYPAD_CALIBRATION_RANGE_PADDING_ADC);
  sample.maxAdc = clampKeypadAdc(values[KEYPAD_CALIBRATION_WINDOWS - 1 - trimCount] +
                                 KEYPAD_CALIBRATION_RANGE_PADDING_ADC);
  sample.averageAdc = (int)(total / KEYPAD_CALIBRATION_WINDOWS);
  sample.observedMinAdc = observedMin;
  sample.observedMaxAdc = observedMax;
  return sample;
}

static bool rangesOverlap(const KeypadAdcRange& a, const KeypadAdcRange& b) {
  return a.minAdc <= b.maxAdc && b.minAdc <= a.maxAdc;
}

static int rangeCenter(const KeypadAdcRange& range) {
  return (range.minAdc + range.maxAdc) / 2;
}

static bool idleRangeOverlapsCalibration(int idleMin, int idleMax) {
  KeypadAdcRange idle = {idleMin, idleMax};
  for (int i = 0; i < 16; i++) {
    if (rangesOverlap(idle, gKeypadCal.ranges[i])) return true;
  }
  return false;
}

void printCalibrationPrompt() {
  if (!gKeypadCal.active) return;

  if (gKeypadCal.step < 16) {
    Serial.println();
    SFMultiConsole.keypadPrintf("\n");
    keypadConsolePrintf("[KEYPAD CAL] Step %d/16\n", gKeypadCal.step + 1);
    keypadConsolePrintf("[KEYPAD CAL] Press and hold key '%c' steadily, then type: sample\n", kCalKeys[gKeypadCal.step]);
    keypadConsolePrint("[KEYPAD CAL] The device will record about one second of runtime-equivalent ADC windows.");
    keypadConsolePrint("[KEYPAD CAL] Type 'cancel' anytime to abort.");
    return;
  }

  Serial.println();
  SFMultiConsole.keypadPrintf("\n");
  keypadConsolePrint("[KEYPAD CAL] Final step");
  keypadConsolePrint("[KEYPAD CAL] Release all keys, then type: sample");
}

void endCalibrationSession(bool success) {
  gKeypadCal.active = false;
  gSerialConsoleExclusive = false;
  if (!success) {
    keypadConsolePrint("[KEYPAD CAL] Calibration cancelled.");
  }
}

bool persistKeypadCalibration() {
  DynamicJsonDocument calibration(3072);
  calibration["version"] = 2;
  calibration["calibrated_at"] = getUtcIsoNow();
  calibration["source"] = "interactive_range_calibration";
  calibration["idle_min"] = gKeypadCal.idleMinAdc;
  calibration["idle_max"] = gKeypadCal.idleMaxAdc;
  JsonArray keys = calibration.createNestedArray("keys");
  for (int i = 0; i < 16; i++) {
    JsonObject entry = keys.createNestedObject();
    char keyName[2] = {kCalKeys[i], '\0'};
    entry["key"] = keyName;
    entry["min"] = gKeypadCal.ranges[i].minAdc;
    entry["max"] = gKeypadCal.ranges[i].maxAdc;
  }

  String out;
  serializeJson(calibration, out);
  if (!saveLocalKeypadCalibration(out)) return false;
  reloadKeypadCalibration();
  return true;
}

void processKeypadCalibrationLine(const String& rawLine) {
  String line = rawLine;
  line.trim();
  line.toLowerCase();

  if (line == "") return;

  if (line == "cancel") {
    endCalibrationSession(false);
    return;
  }

  if (line == "help") {
    keypadConsolePrint("[KEYPAD CAL] Commands: sample | cancel");
    printCalibrationPrompt();
    return;
  }

  if (line != "sample") {
    keypadConsolePrint("[KEYPAD CAL] Unknown command during calibration. Use: sample or cancel");
    printCalibrationPrompt();
    return;
  }

  keypadConsolePrint("[KEYPAD CAL] Sampling...");
  KeypadCalibrationSample sample = sampleKeypadCalibrationRange();

  if (gKeypadCal.step < 16) {
    if (sample.minAdc < 0 || sample.maxAdc > 4095 || sample.minAdc > sample.maxAdc) {
      keypadConsolePrintf("[KEYPAD CAL] Range=%d..%d is invalid for key '%c'. Retry step %d.\n",
                          sample.minAdc,
                          sample.maxAdc,
                          kCalKeys[gKeypadCal.step],
                          gKeypadCal.step + 1);
      return;
    }

    if (gKeypadCal.step > 0) {
      KeypadAdcRange previous = gKeypadCal.ranges[gKeypadCal.step - 1];
      KeypadAdcRange current = {sample.minAdc, sample.maxAdc};
      int previousCenter = rangeCenter(previous);
      int currentCenter = rangeCenter(current);
      int delta = currentCenter - previousCenter;

      if ((delta >= -15 && delta <= 15) || rangesOverlap(previous, current)) {
        keypadConsolePrintf("[KEYPAD CAL] Range=%d..%d is too close to/overlaps previous=%d..%d. Retry step %d.\n",
                            current.minAdc,
                            current.maxAdc,
                            previous.minAdc,
                            previous.maxAdc,
                            gKeypadCal.step + 1);
        return;
      }

      int stepTrend = (delta > 0) ? 1 : -1;
      if (gKeypadCal.trend == 0) {
        gKeypadCal.trend = stepTrend;
      } else if (stepTrend != gKeypadCal.trend) {
        const char* expected = (gKeypadCal.trend > 0) ? "higher" : "lower";
        keypadConsolePrintf("[KEYPAD CAL] Range=%d..%d has wrong direction vs previous=%d..%d (expected %s). Retry step %d.\n",
                            current.minAdc,
                            current.maxAdc,
                            previous.minAdc,
                            previous.maxAdc,
                            expected,
                            gKeypadCal.step + 1);
        return;
      }
    }

    gKeypadCal.ranges[gKeypadCal.step].minAdc = sample.minAdc;
    gKeypadCal.ranges[gKeypadCal.step].maxAdc = sample.maxAdc;
    keypadConsolePrintf("[KEYPAD CAL] Captured key '%c' -> stable=%d..%d average=%d observed=%d..%d windows=%d\n",
                        kCalKeys[gKeypadCal.step],
                        sample.minAdc,
                        sample.maxAdc,
                        sample.averageAdc,
                        sample.observedMinAdc,
                        sample.observedMaxAdc,
                        KEYPAD_CALIBRATION_WINDOWS);
    gKeypadCal.step++;
    printCalibrationPrompt();
    return;
  }

  if (sample.minAdc < 0 || sample.maxAdc > 4095 || sample.minAdc > sample.maxAdc) {
    keypadConsolePrintf("[KEYPAD CAL] Idle range=%d..%d is invalid. Release all keys and retry.\n",
                        sample.minAdc,
                        sample.maxAdc);
    return;
  }

  gKeypadCal.idleMinAdc = sample.minAdc;
  gKeypadCal.idleMaxAdc = sample.maxAdc;
  if (!hasValidKeypadRanges(gKeypadCal.ranges) ||
      idleRangeOverlapsCalibration(gKeypadCal.idleMinAdc, gKeypadCal.idleMaxAdc)) {
    keypadConsolePrintf("[KEYPAD CAL] Idle range=%d..%d overlaps a key range. Release all keys and retry.\n",
                        gKeypadCal.idleMinAdc,
                        gKeypadCal.idleMaxAdc);
    return;
  }

  if (!persistKeypadCalibration()) {
    keypadConsolePrint("[KEYPAD CAL] Failed to save calibration.");
    endCalibrationSession(false);
    return;
  }

  keypadConsolePrintf("[KEYPAD CAL] Saved locally. idle=%d..%d trend=%s\n",
                      gKeypadCal.idleMinAdc,
                      gKeypadCal.idleMaxAdc,
                      gKeypadCal.trend > 0 ? "ascending" : "descending");
  keypadConsolePrint("[KEYPAD CAL] Calibration complete. Returning to normal operation.");
  endCalibrationSession(true);
}

void startKeypadCalibrationSession() {
  stopActuatorsForSafety("keypad_calibration");
  gKeypadCal.active = true;
  gKeypadCal.step = 0;
  gKeypadCal.trend = 0;
  gKeypadCal.idleMinAdc = 0;
  gKeypadCal.idleMaxAdc = 0;
  for (int i = 0; i < 16; i++) {
    gKeypadCal.ranges[i].minAdc = 0;
    gKeypadCal.ranges[i].maxAdc = 0;
  }

  gSerialConsoleExclusive = true;
  Serial.println();
  SFMultiConsole.keypadPrintf("\n");
  keypadConsolePrint("[KEYPAD CAL] Serial output is now in exclusive calibration mode.");
  keypadConsolePrint("[KEYPAD CAL] Normal logs are temporarily muted.");
  keypadConsolePrint("[KEYPAD CAL] Commands during calibration: sample | cancel");
  printCalibrationPrompt();
}

static long absCalLong(long value) {
  return value < 0 ? -value : value;
}

static bool captureLoadCellAverage(long& rawA, long& rawB, uint8_t& samples) {
  serviceFeederLoadCells();
  if (getFeederLoadCellAveragedRaw(rawA, rawB, samples)) return true;
  loadCellConsolePrint("[LOADCELL CAL] No recent HX711 average yet. Check wiring, wait for DOUT ready, then retry.");
  return false;
}

static bool computeSessionKg(long rawA, long rawB, float& outKg) {
  if (!gLoadCellCal.zeroCaptured || !gLoadCellCal.spanCaptured || !gLoadCellCal.knownMassSet) {
    return false;
  }
  long spanDelta = computeFeederLoadCellSpanDelta(
      gLoadCellCal.zeroA,
      gLoadCellCal.zeroB,
      gLoadCellCal.spanA,
      gLoadCellCal.spanB);
  if (absCalLong(spanDelta) < FEED_LC_MIN_SPAN_DELTA_RAW || gLoadCellCal.knownKg <= 0.0f) {
    return false;
  }
  long rawDelta = computeFeederLoadCellSpanDelta(
      gLoadCellCal.zeroA,
      gLoadCellCal.zeroB,
      rawA,
      rawB);
  float countsPerKg = (float)spanDelta / gLoadCellCal.knownKg;
  if (countsPerKg == 0.0f) return false;
  outKg = (float)rawDelta / countsPerKg;
  return true;
}

static void printLoadCellCommands() {
  loadCellConsolePrint("[LOADCELL CAL] Commands: zero | mass <kg> | span | status | cancel");
}

static void printLoadCellStatus(bool verbose) {
  long latestA = 0;
  long latestB = 0;
  unsigned long ageMs = 0;
  long avgA = 0;
  long avgB = 0;
  uint8_t samples = 0U;
  bool hasLatest = getFeederLoadCellLatestRaw(latestA, latestB, ageMs);
  bool hasAverage = getFeederLoadCellAveragedRaw(avgA, avgB, samples);

  loadCellConsolePrintf("[LOADCELL CAL] hardcoded_valid=%d stored_remaining_kg=%.3f known_kg=%.3f zero=%d span=%d\n",
                        isFeederLoadCellCalibrationValid() ? 1 : 0,
                        readRemainingKg(),
                        gLoadCellCal.knownMassSet ? gLoadCellCal.knownKg : 0.0f,
                        gLoadCellCal.zeroCaptured ? 1 : 0,
                        gLoadCellCal.spanCaptured ? 1 : 0);
  loadCellConsolePrintf("[LOADCELL CAL] target_sps=%u min_sps=%u observed_sps=%.1f avg_window_ms=%lu\n",
                        (unsigned int)FEED_LC_TARGET_SAMPLE_RATE_SPS,
                        (unsigned int)FEED_LC_MIN_OBSERVED_SAMPLE_RATE_SPS,
                        getFeederLoadCellObservedSampleRateSps(),
                        FEED_LC_AVERAGE_WINDOW_MS);

  if (hasLatest) {
    loadCellConsolePrintf("[LOADCELL CAL] latest raw_a=%ld raw_b=%ld age_ms=%lu\n",
                          latestA,
                          latestB,
                          ageMs);
  } else {
    loadCellConsolePrint("[LOADCELL CAL] latest raw=(none yet)");
  }

  if (!hasAverage) {
    loadCellConsolePrint("[LOADCELL CAL] avg raw=(waiting for samples)");
    if (verbose) printLoadCellCommands();
    return;
  }

  long sessionDelta = gLoadCellCal.zeroCaptured
                          ? computeFeederLoadCellSpanDelta(gLoadCellCal.zeroA, gLoadCellCal.zeroB, avgA, avgB)
                          : 0L;
  float hardcodedKg = -1.0f;
  bool hardcodedKgValid = computeFeederLoadCellKgFromCalibration(avgA, avgB, hardcodedKg);
  float sessionKg = -1.0f;
  bool sessionKgValid = computeSessionKg(avgA, avgB, sessionKg);

  loadCellConsolePrintf("[LOADCELL CAL] avg raw_a=%ld raw_b=%ld samples=%u delta_from_zero=%ld",
                        avgA,
                        avgB,
                        (unsigned int)samples,
                        sessionDelta);
  if (hardcodedKgValid) {
    loadCellConsolePrintf(" hardcoded_kg=%.3f", hardcodedKg);
  }
  if (sessionKgValid) {
    loadCellConsolePrintf(" session_kg=%.3f", sessionKg);
  }
  loadCellConsolePrint("");

  if (verbose) printLoadCellCommands();
}

static bool loadCellSessionConstantsNonZero() {
  return gLoadCellCal.zeroA != 0L &&
         gLoadCellCal.zeroB != 0L &&
         gLoadCellCal.spanA != 0L &&
         gLoadCellCal.spanB != 0L;
}

static void printLoadCellCopyPasteConstants() {
  if (!gLoadCellCal.zeroCaptured) {
    loadCellConsolePrint("[LOADCELL CAL] Capture empty tank first: zero");
    return;
  }
  if (!gLoadCellCal.knownMassSet || gLoadCellCal.knownKg <= 0.0f) {
    loadCellConsolePrint("[LOADCELL CAL] Set known feed mass first: mass <kg>");
    return;
  }

  uint8_t samples = 0U;
  if (!captureLoadCellAverage(gLoadCellCal.spanA, gLoadCellCal.spanB, samples)) return;
  gLoadCellCal.spanCaptured = true;

  long spanDelta = computeFeederLoadCellSpanDelta(
      gLoadCellCal.zeroA,
      gLoadCellCal.zeroB,
      gLoadCellCal.spanA,
      gLoadCellCal.spanB);

  if (!loadCellSessionConstantsNonZero() || absCalLong(spanDelta) < FEED_LC_MIN_SPAN_DELTA_RAW) {
    loadCellConsolePrintf("[LOADCELL CAL] Invalid span. zero_a=%ld zero_b=%ld span_a=%ld span_b=%ld summed_delta=%ld min_abs_delta=%ld\n",
                          gLoadCellCal.zeroA,
                          gLoadCellCal.zeroB,
                          gLoadCellCal.spanA,
                          gLoadCellCal.spanB,
                          spanDelta,
                          FEED_LC_MIN_SPAN_DELTA_RAW);
    loadCellConsolePrint("[LOADCELL CAL] Keep the known mass on the tank, wait for stable readings, then retry: span");
    return;
  }

  loadCellConsolePrintf("[LOADCELL CAL] Captured span samples=%u summed_delta=%ld known_kg=%.3f\n",
                        (unsigned int)samples,
                        spanDelta,
                        gLoadCellCal.knownKg);
  loadCellConsolePrint("[LOADCELL CAL] Copy these into IOTFirmwaremk1/include/sf_config.h:");
  loadCellConsolePrintf("static const long FEED_LC_A_ZERO_RAW = %ldL;\n", gLoadCellCal.zeroA);
  loadCellConsolePrintf("static const long FEED_LC_B_ZERO_RAW = %ldL;\n", gLoadCellCal.zeroB);
  loadCellConsolePrintf("static const long FEED_LC_A_SPAN_RAW = %ldL;\n", gLoadCellCal.spanA);
  loadCellConsolePrintf("static const long FEED_LC_B_SPAN_RAW = %ldL;\n", gLoadCellCal.spanB);
  loadCellConsolePrintf("static const float FEED_LC_SPAN_KG = %.6ff;\n", gLoadCellCal.knownKg);
  loadCellConsolePrint("[LOADCELL CAL] Calibration mode will keep running. OTA the edited firmware, then reboot.");
}

static void processLoadCellCalibrationLine(const String& rawLine) {
  String line = rawLine;
  line.trim();
  line.toLowerCase();

  if (line.length() == 0) return;

  if (line == "help") {
    printLoadCellCommands();
    return;
  }

  if (line == "status") {
    printLoadCellStatus(true);
    return;
  }

  if (line == "zero") {
    uint8_t samples = 0U;
    if (!captureLoadCellAverage(gLoadCellCal.zeroA, gLoadCellCal.zeroB, samples)) return;
    gLoadCellCal.zeroCaptured = true;
    gLoadCellCal.spanCaptured = false;
    loadCellConsolePrintf("[LOADCELL CAL] Empty-tank zero captured raw_a=%ld raw_b=%ld samples=%u\n",
                          gLoadCellCal.zeroA,
                          gLoadCellCal.zeroB,
                          (unsigned int)samples);
    return;
  }

  if (line.startsWith("mass ")) {
    float knownKg = 0.0f;
    if (!tryParseFloat(line.substring(5), knownKg) || knownKg <= 0.0f) {
      loadCellConsolePrint("[LOADCELL CAL] Usage: mass <known_kg>. Example: mass 2.000");
      return;
    }
    gLoadCellCal.knownKg = knownKg;
    gLoadCellCal.knownMassSet = true;
    gLoadCellCal.spanCaptured = false;
    loadCellConsolePrintf("[LOADCELL CAL] Known calibration mass set to %.3fkg\n", gLoadCellCal.knownKg);
    return;
  }

  if (line == "span") {
    printLoadCellCopyPasteConstants();
    return;
  }

  if (line == "cancel") {
    float storedFallbackKg = readRemainingKg();
    if (!isFeederLoadCellCalibrationValid() && storedFallbackKg <= 0.0f) {
      loadCellConsolePrint("[LOADCELL CAL] Cancel refused: hardcoded calibration is invalid and stored fallback is 0kg.");
      loadCellConsolePrint("[LOADCELL CAL] Capture a valid calibration or OTA firmware with nonzero constants.");
      return;
    }

    gLoadCellCal.active = false;
    gSerialConsoleExclusive = false;
    loadCellConsolePrintf("[LOADCELL CAL] Calibration mode cancelled. hardcoded_valid=%d stored_fallback_kg=%.3f\n",
                          isFeederLoadCellCalibrationValid() ? 1 : 0,
                          storedFallbackKg);
    return;
  }

  loadCellConsolePrint("[LOADCELL CAL] Unknown command.");
  printLoadCellCommands();
}

}  // namespace

void startLoadCellCalibrationSession(float knownKg, const char* reason) {
  if (gLoadCellCal.active) {
    if (knownKg > 0.0f) {
      gLoadCellCal.knownKg = knownKg;
      gLoadCellCal.knownMassSet = true;
      loadCellConsolePrintf("[LOADCELL CAL] Known calibration mass updated to %.3fkg\n", knownKg);
    }
    printLoadCellStatus(true);
    return;
  }

  stopActuatorsForSafety("load_cell_calibration");
  initFeederLoadCells();

  gLoadCellCal = LoadCellCalibrationSession();
  gLoadCellCal.active = true;
  gLoadCellCal.knownKg = knownKg > 0.0f ? knownKg : 0.0f;
  gLoadCellCal.knownMassSet = knownKg > 0.0f;
  gLoadCellCal.lastReportMs = 0UL;
  snprintf(gLoadCellCal.reason,
           sizeof(gLoadCellCal.reason),
           "%s",
           reason != nullptr ? reason : "manual");

  gSerialConsoleExclusive = true;
  Serial.println();
  loadCellConsolePrintf("[LOADCELL CAL] Serial output is now in feeder load-cell calibration mode. reason=%s\n",
                        gLoadCellCal.reason);
  loadCellConsolePrint("[LOADCELL CAL] Normal app work is paused, but OTA and console input stay serviced.");
  loadCellConsolePrint("[LOADCELL CAL] Empty tank mounted on cells: type zero");
  if (gLoadCellCal.knownMassSet) {
    loadCellConsolePrintf("[LOADCELL CAL] Known mass preset: %.3fkg. After adding it, type span\n",
                          gLoadCellCal.knownKg);
  } else {
    loadCellConsolePrint("[LOADCELL CAL] Set known mass before span: mass <kg>");
  }
  printLoadCellCommands();
  printLoadCellStatus(false);
}

bool isLoadCellCalibrationSessionActive() {
  return gLoadCellCal.active;
}

void serviceLoadCellCalibrationSession() {
  if (!gLoadCellCal.active) return;

  unsigned long nowMs = millis();
  if (gLoadCellCal.lastReportMs != 0UL &&
      nowMs - gLoadCellCal.lastReportMs < FEED_LC_CAL_REPORT_INTERVAL_MS) {
    return;
  }
  gLoadCellCal.lastReportMs = nowMs;
  printLoadCellStatus(false);
}

static bool loadCfgDoc() {
  String cfg = loadLocalConfig();
  if (cfg.length() == 0) {
    Serial.println("Config is empty.");
    return false;
  }
  gSerialCfgDoc.clear();
  DeserializationError err = deserializeJson(gSerialCfgDoc, cfg);
  if (err) {
    Serial.printf("Config parse error: %s\n", err.c_str());
    return false;
  }
  return true;
}

static void listSchedulesCommand() {
  if (!loadCfgDoc()) return;

  JsonArray schedules = gSerialCfgDoc["schedules"].as<JsonArray>();
  if (schedules.isNull() || schedules.size() == 0) {
    Serial.println("No schedules in local config.");
    return;
  }

  Serial.println("Schedules (local config):");
  int idx = 0;
  for (JsonVariant s : schedules) {
    idx++;
    int id = s["id"] | -1;
    const char* name = s["schedule_name"] | "(unnamed)";
    const char* timeStr = s["time"] | "--:--";
    bool enabled = s["enabled"] | false;
    float amount = s["feeding_amount_kg"] | 0.0f;
    Serial.printf("  [%d] id=%d enabled=%d time=%s amount=%.3f name=%s\n",
                  idx, id, enabled ? 1 : 0, timeStr, amount, name);
  }
}

static void runScheduleNowCommand(const String& args) {
  if (!loadCfgDoc()) return;

  JsonArray schedules = gSerialCfgDoc["schedules"].as<JsonArray>();
  if (schedules.isNull() || schedules.size() == 0) {
    Serial.println("No schedules to run.");
    return;
  }

  String selector = args;
  selector.trim();

  JsonVariant selected;
  if (selector.length() == 0) {
    for (JsonVariant s : schedules) {
      if (s["enabled"] | false) {
        selected = s;
        break;
      }
    }
  } else {
    int wanted = 0;
    if (!tryParseInt(selector, wanted) || wanted <= 0) {
      Serial.println("Usage: sched_run [id]. Example: sched_run 12");
      return;
    }
    for (JsonVariant s : schedules) {
      if ((s["id"] | -1) == wanted) {
        selected = s;
        break;
      }
    }
  }

  if (selected.isNull()) {
    if (selector.length() == 0) {
      Serial.println("No enabled schedule found.");
    } else {
      Serial.printf("Schedule id=%s not found in local config.\n", selector.c_str());
    }
    return;
  }

  bool enabled = selected["enabled"] | false;
  float amt = selected["feeding_amount_kg"] | 0.0f;
  int id = selected["id"] | -1;
  const char* name = selected["schedule_name"] | "(unnamed)";
  const char* timeStr = selected["time"] | "--:--";

  if (!enabled) {
    Serial.printf("Schedule id=%d is disabled; not running.\n", id);
    return;
  }
  if (amt <= 0.0f) {
    Serial.printf("Schedule id=%d has invalid amount %.3f; not running.\n", id, amt);
    return;
  }

  Serial.printf("Manual run schedule id=%d name=%s time=%s amount=%.3f (time ignored)\n", id, name, timeStr, amt);
  JsonVariant cfg = gSerialCfgDoc.as<JsonVariant>();
  if (isFeedSufficientIncludingQueued(amt, cfg)) {
    if (dispenseFeedManual(amt, cfg, "serial_schedule_run")) {
      Serial.printf("Manual schedule run queued. pending_feed_requests=%u\n", getPendingFeedRequestCount());
    } else {
      Serial.println("Manual schedule run blocked: feed request queue full.");
    }
  } else {
    Serial.println("Manual schedule run blocked: insufficient unreserved feed.");
    sendAlert("low_feed");
  }
}

void printSerialHelp() {
  Serial.println("================ SMART FEEDER SERIAL COMMANDS ================");
  Serial.println("help         -> show this command list");
  Serial.println("dump_cfg     -> print full local JSON config from Preferences");
  Serial.println("dump_state   -> print runtime state snapshot");
  Serial.println("dump_prefs   -> print key preferences values");
  Serial.println("sched_list   -> list schedules from local config");
  Serial.println("sched_run    -> run schedule now (ignore time): sched_run [id]");
  Serial.println("manual_feed_reset -> reset manual-feed snapshot to last handled id");
  Serial.println("battery_shutdown -> request forced shutdown using the safe network-flush protocol");
  Serial.println("keypad_input -> toggle keypad polling: keypad_input [toggle|on|off|status]");
  Serial.println("keypad_cal   -> interactive local min-max keypad calibration wizard");
  Serial.println("keypad_cal_dump -> print local-only keypad calibration ranges");
  Serial.println("loadcell_cal -> feeder HX711 calibration mode: loadcell_cal [known_kg]");
  Serial.println("dump_outbox  -> print buffered offline event queue");
  Serial.println("===============================================================");
}

static void keypadInputCommand(const String& args) {
  String action = args;
  action.trim();
  action.toLowerCase();

  bool current = isKeypadInputEnabled();
  bool next = current;

  if (action.length() == 0 || action == "toggle") {
    next = !current;
  } else if (action == "on" || action == "1" || action == "true") {
    next = true;
  } else if (action == "off" || action == "0" || action == "false") {
    next = false;
  } else if (action == "status") {
    keypadConsolePrintf("keypad_input_enabled=%d\n", current ? 1 : 0);
    return;
  } else {
    keypadConsolePrint("Usage: keypad_input [toggle|on|off|status]");
    return;
  }

  String cfg = loadLocalConfig();
  DynamicJsonDocument d(8192);
  d.clear();
  DeserializationError err = deserializeJson(d, cfg);
  if (err) {
    keypadConsolePrintf("Failed to parse config for keypad_input save: %s\n", err.c_str());
    return;
  }

  d["keypad_input_enabled"] = next;
  d["last_updated"] = getUtcIsoNow();
  d["updated_by"] = "esp32";

  String out;
  serializeJson(d, out);
  saveLocalConfig(out);
  markLocalConfigDirty();
  setKeypadInputEnabled(next);
  keypadConsolePrintf("keypad_input_enabled=%d\n", next ? 1 : 0);
}

static void dumpKeypadCalibrationCommand() {
  String raw = loadLocalKeypadCalibration();
  DynamicJsonDocument calibration(3072);
  DeserializationError err = deserializeJson(calibration, raw);
  if (err) {
    keypadConsolePrintf("Local keypad calibration parse error: %s\n", err.c_str());
    return;
  }

  keypadConsolePrint("----- BEGIN LOCAL KEYPAD CALIBRATION -----");
  keypadConsolePrintf("version=%d source=%s calibrated_at=%s idle=%d..%d\n",
                      calibration["version"] | 0,
                      calibration["source"] | "",
                      calibration["calibrated_at"] | "",
                      calibration["idle_min"] | -1,
                      calibration["idle_max"] | -1);
  JsonArray keys = calibration["keys"].as<JsonArray>();
  if (keys.isNull()) {
    keypadConsolePrint("keys=(missing)");
  } else {
    for (JsonVariant entry : keys) {
      keypadConsolePrintf("key=%s min=%d max=%d width=%d\n",
                          entry["key"] | "?",
                          entry["min"] | -1,
                          entry["max"] | -1,
                          (entry["max"] | -1) - (entry["min"] | -1));
    }
  }
  keypadConsolePrintf("json_bytes=%u\n", (unsigned int)raw.length());
  keypadConsolePrint("----- END LOCAL KEYPAD CALIBRATION -----");
}

void dumpLocalConfigToSerial() {
  String cfg = loadLocalConfig();
  Serial.println("----- BEGIN ESP32 LOCAL CONFIG JSON -----");
  Serial.println(cfg);
  Serial.println("----- END ESP32 LOCAL CONFIG JSON -----");
}

static void resetManualFeedSnapshotCommand() {
  uint32_t lastHandledId = readLastFeedNowCommandId();
  if (!resetManualFeedSnapshotToLastFeedNowCommandId()) {
    Serial.printf("Manual feed snapshot reset failed (last id=%lu)\n", (unsigned long)lastHandledId);
    return;
  }

  Serial.printf("Manual feed snapshot reset to id=%lu\n", (unsigned long)lastHandledId);
}

static void forceBatteryShutdownCommand() {
  if (state.lowBatteryShutdownPending || state.lowBatteryShutdownTriggered) {
    Serial.printf("Battery shutdown already pending=%d triggered=%d\n",
                  state.lowBatteryShutdownPending ? 1 : 0,
                  state.lowBatteryShutdownTriggered ? 1 : 0);
    return;
  }

  float batteryVoltage = -1.0f;
  float thresholdVoltage = DEFAULT_LOW_BATTERY_SHUTDOWN_V;
  if (loadCfgDoc()) {
    JsonVariant cfg = gSerialCfgDoc.as<JsonVariant>();
    batteryVoltage = getBatteryVoltageV(cfg);
    thresholdVoltage = getConfigOrDefault(
        cfg,
        "low_battery_shutdown_v",
        DEFAULT_LOW_BATTERY_SHUTDOWN_V);
  }

  if (!requestForcedBatteryShutdown(batteryVoltage, thresholdVoltage, "serial_command")) {
    Serial.println("Forced battery shutdown request was not accepted.");
    return;
  }

  Serial.printf("Forced battery shutdown accepted batt=%.2fV threshold=%.2fV; relay will wait for shutdown protocol.\n",
                batteryVoltage,
                thresholdVoltage);
}

void dumpStateToSerial() {
  uint32_t lastHandledId = readLastFeedNowCommandId();
  float storedRemainingKg = readRemainingKg();
  float liveRemainingKg = -1.0f;
  uint32_t snapshotId = 0UL;
  uint32_t pendingId = 0UL;
  int synced = -1;
  if (loadCfgDoc()) {
    JsonVariant cfg = gSerialCfgDoc.as<JsonVariant>();
    liveRemainingKg = getFeederRemainingKg(cfg);
    snapshotId = getManualFeedSnapshotId(cfg);
    pendingId = getFeedNowCommandIdFromConfig(cfg);
    synced = isManualFeedSnapshotSynced(cfg) ? 1 : 0;
  }

  Serial.println("----- BEGIN ESP32 RUNTIME STATE -----");
  Serial.printf("wifi_connected=%d\n", WiFi.status() == WL_CONNECTED ? 1 : 0);
  Serial.printf("wifi_ip=%s\n", WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "0.0.0.0");
  Serial.printf("mains_power_present=%d\n", state.mainsPowerPresent ? 1 : 0);
  Serial.printf("is_refilling=%d\n", state.isRefilling ? 1 : 0);
  Serial.printf("remaining_kg=%.4f\n", liveRemainingKg >= 0.0f ? liveRemainingKg : storedRemainingKg);
  Serial.printf("live_remaining_kg=%.4f\n", liveRemainingKg);
  Serial.printf("stored_remaining_kg=%.4f\n", storedRemainingKg);
  Serial.printf("last_feed_now_command_id=%lu\n", (unsigned long)lastHandledId);
  Serial.printf("manual_feed_snapshot_id=%lu\n", (unsigned long)snapshotId);
  Serial.printf("manual_feed_pending_id=%lu\n", (unsigned long)pendingId);
  if (synced >= 0) {
    Serial.printf("manual_feed_synced=%d\n", synced);
  }
  Serial.printf("async_http_pending_count=%u\n", getAsyncHttpPendingCount());
  Serial.printf("buffered_event_count=%u\n", state.bufferedEventCount);
  Serial.printf("feed_request_count=%u\n", getPendingFeedRequestCount());
  Serial.printf("feed_reserved_kg=%.4f\n", getReservedFeedKg());
  Serial.printf("feed_loadcell_calibration_valid=%d\n", isFeederLoadCellCalibrationValid() ? 1 : 0);
  Serial.printf("feed_loadcell_target_sps=%u\n", (unsigned int)FEED_LC_TARGET_SAMPLE_RATE_SPS);
  Serial.printf("feed_loadcell_min_sps=%u\n", (unsigned int)FEED_LC_MIN_OBSERVED_SAMPLE_RATE_SPS);
  Serial.printf("feed_loadcell_observed_sps=%.2f\n", getFeederLoadCellObservedSampleRateSps());
  long loadCellRawA = 0;
  long loadCellRawB = 0;
  unsigned long loadCellAgeMs = 0;
  if (getFeederLoadCellLatestRaw(loadCellRawA, loadCellRawB, loadCellAgeMs)) {
    Serial.printf("feed_loadcell_raw_a=%ld\n", loadCellRawA);
    Serial.printf("feed_loadcell_raw_b=%ld\n", loadCellRawB);
    Serial.printf("feed_loadcell_raw_age_ms=%lu\n", loadCellAgeMs);
  }
  Serial.printf("water_refill_actuator_active=%d\n", isWaterRefillActuatorActive() ? 1 : 0);
  Serial.printf("low_battery_shutdown_pending=%d\n", state.lowBatteryShutdownPending ? 1 : 0);
  Serial.printf("low_battery_shutdown_triggered=%d\n", state.lowBatteryShutdownTriggered ? 1 : 0);
  Serial.printf("uptime_ms=%lu\n", millis());
  Serial.println("----- END ESP32 RUNTIME STATE -----");
}

void dumpPrefsToSerial() {
  Serial.println("----- BEGIN ESP32 PREFS SNAPSHOT -----");
  Serial.printf("namespace=%s\n", PREF_NAMESPACE);
  Serial.printf("config_key=%s\n", PREF_CONFIG_KEY);
  Serial.printf("keypad_calibration_key=%s\n", PREF_KEYPAD_CALIBRATION);
  Serial.printf("remaining_key=%s\n", PREF_REMAINING_KG);
  Serial.printf("stored_remaining_kg=%.4f\n", readRemainingKg());
  Serial.printf("feed_loadcell_calibration_valid=%d\n", isFeederLoadCellCalibrationValid() ? 1 : 0);
  Serial.printf("feed_lc_a_zero_raw=%ld\n", FEED_LC_A_ZERO_RAW);
  Serial.printf("feed_lc_b_zero_raw=%ld\n", FEED_LC_B_ZERO_RAW);
  Serial.printf("feed_lc_a_span_raw=%ld\n", FEED_LC_A_SPAN_RAW);
  Serial.printf("feed_lc_b_span_raw=%ld\n", FEED_LC_B_SPAN_RAW);
  Serial.printf("feed_lc_span_kg=%.6f\n", FEED_LC_SPAN_KG);
  Serial.printf("feed_lc_target_sps=%u\n", (unsigned int)FEED_LC_TARGET_SAMPLE_RATE_SPS);
  Serial.printf("feed_lc_min_observed_sps=%u\n", (unsigned int)FEED_LC_MIN_OBSERVED_SAMPLE_RATE_SPS);
  Serial.printf("feed_lc_observed_sps=%.2f\n", getFeederLoadCellObservedSampleRateSps());
  Serial.printf("async_http_pending_count=%u\n", getAsyncHttpPendingCount());
  Serial.printf("feed_request_count=%u\n", getPendingFeedRequestCount());
  Serial.printf("feed_reserved_kg=%.4f\n", getReservedFeedKg());
  Serial.printf("water_refill_actuator_active=%d\n", isWaterRefillActuatorActive() ? 1 : 0);
  Serial.printf("last_feed_now_command_id=%lu\n", (unsigned long)readLastFeedNowCommandId());
  if (loadCfgDoc()) {
    JsonVariant cfg = gSerialCfgDoc.as<JsonVariant>();
    Serial.printf("live_remaining_kg=%.4f\n", getFeederRemainingKg(cfg));
    Serial.printf("manual_feed_snapshot_id=%lu\n", (unsigned long)getManualFeedSnapshotId(cfg));
    Serial.printf("manual_feed_pending_id=%lu\n", (unsigned long)getFeedNowCommandIdFromConfig(cfg));
    Serial.printf("manual_feed_synced=%d\n", isManualFeedSnapshotSynced(cfg) ? 1 : 0);
  }
  String cfg = loadLocalConfig();
  String keypadCalibration = loadLocalKeypadCalibration();
  Serial.printf("config_bytes=%u\n", (unsigned int)cfg.length());
  Serial.printf("keypad_calibration_bytes=%u\n", (unsigned int)keypadCalibration.length());
  Serial.println("----- END ESP32 PREFS SNAPSHOT -----");
}

void executeSerialCommand(const String& rawCmd) {
  String cmd = rawCmd;
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  if (gLoadCellCal.active) {
    processLoadCellCalibrationLine(cmd);
    return;
  }

  if (gKeypadCal.active) {
    processKeypadCalibrationLine(cmd);
    return;
  }

  LOG_INFO("Serial command received: %s", cmd.c_str());

  if (cmd == "help") {
    printSerialHelp();
    return;
  }

  if (cmd == "dump_cfg") {
    dumpLocalConfigToSerial();
    return;
  }

  if (cmd == "dump_state") {
    dumpStateToSerial();
    return;
  }

  if (cmd == "dump_prefs") {
    dumpPrefsToSerial();
    return;
  }

  if (cmd == "dump_outbox") {
    Serial.println("----- BEGIN ESP32 EVENT OUTBOX -----");
    Serial.println(loadEventOutbox());
    Serial.println("----- END ESP32 EVENT OUTBOX -----");
    return;
  }

  if (cmd == "sched_list") {
    listSchedulesCommand();
    return;
  }

  if (cmd == "sched_run") {
    runScheduleNowCommand("");
    return;
  }

  if (cmd.startsWith("sched_run ")) {
    runScheduleNowCommand(cmd.substring(10));
    return;
  }

  if (cmd == "manual_feed_reset" || cmd == "reset_manual_feed_snapshot") {
    resetManualFeedSnapshotCommand();
    return;
  }

  if (cmd == "battery_shutdown" || cmd == "shutdown_now") {
    forceBatteryShutdownCommand();
    return;
  }

  if (cmd == "keypad_input" || cmd.startsWith("keypad_input ")) {
    keypadInputCommand(cmd.substring(String("keypad_input").length()));
    return;
  }

  if (cmd == "loadcell_cal" || cmd.startsWith("loadcell_cal ")) {
    String args = cmd.substring(String("loadcell_cal").length());
    args.trim();
    float knownKg = 0.0f;
    if (args.length() > 0 && (!tryParseFloat(args, knownKg) || knownKg <= 0.0f)) {
      Serial.println("Usage: loadcell_cal [known_kg]. Example: loadcell_cal 2.000");
      return;
    }
    startLoadCellCalibrationSession(knownKg, "serial_command");
    return;
  }

  if (cmd == "keypad_cal") {
    startKeypadCalibrationSession();
    return;
  }

  if (cmd == "keypad_cal_dump" || cmd == "dump_keypad_cal") {
    dumpKeypadCalibrationCommand();
    return;
  }

  Serial.printf("Unknown command: %s\n", cmd.c_str());
  printSerialHelp();
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      bool routeToCommandPort = Serial.lastReadWasCommandPort();
      if (routeToCommandPort) {
        SFMultiConsole.beginCommandResponse();
      }
      executeSerialCommand(serialCmdBuffer);
      if (routeToCommandPort) {
        SFMultiConsole.endCommandResponse();
      }
      serialCmdBuffer = "";
      continue;
    }
    serialCmdBuffer += c;
    if (serialCmdBuffer.length() > 120) {
      serialCmdBuffer = "";
      Serial.println("Command too long; buffer cleared.");
    }
  }
}
