#include "keypad_calc.h"

namespace {
static const char kKeypadKeysLocal[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'};

bool gPressActive = false;
int gEventMinAdc = 4095;
int gEventMaxAdc = 0;
uint16_t gEventPolls = 0;
uint16_t gMatchedPolls = 0;
uint16_t gVotes[16] = {0};
}

static int clampAdc(int value) {
  if (value < 0) return 0;
  if (value > 4095) return 4095;
  return value;
}

static bool rangeContains(const KeypadAdcRange& range, int adc) {
  return adc >= range.minAdc && adc <= range.maxAdc;
}

bool hasValidKeypadRanges(const KeypadAdcRange ranges[16]) {
  if (!ranges) return false;

  for (int i = 0; i < 16; i++) {
    if (ranges[i].minAdc < 0 || ranges[i].maxAdc > 4095 || ranges[i].minAdc > ranges[i].maxAdc) {
      return false;
    }
    for (int j = i + 1; j < 16; j++) {
      if (ranges[i].minAdc <= ranges[j].maxAdc && ranges[j].minAdc <= ranges[i].maxAdc) {
        return false;
      }
    }
  }
  return true;
}

char decodeKeypadRangeSample(int adjustedAdc, const KeypadAdcRange ranges[16]) {
  if (!hasValidKeypadRanges(ranges)) return '\0';
  adjustedAdc = clampAdc(adjustedAdc);
  for (int i = 0; i < 16; i++) {
    if (rangeContains(ranges[i], adjustedAdc)) return kKeypadKeysLocal[i];
  }
  return '\0';
}

struct SampleWindowStats {
  int averageAdc = 0;
  int minAdc = 4095;
  int maxAdc = 0;
};

static SampleWindowStats sampleAdcWindowStats(int (*adcRead)(), int samples) {
  SampleWindowStats stats;
  const int safeSamples = (samples <= 0) ? 1 : samples;
  long total = 0;

  for (int i = 0; i < safeSamples; i++) {
    int sample = adcRead();
    if (sample < 0) sample = 0;
    if (sample > 4095) sample = 4095;
    total += sample;
    if (sample < stats.minAdc) stats.minAdc = sample;
    if (sample > stats.maxAdc) stats.maxAdc = sample;
    delayMicroseconds(500);
  }

  if (safeSamples >= 3) {
    total -= stats.minAdc;
    total -= stats.maxAdc;
    stats.averageAdc = (int)(total / (safeSamples - 2));
  } else {
    stats.averageAdc = (int)(total / safeSamples);
  }

  return stats;
}

static void clearEventState() {
  gPressActive = false;
  gEventMinAdc = 4095;
  gEventMaxAdc = 0;
  gEventPolls = 0;
  gMatchedPolls = 0;
  for (int i = 0; i < 16; i++) gVotes[i] = 0;
}

static int findWinningKeyIndex(uint16_t* outVotes) {
  int winnerIndex = -1;
  uint16_t winnerVotes = 0;
  for (int i = 0; i < 16; i++) {
    if (gVotes[i] > winnerVotes) {
      winnerVotes = gVotes[i];
      winnerIndex = i;
    }
  }
  if (outVotes) *outVotes = winnerVotes;
  return winnerIndex;
}

static void populateEventStats(KeypadEventStats* outStats,
                               int rawAdc,
                               int adjustedAdc,
                               bool eventCompleted) {
  if (!outStats) return;

  uint16_t winningVotes = 0;
  int winningIndex = findWinningKeyIndex(&winningVotes);
  outStats->pressActive = gPressActive;
  outStats->eventCompleted = eventCompleted;
  outStats->pollRawAdc = rawAdc;
  outStats->pollAdjustedAdc = adjustedAdc;
  outStats->eventMinAdc = gEventPolls > 0 ? gEventMinAdc : adjustedAdc;
  outStats->eventMaxAdc = gEventPolls > 0 ? gEventMaxAdc : adjustedAdc;
  outStats->eventPolls = gEventPolls;
  outStats->matchedPolls = gMatchedPolls;
  outStats->winningVotes = winningVotes;
  outStats->winningKeyIndex = winningIndex;
  outStats->confidencePct = gMatchedPolls > 0
                                ? (uint8_t)((winningVotes * 100U) / gMatchedPolls)
                                : 0U;
}

void resetKeypadCalcState() {
  clearEventState();
}

char calculateKeypadKey(int (*adcRead)(),
                        const KeypadAdcRange ranges[16],
                        int idleMinAdc,
                        int idleMaxAdc,
                        const KeypadAdcTuning& tuning,
                        KeypadEventStats* outStats) {
  if (!adcRead || !ranges || !hasValidKeypadRanges(ranges)) {
    clearEventState();
    return '\0';
  }

  SampleWindowStats window = sampleAdcWindowStats(adcRead, tuning.samples);
  int rawAdc = window.averageAdc;
  int adjustedAdc = clampAdc(rawAdc + tuning.adcOffset);
  int idleBand = (tuning.idleBandAdc < 0) ? 0 : tuning.idleBandAdc;
  int idleMin = clampAdc(idleMinAdc - idleBand);
  int idleMax = clampAdc(idleMaxAdc + idleBand);
  bool idle = adjustedAdc >= idleMin && adjustedAdc <= idleMax;

  if (idle) {
    if (!gPressActive) {
      populateEventStats(outStats, rawAdc, adjustedAdc, false);
      return '\0';
    }

    populateEventStats(outStats, rawAdc, adjustedAdc, true);
    uint16_t winningVotes = 0;
    int winningIndex = findWinningKeyIndex(&winningVotes);
    int minEventPolls = tuning.minEventPolls < 1 ? 1 : tuning.minEventPolls;
    int minMatchedPolls = tuning.minMatchedPolls < 1 ? 1 : tuning.minMatchedPolls;
    int minConfidencePct = tuning.minConfidencePct;
    if (minConfidencePct < 1) minConfidencePct = 1;
    if (minConfidencePct > 100) minConfidencePct = 100;
    int confidencePct = gMatchedPolls > 0 ? (int)((winningVotes * 100U) / gMatchedPolls) : 0;
    bool accepted =
        winningIndex >= 0 &&
        gEventPolls >= (uint16_t)minEventPolls &&
        gMatchedPolls >= (uint16_t)minMatchedPolls &&
        confidencePct >= minConfidencePct;
    char resolvedKey = accepted ? kKeypadKeysLocal[winningIndex] : '\0';
    clearEventState();
    return resolvedKey;
  }

  if (!gPressActive) {
    clearEventState();
    gPressActive = true;
  }

  if (gEventPolls < UINT16_MAX) gEventPolls++;
  if (adjustedAdc < gEventMinAdc) gEventMinAdc = adjustedAdc;
  if (adjustedAdc > gEventMaxAdc) gEventMaxAdc = adjustedAdc;
  for (int i = 0; i < 16; i++) {
    if (!rangeContains(ranges[i], adjustedAdc)) continue;
    if (gVotes[i] < UINT16_MAX) gVotes[i]++;
    if (gMatchedPolls < UINT16_MAX) gMatchedPolls++;
    break;
  }
  populateEventStats(outStats, rawAdc, adjustedAdc, false);
  return '\0';
}
