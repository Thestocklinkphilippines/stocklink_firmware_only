#ifndef KEYPAD_CALC_H
#define KEYPAD_CALC_H

#include <Arduino.h>

struct KeypadAdcRange {
    int minAdc;
    int maxAdc;

    KeypadAdcRange(int minValue = 0, int maxValue = 0)
        : minAdc(minValue), maxAdc(maxValue) {}
};

struct KeypadAdcTuning {
    int idleBandAdc = 30;
    int adcOffset = 0;
    int samples = 5;
    int minEventPolls = 2;
    int minMatchedPolls = 2;
    int minConfidencePct = 60;
};

struct KeypadEventStats {
    bool pressActive = false;
    bool eventCompleted = false;
    int pollRawAdc = 0;
    int pollAdjustedAdc = 0;
    int eventMinAdc = 4095;
    int eventMaxAdc = 0;
    uint16_t eventPolls = 0;
    uint16_t matchedPolls = 0;
    uint16_t winningVotes = 0;
    int winningKeyIndex = -1;
    uint8_t confidencePct = 0;
};

// Standalone keypad calculation API.
// `adcRead` should be a function that returns a single ADC sample (0..4095).
// Every non-idle poll is counted toward the current press event. When the input
// returns to idle, the event is assigned to the key range with the strongest
// vote. Transition samples outside all key ranges are ignored.
// Returns a decoded key only once, when a complete press/release event resolves.
char calculateKeypadKey(int (*adcRead)(),
                        const KeypadAdcRange ranges[16],
                        int idleMinAdc,
                        int idleMaxAdc,
                        const KeypadAdcTuning& tuning,
                        KeypadEventStats* outStats = nullptr);

// Decode one already-adjusted ADC sample against calibrated key ranges.
char decodeKeypadRangeSample(int adjustedAdc, const KeypadAdcRange ranges[16]);

// Validate that all ranges are usable and do not overlap.
bool hasValidKeypadRanges(const KeypadAdcRange ranges[16]);

// Reset the in-progress press event (call after calibration changes).
void resetKeypadCalcState();

#endif
