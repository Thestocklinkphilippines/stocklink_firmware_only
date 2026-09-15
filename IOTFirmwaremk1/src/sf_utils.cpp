#include "sf_utils.h"

#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include "sf_config.h"
#include "sf_debug.h"
#include "sf_globals.h"
#include "sf_pins.h"
#include "sf_sensors.h"

float clampf(float v, float minV, float maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

void configureDeviceTimezone() {
  const char* currentTz = getenv("TZ");
  if (currentTz != nullptr && strcmp(currentTz, DEVICE_TZ_POSIX) == 0) {
    return;
  }

  setenv("TZ", DEVICE_TZ_POSIX, 1);
  tzset();
  LOG_INFO("Device timezone configured tz=%s offset_sec=%ld", DEVICE_TZ_POSIX, DEVICE_TZ_OFFSET_SECONDS);
}

String getUtcIsoNow() {
  time_t now;
  time(&now);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

time_t parseIsoUtc(const char* iso) {
  auto parseTzOffsetSeconds = [](const char* tzPart) -> int {
    if (!tzPart || *tzPart == '\0') return 0;
    if (*tzPart == 'Z' || *tzPart == 'z') return 0;
    if (*tzPart != '+' && *tzPart != '-') return 0;

    int sign = (*tzPart == '-') ? -1 : 1;
    int hh = 0;
    int mm = 0;
    const char* p = tzPart + 1;

    if (!isdigit((unsigned char)p[0]) || !isdigit((unsigned char)p[1])) return 0;
    hh = (p[0] - '0') * 10 + (p[1] - '0');
    p += 2;

    if (*p == ':') {
      p++;
      if (!isdigit((unsigned char)p[0]) || !isdigit((unsigned char)p[1])) return sign * hh * 3600;
      mm = (p[0] - '0') * 10 + (p[1] - '0');
    } else if (isdigit((unsigned char)p[0]) && isdigit((unsigned char)p[1])) {
      mm = (p[0] - '0') * 10 + (p[1] - '0');
    }

    return sign * (hh * 3600 + mm * 60);
  };

  struct tm tm = {};
  if (!iso) return 0;

  char* end = strptime(iso, "%Y-%m-%dT%H:%M:%S", &tm);
  if (end == NULL) {
    end = strptime(iso, "%Y-%m-%d %H:%M:%S", &tm);
  }
  if (end == NULL) return 0;

  if (*end == '.') {
    end++;
    while (*end && isdigit((unsigned char)*end)) end++;
  }

  int tzOffsetSeconds = parseTzOffsetSeconds(end);

  const char* tz = getenv("TZ");
  bool hadTz = tz != nullptr;
  String previousTz = hadTz ? String(tz) : "";
  setenv("TZ", "UTC", 1);
  tzset();
  time_t t = mktime(&tm);
  if (hadTz) {
    setenv("TZ", previousTz.c_str(), 1);
  } else {
    unsetenv("TZ");
  }
  tzset();
  if (t <= 0) return 0;

  return t - tzOffsetSeconds;
}

namespace {

enum class NetworkTonePhase : uint8_t {
  IDLE = 0,
  FIRST,
  GAP,
  SECOND,
};

NetworkTonePhase gNetworkTonePhase = NetworkTonePhase::IDLE;
unsigned long gNetworkTonePhaseStartedMs = 0UL;
unsigned int gNetworkToneFirstHz = 0U;
unsigned int gNetworkToneSecondHz = 0U;
unsigned long gNetworkToneFirstMs = 0UL;
unsigned long gNetworkToneSecondMs = 0UL;
unsigned long gNetworkToneGapMs = 0UL;

static void startTwoToneCue(unsigned int firstHz,
                            unsigned long firstMs,
                            unsigned int secondHz,
                            unsigned long secondMs,
                            unsigned long gapMs) {
  gNetworkToneFirstHz = firstHz;
  gNetworkToneSecondHz = secondHz;
  gNetworkToneFirstMs = firstMs;
  gNetworkToneSecondMs = secondMs;
  gNetworkToneGapMs = gapMs;
  gNetworkTonePhase = NetworkTonePhase::FIRST;
  gNetworkTonePhaseStartedMs = millis();
  tone(PIN_BUZZER, gNetworkToneFirstHz);
}

}  // namespace

void playNetworkConnectedTone() {
  startTwoToneCue(
      BUZZER_NET_CONNECTED_TONE_A_HZ,
      BUZZER_NET_CONNECTED_TONE_A_MS,
      BUZZER_NET_CONNECTED_TONE_B_HZ,
      BUZZER_NET_CONNECTED_TONE_B_MS,
      BUZZER_NET_CONNECTED_TONE_GAP_MS);
}

void playNetworkDisconnectedTone() {
  startTwoToneCue(
      BUZZER_NET_DISCONNECTED_TONE_A_HZ,
      BUZZER_NET_DISCONNECTED_TONE_A_MS,
      BUZZER_NET_DISCONNECTED_TONE_B_HZ,
      BUZZER_NET_DISCONNECTED_TONE_B_MS,
      BUZZER_NET_DISCONNECTED_TONE_GAP_MS);
}

void serviceNetworkToneCue(unsigned long nowMs) {
  unsigned long elapsedMs = nowMs - gNetworkTonePhaseStartedMs;
  switch (gNetworkTonePhase) {
    case NetworkTonePhase::FIRST:
      if (elapsedMs < gNetworkToneFirstMs) return;
      noTone(PIN_BUZZER);
      gNetworkTonePhase = NetworkTonePhase::GAP;
      gNetworkTonePhaseStartedMs = nowMs;
      return;
    case NetworkTonePhase::GAP:
      if (elapsedMs < gNetworkToneGapMs) return;
      tone(PIN_BUZZER, gNetworkToneSecondHz);
      gNetworkTonePhase = NetworkTonePhase::SECOND;
      gNetworkTonePhaseStartedMs = nowMs;
      return;
    case NetworkTonePhase::SECOND:
      if (elapsedMs < gNetworkToneSecondMs) return;
      noTone(PIN_BUZZER);
      gNetworkTonePhase = NetworkTonePhase::IDLE;
      gNetworkTonePhaseStartedMs = 0UL;
      return;
    case NetworkTonePhase::IDLE:
    default:
      return;
  }
}

bool isNetworkToneCueActive() {
  return gNetworkTonePhase != NetworkTonePhase::IDLE;
}

void cancelNetworkToneCue() {
  if (gNetworkTonePhase == NetworkTonePhase::IDLE) return;
  noTone(PIN_BUZZER);
  gNetworkTonePhase = NetworkTonePhase::IDLE;
  gNetworkTonePhaseStartedMs = 0UL;
}

static void setLevelErrorBuzzerAlarmActive(bool alarmActive, unsigned long nowMs) {
  if (alarmActive == state.buzzerAlarmActive) return;

  state.buzzerAlarmActive = alarmActive;
  state.buzzerPhaseStartedMs = nowMs;
  state.buzzerAlarmCycleStartedMs = nowMs;
  state.buzzerPatternStep = 0;

  if (alarmActive) {
    LOG_WARN("Error tone enabled feeder=%.1f water=%.1f", state.lastFeederLevelPct, state.lastWaterLevelPct);
    state.buzzerResolvedToneActive = false;
    state.buzzerResolvedStep = 0;
    tone(PIN_BUZZER, BUZZER_ERROR_TONE_A_HZ);
    state.buzzerToneOn = true;
  } else {
    LOG_INFO("Error tone cleared feeder=%.1f water=%.1f", state.lastFeederLevelPct, state.lastWaterLevelPct);
    noTone(PIN_BUZZER);
    state.buzzerToneOn = false;
    state.buzzerResolvedToneActive = true;
    state.buzzerResolvedStep = 0;
    state.buzzerResolvedPhaseStartedMs = nowMs;
  }
}

void updateLevelErrorBuzzerAlarm(JsonVariant cfg) {
  float feederLowThreshold = getConfigOrDefault(
      cfg,
      "alert_feeder_low_threshold_pct",
      getConfigOrDefault(cfg, "feeder_low_threshold_pct", DEFAULT_ALERT_FEEDER_LOW_THRESHOLD_PCT));
  float waterLowThreshold = getConfigOrDefault(
      cfg,
      "alert_water_low_threshold_pct",
      getConfigOrDefault(cfg, "water_low_threshold_pct", DEFAULT_ALERT_WATER_LOW_THRESHOLD_PCT));

  bool alarmActive = state.lastFeederLevelPct <= feederLowThreshold || state.lastWaterLevelPct <= waterLowThreshold;
  unsigned long nowMs = millis();
  setLevelErrorBuzzerAlarmActive(alarmActive, nowMs);
}

void serviceLevelErrorBuzzer() {
  bool alarmActive = state.buzzerAlarmActive;
  unsigned long nowMs = millis();

  if (alarmActive) {
    unsigned long cycleElapsedMs = nowMs - state.buzzerAlarmCycleStartedMs;
    if (cycleElapsedMs >= BUZZER_ERROR_ALERT_INTERVAL_MS) {
      state.buzzerAlarmCycleStartedMs = nowMs;
      state.buzzerPhaseStartedMs = nowMs;
      state.buzzerPatternStep = 0;
      cycleElapsedMs = 0;
    }

    if (cycleElapsedMs >= BUZZER_ERROR_ALERT_WINDOW_MS) {
      if (state.buzzerToneOn) {
        noTone(PIN_BUZZER);
        state.buzzerToneOn = false;
      }
      return;
    }

    unsigned long elapsedMs = nowMs - state.buzzerPhaseStartedMs;
    if (state.buzzerToneOn) {
      unsigned long targetMs = (state.buzzerPatternStep == 0) ? BUZZER_ERROR_TONE_A_MS : BUZZER_ERROR_TONE_E_MS;
      if (elapsedMs >= targetMs) {
        noTone(PIN_BUZZER);
        state.buzzerToneOn = false;
        state.buzzerPhaseStartedMs = nowMs;
        state.buzzerPatternStep = (state.buzzerPatternStep == 0) ? 1 : 0;
      }
    } else if (elapsedMs >= BUZZER_ERROR_TONE_GAP_MS) {
      unsigned int toneHz = (state.buzzerPatternStep == 0) ? BUZZER_ERROR_TONE_A_HZ : BUZZER_ERROR_TONE_E_HZ;
      tone(PIN_BUZZER, toneHz);
      state.buzzerToneOn = true;
      state.buzzerPhaseStartedMs = nowMs;
    }
    return;
  }

  if (state.buzzerToneOn) {
    noTone(PIN_BUZZER);
    state.buzzerToneOn = false;
  }

  if (!state.buzzerResolvedToneActive) {
    return;
  }

  unsigned long resolvedElapsedMs = nowMs - state.buzzerResolvedPhaseStartedMs;
  if (state.buzzerResolvedStep == 0) {
    tone(PIN_BUZZER, BUZZER_RESOLVED_TONE_C_HZ, BUZZER_RESOLVED_TONE_C_MS);
    state.buzzerResolvedStep = 1;
    state.buzzerResolvedPhaseStartedMs = nowMs;
    return;
  }

  if (state.buzzerResolvedStep == 1 && resolvedElapsedMs >= BUZZER_RESOLVED_TONE_GAP_MS) {
    tone(PIN_BUZZER, BUZZER_RESOLVED_TONE_E_HZ, BUZZER_RESOLVED_TONE_E_MS);
    state.buzzerResolvedStep = 2;
    state.buzzerResolvedPhaseStartedMs = nowMs;
    return;
  }

  if (state.buzzerResolvedStep == 2 && resolvedElapsedMs >= BUZZER_RESOLVED_TONE_E_MS) {
    noTone(PIN_BUZZER);
    state.buzzerResolvedToneActive = false;
    state.buzzerResolvedStep = 0;
    state.buzzerResolvedPhaseStartedMs = nowMs;
  }
}
