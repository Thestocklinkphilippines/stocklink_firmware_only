#include "sf_actuators.h"

#include <string.h>

#include "sf_config.h"
#include "sf_debug.h"
#include "sf_globals.h"
#include "sf_network.h"
#include "sf_pins.h"
#include "sf_sensors.h"
#include "sf_storage.h"
#include "sf_utils.h"

namespace {

enum class FeedRequestKind : uint8_t {
  UNKNOWN = 0,
  MANUAL,
  SCHEDULED,
  FEED_NOW,
};

enum class FeedActuatorPhase : uint8_t {
  IDLE = 0,
  TONE_C,
  TONE_GAP,
  TONE_E,
  MOTOR,
};

enum class WaterActuatorMode : uint8_t {
  IDLE = 0,
  PROBING,
  CONTINUOUS,
};

struct FeedRequest {
  FeedRequestKind kind = FeedRequestKind::UNKNOWN;
  float amountKg = 0.0f;
  unsigned long runMs = 0UL;
  uint32_t commandId = 0UL;
  int scheduleId = 0;
  char source[32] = "";
  char scheduleName[64] = "";
  char scheduleTime[9] = "";
};

FeedRequest gFeedQueue[FEED_REQUEST_QUEUE_SIZE];
uint8_t gFeedQueueHead = 0U;
uint8_t gFeedQueueTail = 0U;
uint8_t gFeedQueueCount = 0U;
FeedRequest gActiveFeed;
bool gFeedActive = false;
FeedActuatorPhase gFeedPhase = FeedActuatorPhase::IDLE;
unsigned long gFeedPhaseStartedMs = 0UL;

WaterActuatorMode gWaterMode = WaterActuatorMode::IDLE;
unsigned long gWaterStartedMs = 0UL;
unsigned long gWaterLastCheckMs = 0UL;
unsigned long gWaterLastProgressMs = 0UL;
float gWaterBaselinePct = 0.0f;
float gWaterLastProgressPct = 0.0f;
uint8_t gWaterProgressConfirmations = 0U;

static void copyText(char* target, size_t targetSize, const char* source) {
  if (target == nullptr || targetSize == 0U) return;
  snprintf(target, targetSize, "%s", source != nullptr ? source : "");
}

static const char* feedKindName(FeedRequestKind kind) {
  switch (kind) {
    case FeedRequestKind::MANUAL:
      return "manual";
    case FeedRequestKind::SCHEDULED:
      return "scheduled";
    case FeedRequestKind::FEED_NOW:
      return "feed_now";
    case FeedRequestKind::UNKNOWN:
    default:
      return "unknown";
  }
}

static float resolveGrainMsPerKg(JsonVariant cfg) {
  int selectedIndex = getSelectedGrainTypeIndex(cfg);
  float resolved = getGrainTypeMsPerKgByIndex(cfg, selectedIndex);
  return resolved > 0.0f ? resolved : FEED_MS_PER_KG_STANDARD_PELLETS;
}

static float getAvailableFeedKg(JsonVariant cfg) {
  float remaining = getFeederRemainingKg(cfg);
  const char* source = "load_cell";
  if (remaining < 0.0f) {
    remaining = readRemainingKg();
    source = "stored_fallback";
  }
  float available = remaining - getReservedFeedKg();
  if (available < 0.0f) available = 0.0f;
  LOG_DEBUG("Feed queue availability remaining=%.3fkg reserved=%.3fkg available=%.3fkg source=%s",
            remaining,
            getReservedFeedKg(),
            available,
            source);
  return available;
}

static bool enqueueFeedRequest(const FeedRequest& request) {
  if (request.amountKg <= 0.0f || request.runMs == 0UL) return false;
  if (gFeedQueueCount >= FEED_REQUEST_QUEUE_SIZE) {
    LOG_WARN("Feed request queue full kind=%s amount=%.3f queued=%u",
             feedKindName(request.kind),
             request.amountKg,
             (unsigned int)gFeedQueueCount);
    return false;
  }

  gFeedQueue[gFeedQueueTail] = request;
  gFeedQueueTail = (uint8_t)((gFeedQueueTail + 1U) % FEED_REQUEST_QUEUE_SIZE);
  gFeedQueueCount++;
  LOG_INFO("Feed request queued kind=%s amount=%.3f run_ms=%lu pending=%u",
           feedKindName(request.kind),
           request.amountKg,
           request.runMs,
           (unsigned int)gFeedQueueCount);
  return true;
}

static void startNextFeedRequest(unsigned long nowMs) {
  if (gFeedActive || gFeedQueueCount == 0U) return;

  gActiveFeed = gFeedQueue[gFeedQueueHead];
  gFeedQueueHead = (uint8_t)((gFeedQueueHead + 1U) % FEED_REQUEST_QUEUE_SIZE);
  gFeedQueueCount--;
  gFeedActive = true;
  gFeedPhase = FeedActuatorPhase::TONE_C;
  gFeedPhaseStartedMs = nowMs;
  tone(PIN_BUZZER, BUZZER_FEED_TONE_C_HZ, BUZZER_FEED_TONE_C_MS);
  LOG_INFO("Feed request started kind=%s amount=%.3f run_ms=%lu pending=%u",
           feedKindName(gActiveFeed.kind),
           gActiveFeed.amountKg,
           gActiveFeed.runMs,
           (unsigned int)gFeedQueueCount);
}

static void appendFeedMetadata(JsonVariant payload, const FeedRequest& request) {
  switch (request.kind) {
    case FeedRequestKind::MANUAL:
      payload["trigger"] = "manual";
      payload["feed_type"] = "manual";
      payload["source"] = request.source[0] != '\0' ? request.source : "control_panel";
      break;
    case FeedRequestKind::SCHEDULED:
      payload["trigger"] = "schedule";
      payload["feed_type"] = "scheduled";
      payload["source"] = "scheduler";
      payload["schedule_id"] = request.scheduleId;
      if (request.scheduleName[0] != '\0') payload["schedule_name"] = request.scheduleName;
      if (request.scheduleTime[0] != '\0') payload["schedule_time"] = request.scheduleTime;
      break;
    case FeedRequestKind::FEED_NOW:
      payload["trigger"] = "feed_now";
      payload["feed_type"] = "manual";
      payload["source"] = "server";
      payload["command_id"] = request.commandId;
      payload["status"] = "executed";
      payload["phase"] = "completed";
      break;
    case FeedRequestKind::UNKNOWN:
    default:
      payload["trigger"] = "unknown";
      payload["feed_type"] = "unknown";
      payload["source"] = "firmware";
      break;
  }
}

static void completeFeedRequest(JsonVariant cfg) {
  setFeedMotorEnabled(false);
  noTone(PIN_BUZZER);

  float remaining = getFeederRemainingKg(cfg);
  if (remaining < 0.0f) {
    remaining = readRemainingKg() - gActiveFeed.amountKg;
    if (remaining < 0.0f) remaining = 0.0f;
  }
  writeRemainingKg(remaining);
  addToDailyFeedTotalKg(gActiveFeed.amountKg);

  StaticJsonDocument<512> feeding;
  feeding["amount_kg"] = gActiveFeed.amountKg;
  feeding["remaining_kg"] = remaining;
  feeding["feeder_level_pct"] = state.lastFeederLevelPct;
  appendFeedMetadata(feeding.as<JsonVariant>(), gActiveFeed);
  sendLog("feeding", feeding.as<JsonVariant>());

  if (gActiveFeed.kind == FeedRequestKind::FEED_NOW && gActiveFeed.commandId > 0UL) {
    bool ackOk = sendFeedNowAck(gActiveFeed.commandId, "executed", nullptr);
    if (!ackOk) {
      LOG_WARN("feed_now completion ack queued/failed id=%lu", (unsigned long)gActiveFeed.commandId);
    }
    writeLastFeedNowCommandId(gActiveFeed.commandId);

    StaticJsonDocument<256> feedNow;
    feedNow["event"] = "feed_now";
    feedNow["command_id"] = gActiveFeed.commandId;
    feedNow["amount_kg"] = gActiveFeed.amountKg;
    feedNow["status"] = "executed";
    sendLog("feed_now", feedNow.as<JsonVariant>());
  }

  LOG_INFO("Feed request completed kind=%s amount=%.3f remaining=%.3f",
           feedKindName(gActiveFeed.kind),
           gActiveFeed.amountKg,
           remaining);
  gFeedActive = false;
  gFeedPhase = FeedActuatorPhase::IDLE;
  gFeedPhaseStartedMs = 0UL;
  gActiveFeed = FeedRequest();
}

static void serviceFeedActuator(JsonVariant cfg, unsigned long nowMs) {
  if (!gFeedActive) {
    startNextFeedRequest(nowMs);
    return;
  }

  unsigned long phaseElapsedMs = nowMs - gFeedPhaseStartedMs;
  switch (gFeedPhase) {
    case FeedActuatorPhase::TONE_C:
      if (phaseElapsedMs < BUZZER_FEED_TONE_C_MS) return;
      noTone(PIN_BUZZER);
      gFeedPhase = FeedActuatorPhase::TONE_GAP;
      gFeedPhaseStartedMs = nowMs;
      return;
    case FeedActuatorPhase::TONE_GAP:
      if (phaseElapsedMs < BUZZER_FEED_TONE_GAP_MS) return;
      tone(PIN_BUZZER, BUZZER_FEED_TONE_E_HZ, BUZZER_FEED_TONE_E_MS);
      gFeedPhase = FeedActuatorPhase::TONE_E;
      gFeedPhaseStartedMs = nowMs;
      return;
    case FeedActuatorPhase::TONE_E:
      if (phaseElapsedMs < BUZZER_FEED_TONE_E_MS) return;
      noTone(PIN_BUZZER);
      setFeedMotorEnabled(true);
      gFeedPhase = FeedActuatorPhase::MOTOR;
      gFeedPhaseStartedMs = nowMs;
      LOG_INFO("Feed motor enabled kind=%s run_ms=%lu",
               feedKindName(gActiveFeed.kind),
               gActiveFeed.runMs);
      return;
    case FeedActuatorPhase::MOTOR:
      if (phaseElapsedMs < gActiveFeed.runMs) return;
      completeFeedRequest(cfg);
      return;
    case FeedActuatorPhase::IDLE:
    default:
      return;
  }
}

static void logWateringEvent(const char* eventName, float waterPct, const char* reason = nullptr) {
  StaticJsonDocument<192> payload;
  payload["event"] = eventName;
  payload["water_level_pct"] = waterPct;
  if (reason != nullptr && reason[0] != '\0') payload["reason"] = reason;
  sendLog("watering", payload.as<JsonVariant>());
}

static void stopWaterAttemptForCooldown(float waterPct, const char* reason) {
  setWaterSolenoidEnabled(false);
  gWaterMode = WaterActuatorMode::IDLE;
  gWaterProgressConfirmations = 0U;
  state.lastWaterRefillAttemptMs = millis();
  LOG_WARN("Water refill stopped for cooldown reason=%s pct=%.1f", reason, waterPct);
  logWateringEvent("refill_paused", waterPct, reason);
  sendAlert("low_water");
}

static void serviceWaterActuator(JsonVariant cfg, unsigned long nowMs) {
  if (gWaterMode == WaterActuatorMode::IDLE) return;
  if (nowMs - gWaterLastCheckMs < WATER_REFILL_PROGRESS_CHECK_INTERVAL_MS) return;
  gWaterLastCheckMs = nowMs;

  float waterHigh = getConfigOrDefault(cfg, "water_high_threshold_pct", DEFAULT_WATER_HIGH_THRESHOLD_PCT);
  float waterPct = getWaterLevelPct(cfg);
  if (waterPct >= waterHigh) {
    finishWaterRefillAtHighThreshold(waterPct);
    return;
  }

  if (gWaterMode == WaterActuatorMode::PROBING) {
    if (waterPct >= gWaterBaselinePct + WATER_REFILL_PROGRESS_DELTA_PCT) {
      if (gWaterProgressConfirmations < 255U) gWaterProgressConfirmations++;
    } else {
      gWaterProgressConfirmations = 0U;
    }

    if (gWaterProgressConfirmations >= WATER_REFILL_PROGRESS_CONFIRMATIONS) {
      gWaterMode = WaterActuatorMode::CONTINUOUS;
      gWaterLastProgressPct = waterPct;
      gWaterLastProgressMs = nowMs;
      LOG_INFO("Water refill progress confirmed baseline=%.1f current=%.1f; continuing until high threshold",
               gWaterBaselinePct,
               waterPct);
      logWateringEvent("refill_continuous", waterPct, "level_progress_confirmed");
      return;
    }

    if (nowMs - gWaterStartedMs >= WATER_REFILL_ATTEMPT_MS) {
      stopWaterAttemptForCooldown(waterPct, "no_level_progress");
    }
    return;
  }

  if (waterPct >= gWaterLastProgressPct + WATER_REFILL_PROGRESS_DELTA_PCT) {
    gWaterLastProgressPct = waterPct;
    gWaterLastProgressMs = nowMs;
  }

  if (nowMs - gWaterLastProgressMs >= WATER_REFILL_CONTINUOUS_STALL_TIMEOUT_MS) {
    stopWaterAttemptForCooldown(waterPct, "continuous_refill_stalled");
    return;
  }
  if (nowMs - gWaterStartedMs >= WATER_REFILL_CONTINUOUS_MAX_MS) {
    stopWaterAttemptForCooldown(waterPct, "continuous_refill_max_runtime");
  }
}

}  // namespace

void setFeedMotorEnabled(bool enabled) {
  int activeState = FEED_MOTOR_ACTIVE_HIGH ? HIGH : LOW;
  int idleState = FEED_MOTOR_ACTIVE_HIGH ? LOW : HIGH;
  digitalWrite(PIN_FEED_MOTOR, enabled ? activeState : idleState);
}

void setWaterSolenoidEnabled(bool enabled) {
  int activeState = WATER_SOLENOID_ACTIVE_HIGH ? HIGH : LOW;
  int idleState = WATER_SOLENOID_ACTIVE_HIGH ? LOW : HIGH;
  digitalWrite(PIN_WATER_SOLENOID, enabled ? activeState : idleState);
}

void setBatteryShutdownRelayEnabled(bool enabled) {
  digitalWrite(PIN_BATTERY_SHUTDOWN_RELAY, enabled ? HIGH : LOW);
}

unsigned long computeFeedMotorRunMs(float amountKg, JsonVariant cfg) {
  float safeAmountKg = amountKg;
  if (safeAmountKg < 0.0f) safeAmountKg = 0.0f;
  float runMs = FEED_MOTOR_STARTUP_MS + (safeAmountKg * resolveGrainMsPerKg(cfg));
  if (runMs < FEED_MOTOR_STARTUP_MS) runMs = FEED_MOTOR_STARTUP_MS;
  return (unsigned long)runMs;
}

bool isFeedSufficient(float requiredKg, JsonVariant cfg) {
  float remaining = getFeederRemainingKg(cfg);
  const char* source = "load_cell";
  if (remaining < 0.0f) {
    remaining = readRemainingKg();
    source = "stored_fallback";
  }
  LOG_DEBUG("Feed sufficiency check required=%.3fkg remaining=%.3fkg source=%s",
            requiredKg,
            remaining,
            source);
  return remaining >= requiredKg;
}

bool isFeedSufficientIncludingQueued(float requiredKg, JsonVariant cfg) {
  float available = getAvailableFeedKg(cfg);
  LOG_DEBUG("Queued feed sufficiency check required=%.3fkg available=%.3fkg", requiredKg, available);
  return available >= requiredKg;
}

float getReservedFeedKg() {
  float reserved = gFeedActive ? gActiveFeed.amountKg : 0.0f;
  for (uint8_t offset = 0U; offset < gFeedQueueCount; offset++) {
    uint8_t index = (uint8_t)((gFeedQueueHead + offset) % FEED_REQUEST_QUEUE_SIZE);
    reserved += gFeedQueue[index].amountKg;
  }
  return reserved;
}

unsigned int getPendingFeedRequestCount() {
  return (unsigned int)gFeedQueueCount + (gFeedActive ? 1U : 0U);
}

bool isFeedNowCommandQueuedOrActive(uint32_t commandId) {
  if (commandId == 0UL) return false;
  if (gFeedActive && gActiveFeed.kind == FeedRequestKind::FEED_NOW && gActiveFeed.commandId == commandId) {
    return true;
  }
  for (uint8_t offset = 0U; offset < gFeedQueueCount; offset++) {
    uint8_t index = (uint8_t)((gFeedQueueHead + offset) % FEED_REQUEST_QUEUE_SIZE);
    const FeedRequest& request = gFeedQueue[index];
    if (request.kind == FeedRequestKind::FEED_NOW && request.commandId == commandId) return true;
  }
  return false;
}

bool dispenseFeed(float amountKg, JsonVariant cfg) {
  FeedRequest request;
  request.kind = FeedRequestKind::UNKNOWN;
  request.amountKg = amountKg;
  request.runMs = computeFeedMotorRunMs(amountKg, cfg);
  return enqueueFeedRequest(request);
}

bool dispenseFeedManual(float amountKg, JsonVariant cfg, const char* source) {
  FeedRequest request;
  request.kind = FeedRequestKind::MANUAL;
  request.amountKg = amountKg;
  request.runMs = computeFeedMotorRunMs(amountKg, cfg);
  copyText(request.source, sizeof(request.source), source != nullptr ? source : "control_panel");
  return enqueueFeedRequest(request);
}

bool dispenseFeedScheduled(float amountKg, JsonVariant cfg, JsonVariant schedule) {
  FeedRequest request;
  request.kind = FeedRequestKind::SCHEDULED;
  request.amountKg = amountKg;
  request.runMs = computeFeedMotorRunMs(amountKg, cfg);
  if (!schedule.isNull()) {
    request.scheduleId = schedule["id"] | 0;
    copyText(request.scheduleName, sizeof(request.scheduleName), schedule["schedule_name"] | "");
    copyText(request.scheduleTime, sizeof(request.scheduleTime), schedule["time"] | "");
  }
  return enqueueFeedRequest(request);
}

bool dispenseFeedNow(float amountKg, JsonVariant cfg, uint32_t commandId) {
  FeedRequest request;
  request.kind = FeedRequestKind::FEED_NOW;
  request.amountKg = amountKg;
  request.runMs = computeFeedMotorRunMs(amountKg, cfg);
  request.commandId = commandId;
  return enqueueFeedRequest(request);
}

bool requestWaterRefillAttempt(float baselineWaterPct) {
  if (gWaterMode != WaterActuatorMode::IDLE) return false;
  unsigned long nowMs = millis();
  gWaterMode = WaterActuatorMode::PROBING;
  gWaterStartedMs = nowMs;
  gWaterLastCheckMs = nowMs;
  gWaterLastProgressMs = nowMs;
  gWaterBaselinePct = baselineWaterPct;
  gWaterLastProgressPct = baselineWaterPct;
  gWaterProgressConfirmations = 0U;
  state.lastWaterRefillAttemptMs = nowMs;
  setWaterSolenoidEnabled(true);
  LOG_WARN("Water refill proof attempt started baseline=%.1f duration_ms=%lu",
           baselineWaterPct,
           WATER_REFILL_ATTEMPT_MS);
  return true;
}

bool isWaterRefillActuatorActive() {
  return gWaterMode != WaterActuatorMode::IDLE;
}

void finishWaterRefillAtHighThreshold(float waterPct) {
  setWaterSolenoidEnabled(false);
  gWaterMode = WaterActuatorMode::IDLE;
  gWaterProgressConfirmations = 0U;
  state.isRefilling = false;
  LOG_INFO("Water high threshold reached; refill completed pct=%.1f", waterPct);
  logWateringEvent("refill_complete", waterPct);
}

void serviceActuators(JsonVariant cfg, unsigned long nowMs) {
  serviceFeedActuator(cfg, nowMs);
  serviceWaterActuator(cfg, nowMs);
}

void stopActuatorsForSafety(const char* reason) {
  setFeedMotorEnabled(false);
  setWaterSolenoidEnabled(false);
  cancelNetworkToneCue();
  noTone(PIN_BUZZER);
  gFeedActive = false;
  gFeedPhase = FeedActuatorPhase::IDLE;
  gFeedQueueHead = 0U;
  gFeedQueueTail = 0U;
  gFeedQueueCount = 0U;
  gWaterMode = WaterActuatorMode::IDLE;
  state.isRefilling = false;
  LOG_WARN("Actuators stopped for safety reason=%s", reason != nullptr ? reason : "unknown");
}
