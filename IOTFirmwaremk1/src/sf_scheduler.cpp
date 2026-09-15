#include "sf_scheduler.h"

#include <time.h>

#include "sf_actuators.h"
#include "sf_config.h"
#include "sf_debug.h"
#include "sf_globals.h"
#include "sf_network.h"
#include "sf_sensors.h"
#include "sf_storage.h"

static const char* kScheduleWeekdays[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static const time_t kScheduleValidEpochThreshold = 100000;

static bool scheduleIncludesDay(JsonArrayConst days, const char* dayName) {
  if (dayName == nullptr) return false;
  if (days.isNull() || days.size() == 0) return true;

  for (JsonVariantConst day : days) {
    const char* candidate = day | "";
    if (strcmp(candidate, dayName) == 0) return true;
  }
  return false;
}

void checkLowFeedPrediction(JsonArray schedules, JsonVariant cfg) {
  static bool sLowFeedPredictionActive = false;

  float required = 0.0f;
  int count = 0;
  for (JsonVariant s : schedules) {
    if (!s["enabled"]) continue;
    if (!s.containsKey("feeding_amount_kg")) continue;
    required += s["feeding_amount_kg"].as<float>();
    count++;
    if (count >= 2) break;
  }

  float feederThreshold = getConfigOrDefault(
      cfg,
      "alert_feeder_low_threshold_pct",
      getConfigOrDefault(cfg, "feeder_low_threshold_pct", DEFAULT_ALERT_FEEDER_LOW_THRESHOLD_PCT));
  float feederLevelPct = getFeederLevelPct(cfg);
  LOG_DEBUG("Low-feed prediction requiredNext=%.3fkg feederPct=%.1f alertThreshold=%.1f",
            required,
            feederLevelPct,
            feederThreshold);

  bool lowFeedPredicted =
      !isFeedSufficientIncludingQueued(required, cfg) || feederLevelPct <= feederThreshold;
  if (lowFeedPredicted && !sLowFeedPredictionActive) {
    sLowFeedPredictionActive = true;
    LOG_WARN("Low feed predicted");
    sendAlert("low_feed");
  } else if (!lowFeedPredicted && sLowFeedPredictionActive) {
    sLowFeedPredictionActive = false;
    LOG_INFO("Low feed prediction cleared");
  } else if (lowFeedPredicted) {
    LOG_DEBUG("Low feed prediction still active; duplicate alert suppressed");
  }
}

void checkSchedulesAndExecute(JsonArray schedules, JsonVariant cfg) {
  time_t now;
  time(&now);
  if (now < kScheduleValidEpochThreshold) {
    LOG_WARN("Schedule check skipped; system time not synced epoch=%ld", (long)now);
    return;
  }

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  char nowStr[6];
  strftime(nowStr, sizeof(nowStr), "%H:%M", &timeinfo);

  char slotKeyBuf[18];
  strftime(slotKeyBuf, sizeof(slotKeyBuf), "%Y-%m-%d %H:%M", &timeinfo);
  String slot = String(slotKeyBuf);
  const char* todayName = kScheduleWeekdays[timeinfo.tm_wday];

  if (slot == lastScheduleSlot) {
    LOG_DEBUG("Schedule slot already processed: %s", slot.c_str());
    return;
  }

  LOG_INFO("Checking schedules for day=%s slot=%s", todayName, nowStr);
  for (JsonVariant s : schedules) {
    if (!s["enabled"]) continue;
    JsonArrayConst days = s["days"].as<JsonArrayConst>();
    if (!scheduleIncludesDay(days, todayName)) {
      LOG_DEBUG("Schedule skipped day mismatch id=%d today=%s", s["id"] | 0, todayName);
      continue;
    }

    const char* t = s["time"] | "";
    if (strcmp(t, nowStr) == 0) {
      float amt = s["feeding_amount_kg"].as<float>();
      LOG_INFO("Schedule match day=%s time=%s amount=%.3fkg", todayName, t, amt);
      if (isFeedSufficientIncludingQueued(amt, cfg)) {
        if (dispenseFeedScheduled(amt, cfg, s)) {
          LOG_INFO("Scheduled feed queued id=%d amount=%.3f pending=%u",
                   s["id"] | 0,
                   amt,
                   getPendingFeedRequestCount());
        } else {
          LOG_WARN("Scheduled feed queue full id=%d amount=%.3f; request not executed",
                   s["id"] | 0,
                   amt);
        }
      } else {
        LOG_WARN("Insufficient unreserved feed for scheduled dispense");
        sendAlert("low_feed");
      }
    }
  }
  lastScheduleSlot = slot;
}

void processFeedNowCommand(JsonVariant cfg) {
  static uint32_t sLastDuplicateLoggedId = 0;

  JsonVariant cmd = cfg["feed_now_command"];
  if (cmd.isNull()) return;

  uint32_t commandId = getFeedNowCommandIdFromConfig(cfg);
  if (commandId == 0UL) {
    LOG_WARN("feed_now_command ignored; invalid command id (expected command_id or id)");
    return;
  }
  uint32_t lastAckId = readLastFeedNowCommandId();
  if (commandId <= lastAckId) {
    if (sLastDuplicateLoggedId != commandId) {
      LOG_DEBUG("feed_now_command duplicate id=%lu lastAck=%lu",
                (unsigned long)commandId,
                (unsigned long)lastAckId);
      sLastDuplicateLoggedId = commandId;
    }
    return;
  }
  sLastDuplicateLoggedId = 0;

  if (isFeedNowCommandQueuedOrActive(commandId)) {
    return;
  }

  float amountKg = cmd["amount_kg"] | -1.0f;
  String reason = "";

  float safeMaxKg = getMaxSingleFeedKg(cfg);

  if (amountKg <= 0.0f) {
    reason = "invalid_amount";
  } else if (amountKg > safeMaxKg) {
    reason = "amount_exceeds_limit";
  } else if (!isFeedSufficientIncludingQueued(amountKg, cfg)) {
    reason = "insufficient_feed";
  } else {
    if (dispenseFeedNow(amountKg, cfg, commandId)) {
      LOG_INFO("feed_now queued id=%lu amount=%.3f pending=%u; acknowledgement deferred until completion",
               (unsigned long)commandId,
               amountKg,
               getPendingFeedRequestCount());
      return;
    }
    LOG_WARN("feed_now deferred id=%lu amount=%.3f because feed queue is full",
             (unsigned long)commandId,
             amountKg);
    return;
  }

  bool ackOk = sendFeedNowAck(commandId, "failed", reason.c_str());
  if (!ackOk) {
    LOG_WARN("feed_now ack upload failed for id=%lu", (unsigned long)commandId);
  }

  LOG_WARN("feed_now rejected id=%lu amount=%.3f reason=%s safeMax=%.3f maxCap=%.3f",
           (unsigned long)commandId,
           amountKg,
           reason.c_str(),
           safeMaxKg,
           readRemainingKg());

  // Rejections are final and can advance the dedup watermark immediately.
  writeLastFeedNowCommandId(commandId);

  StaticJsonDocument<256> p;
  p["event"] = "feed_now";
  p["command_id"] = commandId;
  p["amount_kg"] = amountKg;
  p["status"] = "failed";
  p["reason"] = reason;
  sendLog("feed_now", p.as<JsonVariant>());

  LOG_INFO("feed_now handled id=%lu amount=%.3f status=failed reason=%s",
           (unsigned long)commandId,
           amountKg,
           reason.c_str());
}
