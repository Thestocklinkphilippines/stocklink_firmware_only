#include "sf_network.h"

#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "sf_config.h"
#include "sf_debug.h"
#include "sf_globals.h"
#include "sf_storage.h"
#include "sf_utils.h"

#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#define LOG_DEBUG(fmt, ...) do { SFMultiConsole.networkPrintf("[DBG ] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_INFO(fmt, ...)  do { SFMultiConsole.networkPrintf("[INFO] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_WARN(fmt, ...)  do { SFMultiConsole.networkPrintf("[WARN] " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(fmt, ...) do { SFMultiConsole.networkPrintf("[ERR ] " fmt "\n", ##__VA_ARGS__); } while (0)

// Config payloads can grow with schedules/runtime fields. Allocate their
// documents only while processing a completed request so TLS handshakes keep
// the extra 16 KB of internal RAM available.
static bool gOtaStarted = false;
static volatile bool gOtaInProgress = false;
static unsigned int gOtaLastProgressBucket = 101U;
static bool gOtaCore0WdtSuspended = false;
static String gOtaHostname;
static String gPendingConfigPostBody;
static bool gPendingConfigPostReady = false;

static unsigned long gServerReachabilityRetryAfterMs = 0;
static unsigned long gLastTransportWiFiRecoveryMs = 0UL;
static uint8_t gConsecutiveTransportFailures = 0U;
static const unsigned long kServerReachabilityCooldownMs = 5000UL;
static const unsigned long kServerReachabilityMaxCooldownMs = 60000UL;
static const unsigned long kTlsMemoryPressureCooldownMs = 30000UL;
static const unsigned long kTransportWiFiRecoveryCooldownMs = 60000UL;
static const uint16_t kHttpConnectTimeoutMs = 10000U;
static const uint16_t kHttpResponseTimeoutMs = 45000U;
static const uint8_t kHttpTransportMaxAttempts = 3U;
static const uint16_t kHttpTransportRetryBaseDelayMs = 2000U;
static const uint16_t kHttpMinimumRequestGapMs = 1500U;
static const uint8_t kTransportFailuresBeforeWiFiRecovery = 3U;
static const uint32_t kTlsMinimumFreeHeapBytes = 60000U;
static const uint32_t kTlsMinimumMaxAllocHeapBytes = 32000U;
static const unsigned long kConfigMaxBufferedOutboxDeferralMs = 60000UL;

enum class AsyncHttpMethod : uint8_t {
  GET,
  POST,
};

enum class AsyncHttpKind : uint8_t {
  CONFIG_GET,
  CONFIG_POST,
  LOG_POST,
  ALERT_POST,
  FEED_ACK_POST,
  SENSOR_POST,
  OUTBOX_POST,
};

static const char* asyncHttpMethodName(AsyncHttpMethod method) {
  return method == AsyncHttpMethod::POST ? "POST" : "GET";
}

static const char* asyncHttpKindName(AsyncHttpKind kind) {
  switch (kind) {
    case AsyncHttpKind::CONFIG_GET:
      return "config_download";
    case AsyncHttpKind::CONFIG_POST:
      return "config_upload";
    case AsyncHttpKind::LOG_POST:
      return "log_upload";
    case AsyncHttpKind::ALERT_POST:
      return "alert_upload";
    case AsyncHttpKind::FEED_ACK_POST:
      return "feed_now_ack";
    case AsyncHttpKind::SENSOR_POST:
      return "sensor_state_upload";
    case AsyncHttpKind::OUTBOX_POST:
      return "outbox_replay";
  }
  return "unknown";
}

struct AsyncHttpSlot {
  bool inUse = false;
  AsyncHttpMethod method = AsyncHttpMethod::GET;
  AsyncHttpKind kind = AsyncHttpKind::CONFIG_GET;
  String url;
  String payload;
  String endpoint;
  String label;
  bool critical = false;
  uint32_t sequence = 0UL;
  int statusCode = 0;
  bool ok = false;
  uint8_t attemptCount = 0U;
  uint8_t maxAttempts = kHttpTransportMaxAttempts;
  uint32_t freeHeapBefore = 0U;
  uint32_t maxAllocHeapBefore = 0U;
  uint32_t minFreeHeap = 0U;
  uint32_t workerStackFreeWords = 0U;
  unsigned long queuedAtMs = 0UL;
  String errorDetail;
  String response;
};

// Only one request is allowed in flight, so two slots are sufficient for queue handoff.
static const uint8_t kAsyncHttpSlotCount = 2U;
static AsyncHttpSlot gAsyncHttpSlots[kAsyncHttpSlotCount];
static QueueHandle_t gAsyncHttpRequestQueue = nullptr;
static QueueHandle_t gAsyncHttpResultQueue = nullptr;
static TaskHandle_t gAsyncHttpTask = nullptr;
static unsigned long gOutboxPendingSinceMs = 0UL;
static volatile bool gShutdownFlushActive = false;
static bool gShutdownFlushCriticalFailure = false;
static bool gShutdownAlertDelivered = false;
static uint8_t gShutdownFlushTransportAttempts = kHttpTransportMaxAttempts;

static void suspendCore0WatchdogForOTA() {
  if (gOtaCore0WdtSuspended) return;
  disableCore0WDT();
  gOtaCore0WdtSuspended = true;
}

static void restoreCore0WatchdogAfterOTA() {
  if (!gOtaCore0WdtSuspended) return;
  enableCore0WDT();
  gOtaCore0WdtSuspended = false;
}

static bool canProbeServerNow(unsigned long nowMs) {
  if (gShutdownFlushActive) return true;
  if (gServerReachabilityRetryAfterMs == 0) return true;
  return (long)(nowMs - gServerReachabilityRetryAfterMs) >= 0;
}

static unsigned long getServerReachabilityCooldownMs() {
  unsigned long cooldownMs = kServerReachabilityCooldownMs;
  uint8_t shifts = gConsecutiveTransportFailures > 1U ? gConsecutiveTransportFailures - 1U : 0U;
  while (shifts-- > 0U && cooldownMs < kServerReachabilityMaxCooldownMs) {
    cooldownMs *= 2UL;
  }
  return cooldownMs > kServerReachabilityMaxCooldownMs ? kServerReachabilityMaxCooldownMs : cooldownMs;
}

static void markServerUnreachable(unsigned long nowMs) {
  gServerReachabilityRetryAfterMs = nowMs + getServerReachabilityCooldownMs();
}

static void noteHttpTransportResult(int statusCode, unsigned long nowMs) {
  if (statusCode > 0 && statusCode < 500) {
    gConsecutiveTransportFailures = 0U;
    gServerReachabilityRetryAfterMs = 0;
    return;
  }

  if (gConsecutiveTransportFailures < UINT8_MAX) {
    gConsecutiveTransportFailures++;
  }
  markServerUnreachable(nowMs);

  if (statusCode >= 500) {
    return;
  }

  bool recoveryAllowed = gLastTransportWiFiRecoveryMs == 0UL ||
                         nowMs - gLastTransportWiFiRecoveryMs >= kTransportWiFiRecoveryCooldownMs;
  if (gConsecutiveTransportFailures >= kTransportFailuresBeforeWiFiRecovery && recoveryAllowed) {
    gLastTransportWiFiRecoveryMs = nowMs;
    LOG_WARN("Resetting WiFi station after repeated TCP/TLS failures");
    WiFi.disconnect(false, false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

static void clearAsyncHttpSlot(AsyncHttpSlot& slot) {
  slot.inUse = false;
  slot.url = "";
  slot.payload = "";
  slot.endpoint = "";
  slot.label = "";
  slot.critical = false;
  slot.sequence = 0UL;
  slot.statusCode = 0;
  slot.ok = false;
  slot.attemptCount = 0U;
  slot.maxAttempts = kHttpTransportMaxAttempts;
  slot.freeHeapBefore = 0U;
  slot.maxAllocHeapBefore = 0U;
  slot.minFreeHeap = 0U;
  slot.workerStackFreeWords = 0U;
  slot.queuedAtMs = 0UL;
  slot.errorDetail = "";
  slot.response = "";
}

static void asyncHttpWorkerTask(void*) {
  uint8_t slotIndex = 0U;
  unsigned long lastRequestFinishedMs = 0UL;
  WiFiClient plainClient;
  WiFiClientSecure secureClient;
  HTTPClient http;
  secureClient.setInsecure();
  secureClient.setHandshakeTimeout(45UL);
  http.setReuse(false);
  http.setConnectTimeout(kHttpConnectTimeoutMs);
  http.setTimeout(kHttpResponseTimeoutMs);

  while (true) {
    if (xQueueReceive(gAsyncHttpRequestQueue, &slotIndex, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (slotIndex >= kAsyncHttpSlotCount) continue;

    if (lastRequestFinishedMs != 0UL) {
      unsigned long elapsedMs = millis() - lastRequestFinishedMs;
      if (elapsedMs < kHttpMinimumRequestGapMs) {
        vTaskDelay(pdMS_TO_TICKS(kHttpMinimumRequestGapMs - elapsedMs));
      }
    }

    AsyncHttpSlot& slot = gAsyncHttpSlots[slotIndex];
    slot.ok = false;
    slot.statusCode = 0;
    slot.attemptCount = 0U;
    slot.freeHeapBefore = ESP.getFreeHeap();
    slot.maxAllocHeapBefore = ESP.getMaxAllocHeap();
    slot.minFreeHeap = ESP.getMinFreeHeap();
    slot.errorDetail = "";
    slot.response = "";

    for (uint8_t attempt = 1U; attempt <= slot.maxAttempts; ++attempt) {
      slot.attemptCount = attempt;
      slot.statusCode = 0;
      slot.ok = false;
      slot.errorDetail = "";
      slot.response = "";
      if (gOtaInProgress) {
        slot.statusCode = HTTPC_ERROR_CONNECTION_REFUSED;
        slot.errorDetail = "HTTP deferred: OTA active";
        break;
      }
      if (!WiFi.isConnected()) {
        slot.statusCode = HTTPC_ERROR_NOT_CONNECTED;
        slot.errorDetail = HTTPClient::errorToString(slot.statusCode);
        if (attempt >= slot.maxAttempts) break;
        vTaskDelay(pdMS_TO_TICKS((unsigned long)kHttpTransportRetryBaseDelayMs * attempt));
        continue;
      }

      uint32_t freeHeap = ESP.getFreeHeap();
      uint32_t maxAllocHeap = ESP.getMaxAllocHeap();
      if (slot.url.startsWith("https://") &&
          (freeHeap < kTlsMinimumFreeHeapBytes || maxAllocHeap < kTlsMinimumMaxAllocHeapBytes)) {
        slot.statusCode = HTTPC_ERROR_CONNECTION_REFUSED;
        slot.errorDetail = "TLS deferred: low heap free=";
        slot.errorDetail += String(freeHeap);
        slot.errorDetail += "/";
        slot.errorDetail += String(kTlsMinimumFreeHeapBytes);
        slot.errorDetail += " max_alloc=";
        slot.errorDetail += String(maxAllocHeap);
        slot.errorDetail += "/";
        slot.errorDetail += String(kTlsMinimumMaxAllocHeapBytes);
        break;
      }

      WiFiClient* transport = &plainClient;
      bool isHttps = slot.url.startsWith("https://");
      if (isHttps) {
        transport = &secureClient;
      }

      if (http.begin(*transport, slot.url)) {
        http.addHeader("Authorization", String("Token ") + AUTH_TOKEN);
        if (slot.method == AsyncHttpMethod::POST) {
          http.addHeader("Content-Type", "application/json");
          slot.statusCode = http.POST(slot.payload);
        } else {
          slot.statusCode = http.GET();
        }
        if (slot.statusCode > 0 &&
            (slot.kind == AsyncHttpKind::CONFIG_GET || slot.kind == AsyncHttpKind::CONFIG_POST)) {
          slot.response = http.getString();
        }
        slot.ok = slot.statusCode >= 200 && slot.statusCode < 300;
        if (slot.statusCode <= 0) {
          slot.errorDetail = HTTPClient::errorToString(slot.statusCode);
          if (isHttps) {
            char tlsError[128] = {0};
            int tlsCode = secureClient.lastError(tlsError, sizeof(tlsError));
            if (tlsCode != 0) {
              slot.errorDetail += " TLS(";
              slot.errorDetail += String(tlsCode);
              slot.errorDetail += "): ";
              slot.errorDetail += tlsError;
            }
          }
        }
        http.end();
      } else {
        slot.statusCode = HTTPC_ERROR_CONNECTION_REFUSED;
        slot.errorDetail = "HTTP begin failed";
      }

      // Release TLS/socket state before evaluating a retry. This is especially
      // important after failed handshakes, where mbedTLS may still own large buffers.
      http.end();
      secureClient.stop();
      plainClient.stop();

      bool tlsMemoryAllocationFailure = slot.errorDetail.indexOf("Memory allocation failed") >= 0;
      bool retryableServerFailure =
          gShutdownFlushActive && slot.critical && slot.statusCode >= 500 && slot.statusCode < 600;
      if ((slot.statusCode > 0 && !retryableServerFailure) ||
          attempt >= slot.maxAttempts) {
        break;
      }

      unsigned long retryDelayMs = (unsigned long)kHttpTransportRetryBaseDelayMs * attempt;
      if (tlsMemoryAllocationFailure && retryDelayMs < 5000UL) {
        retryDelayMs = 5000UL;
      }
      if (tlsMemoryAllocationFailure) {
        LOG_WARN("TLS allocation failed; retrying request=%s attempt=%u/%u after cleanup delay_ms=%lu",
                 asyncHttpKindName(slot.kind),
                 (unsigned int)(attempt + 1U),
                 (unsigned int)slot.maxAttempts,
                 retryDelayMs);
      }
      vTaskDelay(pdMS_TO_TICKS(retryDelayMs));
    }

    slot.minFreeHeap = ESP.getMinFreeHeap();
    slot.workerStackFreeWords = uxTaskGetStackHighWaterMark(nullptr);
    lastRequestFinishedMs = millis();
    xQueueSend(gAsyncHttpResultQueue, &slotIndex, portMAX_DELAY);
  }
}

void initAsyncHttp() {
  if (gAsyncHttpTask != nullptr) return;

  gAsyncHttpRequestQueue = xQueueCreate(kAsyncHttpSlotCount, sizeof(uint8_t));
  gAsyncHttpResultQueue = xQueueCreate(kAsyncHttpSlotCount, sizeof(uint8_t));
  if (gAsyncHttpRequestQueue == nullptr || gAsyncHttpResultQueue == nullptr) {
    if (gAsyncHttpRequestQueue != nullptr) vQueueDelete(gAsyncHttpRequestQueue);
    if (gAsyncHttpResultQueue != nullptr) vQueueDelete(gAsyncHttpResultQueue);
    gAsyncHttpRequestQueue = nullptr;
    gAsyncHttpResultQueue = nullptr;
    LOG_ERROR("Async HTTP queue initialization failed");
    return;
  }

  BaseType_t created = xTaskCreatePinnedToCore(
      asyncHttpWorkerTask,
      "sf_http",
      8192,
      nullptr,
      1,
      &gAsyncHttpTask,
      0);
  if (created != pdPASS) {
    gAsyncHttpTask = nullptr;
    vQueueDelete(gAsyncHttpRequestQueue);
    vQueueDelete(gAsyncHttpResultQueue);
    gAsyncHttpRequestQueue = nullptr;
    gAsyncHttpResultQueue = nullptr;
    LOG_ERROR("Async HTTP worker task creation failed");
    return;
  }
  LOG_INFO("Async HTTP worker ready slots=%u", (unsigned int)kAsyncHttpSlotCount);
}

static bool hasPendingAsyncHttpKind(AsyncHttpKind kind) {
  for (uint8_t i = 0; i < kAsyncHttpSlotCount; ++i) {
    if (gAsyncHttpSlots[i].inUse && gAsyncHttpSlots[i].kind == kind) return true;
  }
  return false;
}

unsigned int getAsyncHttpPendingCount() {
  unsigned int count = 0U;
  for (uint8_t i = 0; i < kAsyncHttpSlotCount; ++i) {
    if (gAsyncHttpSlots[i].inUse) count++;
  }
  return count;
}

static bool hasPendingAsyncHttpLabel(AsyncHttpKind kind, const String& label) {
  for (uint8_t i = 0; i < kAsyncHttpSlotCount; ++i) {
    if (gAsyncHttpSlots[i].inUse &&
        gAsyncHttpSlots[i].kind == kind &&
        gAsyncHttpSlots[i].label == label) {
      return true;
    }
  }
  return false;
}

static bool enqueueAsyncHttp(AsyncHttpMethod method,
                             AsyncHttpKind kind,
                             const String& url,
                             const String& payload = "",
                             const String& endpoint = "",
                             const String& label = "",
                             bool critical = false,
                             uint32_t sequence = 0UL) {
  if (gOtaInProgress) return false;
  if (gAsyncHttpTask == nullptr) initAsyncHttp();
  if (gAsyncHttpTask == nullptr || !WiFi.isConnected()) return false;
  if (!canProbeServerNow(millis())) return false;
  // A fresh HTTPS handshake needs a large contiguous heap block on ESP32.
  // Keep exactly one request resident; callers persist/coalesce overflow.
  if (getAsyncHttpPendingCount() > 0U) return false;

  for (uint8_t i = 0; i < kAsyncHttpSlotCount; ++i) {
    AsyncHttpSlot& slot = gAsyncHttpSlots[i];
    if (slot.inUse) continue;

    slot.inUse = true;
    slot.method = method;
    slot.kind = kind;
    slot.url = url;
    slot.payload = payload;
    slot.endpoint = endpoint;
    slot.label = label;
    slot.critical = critical;
    slot.sequence = sequence;
    slot.statusCode = 0;
    slot.ok = false;
    slot.attemptCount = 0U;
    slot.maxAttempts = critical && gShutdownFlushActive
                           ? gShutdownFlushTransportAttempts
                           : kHttpTransportMaxAttempts;
    slot.freeHeapBefore = 0U;
    slot.maxAllocHeapBefore = 0U;
    slot.minFreeHeap = 0U;
    slot.workerStackFreeWords = 0U;
    slot.queuedAtMs = millis();
    slot.errorDetail = "";
    slot.response = "";

    uint8_t slotIndex = i;
    BaseType_t queued = xQueueSendToBack(gAsyncHttpRequestQueue, &slotIndex, 0);
    if (queued == pdTRUE) {
      LOG_DEBUG("HTTP queued request=%s method=%s endpoint=%s label=%s payload_bytes=%u seq=%lu",
                asyncHttpKindName(kind),
                asyncHttpMethodName(method),
                endpoint.length() > 0 ? endpoint.c_str() : "(not-set)",
                label.length() > 0 ? label.c_str() : "-",
                (unsigned int)payload.length(),
                (unsigned long)sequence);
      return true;
    }

    clearAsyncHttpSlot(slot);
    return false;
  }

  LOG_WARN("HTTP queue full request=%s method=%s endpoint=%s label=%s",
           asyncHttpKindName(kind),
           asyncHttpMethodName(method),
           endpoint.length() > 0 ? endpoint.c_str() : "(not-set)",
           label.length() > 0 ? label.c_str() : "-");
  return false;
}

static bool enqueuePendingConfigPost() {
  if (!gPendingConfigPostReady || gPendingConfigPostBody.length() == 0) return false;

  String url = String(SERVER_BASE) + "/device/" + DEVICE_ID + "/config/";
  bool queued = enqueueAsyncHttp(
      AsyncHttpMethod::POST,
      AsyncHttpKind::CONFIG_POST,
      url,
      gPendingConfigPostBody,
      String("/device/") + DEVICE_ID + "/config/",
      "config_post",
      true);

  if (queued) {
    LOG_INFO("Config POST queued bytes=%u", (unsigned int)gPendingConfigPostBody.length());
    gPendingConfigPostBody = "";
    gPendingConfigPostReady = false;
  }
  return queued;
}

static void stageConfigPost(const String& postBody) {
  gPendingConfigPostBody = postBody;
  gPendingConfigPostReady = postBody.length() > 0;
  LOG_INFO("Config POST staged bytes=%u", (unsigned int)gPendingConfigPostBody.length());
}

static const unsigned int kOutboxMaxEvents = 20U;

static bool isQueuedLogImportant(const char* type) {
  if (type == nullptr) return false;
  return strcmp(type, "feeding") == 0 || strcmp(type, "watering") == 0 || strcmp(type, "power") == 0 ||
         strcmp(type, "feed_now") == 0 || strcmp(type, "low_feed") == 0 || strcmp(type, "low_water") == 0 ||
         strcmp(type, "shutdown") == 0;
}

static bool isSystemConsoleCriticalType(const char* type) {
  if (type == nullptr) return false;
  return strcmp(type, "feeding") == 0 || strcmp(type, "watering") == 0 || strcmp(type, "power") == 0 ||
         strcmp(type, "feed_now") == 0 || strcmp(type, "low_feed") == 0 || strcmp(type, "low_water") == 0 ||
         strcmp(type, "shutdown") == 0;
}

static void emitSystemEventForLog(const char* type, JsonVariant payload) {
  if (!isSystemConsoleCriticalType(type)) return;

  if (strcmp(type, "power") == 0) {
    const char* event = payload["event"] | "update";
    LOG_SYSTEM_EVENT("power_%s", event);
    return;
  }

  if (strcmp(type, "feeding") == 0) {
    float amountKg = payload["amount_kg"] | -1.0f;
    float remainingKg = payload["remaining_kg"] | -1.0f;
    const char* trigger = payload["trigger"] | "";
    const char* source = payload["source"] | "";
    const char* eventName = "feeding";

    if (strcmp(trigger, "feed_now") == 0) {
      eventName = "feed_now";
    } else if (strcmp(trigger, "manual") == 0) {
      eventName = "manual_feed";
    } else if (strcmp(trigger, "schedule") == 0) {
      eventName = "scheduled_feed";
    }

    if (amountKg >= 0.0f && remainingKg >= 0.0f) {
      uint32_t commandId = payload["command_id"] | 0UL;
      const char* scheduleName = payload["schedule_name"] | "";
      if (commandId > 0UL) {
        LOG_SYSTEM_EVENT("%s command=%lu amount=%.3fkg remaining=%.3fkg",
                         eventName,
                         (unsigned long)commandId,
                         amountKg,
                         remainingKg);
      } else if (scheduleName[0] != '\0') {
        LOG_SYSTEM_EVENT("%s schedule=%s amount=%.3fkg remaining=%.3fkg",
                         eventName,
                         scheduleName,
                         amountKg,
                         remainingKg);
      } else if (source[0] != '\0') {
        LOG_SYSTEM_EVENT("%s source=%s amount=%.3fkg remaining=%.3fkg",
                         eventName,
                         source,
                         amountKg,
                         remainingKg);
      } else {
        LOG_SYSTEM_EVENT("%s amount=%.3fkg remaining=%.3fkg", eventName, amountKg, remainingKg);
      }
    } else {
      LOG_SYSTEM_EVENT("%s", eventName);
    }
    return;
  }

  if (strcmp(type, "feed_now") == 0) {
    const char* statusValue = payload["status"] | "event";
    uint32_t commandId = payload["command_id"] | 0UL;
    float amountKg = payload["amount_kg"] | -1.0f;
    if (commandId > 0UL && amountKg >= 0.0f) {
      LOG_SYSTEM_EVENT("feed_now_%s command=%lu amount=%.3fkg",
                       statusValue,
                       (unsigned long)commandId,
                       amountKg);
    } else if (commandId > 0UL) {
      LOG_SYSTEM_EVENT("feed_now_%s command=%lu", statusValue, (unsigned long)commandId);
    } else {
      LOG_SYSTEM_EVENT("feed_now_%s", statusValue);
    }
    return;
  }

  if (strcmp(type, "watering") == 0) {
    const char* event = payload["event"] | "update";
    float waterLevelPct = payload["water_level_pct"] | -1.0f;
    if (waterLevelPct >= 0.0f) {
      LOG_SYSTEM_EVENT("watering_%s level=%.1f%%", event, waterLevelPct);
    } else {
      LOG_SYSTEM_EVENT("watering_%s", event);
    }
    return;
  }

  LOG_SYSTEM_EVENT("%s", type);
}

static bool loadOutboxDoc(DynamicJsonDocument& outboxDoc) {
  String raw = loadEventOutbox();
  outboxDoc.clear();
  DeserializationError err = deserializeJson(outboxDoc, raw);
  if (err) {
    outboxDoc.clear();
    outboxDoc["next_seq"] = readEventSequence();
    outboxDoc.createNestedArray("events");
    return false;
  }

  if (!outboxDoc.containsKey("next_seq")) {
    outboxDoc["next_seq"] = readEventSequence();
  }
  if (!outboxDoc.containsKey("events") || !outboxDoc["events"].is<JsonArray>()) {
    outboxDoc.createNestedArray("events");
  }
  return true;
}

static void updateOutboxState(DynamicJsonDocument& outboxDoc) {
  JsonArray events = outboxDoc["events"].as<JsonArray>();
  state.bufferedEventCount = events.isNull() ? 0U : (unsigned int)events.size();
  if (state.bufferedEventCount == 0U) {
    gOutboxPendingSinceMs = 0UL;
  } else if (gOutboxPendingSinceMs == 0UL) {
    gOutboxPendingSinceMs = millis();
  }
}

static void saveOutboxDoc(DynamicJsonDocument& outboxDoc) {
  String out;
  serializeJson(outboxDoc, out);
  saveEventOutbox(out);
  uint32_t nextSeq = outboxDoc["next_seq"] | readEventSequence();
  writeEventSequence(nextSeq);
  updateOutboxState(outboxDoc);
}

bool queueBufferedRequest(const String& endpoint, const String& body, const char* kind, bool critical) {
  if (endpoint.length() == 0 || body.length() == 0) return false;

  DynamicJsonDocument outboxDoc(8192);
  loadOutboxDoc(outboxDoc);
  JsonArray events = outboxDoc["events"].as<JsonArray>();

  if (strcmp(kind, "sensor_state") == 0) {
    for (JsonVariant ev : events) {
      if (strcmp((const char*)(ev["kind"] | ""), "sensor_state") == 0) {
        ev["endpoint"] = endpoint;
        ev["body"] = body;
        ev["ts"] = getUtcIsoNow();
        saveOutboxDoc(outboxDoc);
        LOG_WARN("Buffered sensor state replaced existing queued sample");
        return true;
      }
    }
  }

  if (events.size() >= kOutboxMaxEvents) {
    int dropIndex = -1;
    for (int i = 0; i < (int)events.size(); i++) {
      const char* evKind = events[i]["kind"] | "";
      bool evCritical = strcmp(evKind, "sensor_state") != 0 && strcmp(evKind, "heartbeat") != 0;
      if (!evCritical) {
        dropIndex = i;
        break;
      }
    }
    if (dropIndex >= 0) {
      events.remove(dropIndex);
    } else if (!critical) {
      LOG_WARN("Outbox full; dropping non-critical %s event", kind);
      return false;
    } else {
      events.remove(0);
      LOG_WARN("Outbox full; dropped oldest event to preserve critical %s event", kind);
    }
  }

  JsonObject ev = events.createNestedObject();
  uint32_t seq = outboxDoc["next_seq"] | readEventSequence();
  if (seq == 0UL) seq = 1UL;
  ev["seq"] = seq;
  ev["event_id"] = String(DEVICE_ID) + "-" + String(seq);
  ev["kind"] = kind;
  ev["endpoint"] = endpoint;
  ev["body"] = body;
  ev["ts"] = getUtcIsoNow();
  ev["critical"] = critical;
  outboxDoc["next_seq"] = seq + 1UL;

  saveOutboxDoc(outboxDoc);
  LOG_INFO("Buffered outbound %s event seq=%lu queued=%u", kind, (unsigned long)seq, state.bufferedEventCount);
  return true;
}

static bool removeBufferedOutboxEvent(uint32_t sequence) {
  DynamicJsonDocument outboxDoc(8192);
  loadOutboxDoc(outboxDoc);
  JsonArray events = outboxDoc["events"].as<JsonArray>();
  if (events.isNull()) return false;

  for (int i = 0; i < (int)events.size(); ++i) {
    if ((events[i]["seq"] | 0UL) != sequence) continue;
    events.remove(i);
    saveOutboxDoc(outboxDoc);
    LOG_INFO("Flushed queued event seq=%lu remaining=%u",
             (unsigned long)sequence,
             state.bufferedEventCount);
    return true;
  }
  return false;
}

static int selectBufferedOutboxEventIndex(JsonArray events) {
  int firstCriticalIndex = -1;
  int firstValidIndex = -1;

  for (int i = 0; i < (int)events.size(); ++i) {
    String endpoint = events[i]["endpoint"] | "";
    String body = events[i]["body"] | "";
    uint32_t sequence = events[i]["seq"] | 0UL;
    if (endpoint.length() == 0 || body.length() == 0 || sequence == 0UL) continue;

    if (firstValidIndex < 0) firstValidIndex = i;

    const char* kind = events[i]["kind"] | "";
    if (strcmp(kind, "shutdown_alert") == 0) {
      return i;
    }
    if (strcmp(kind, "feed_now_ack") == 0) {
      return i;
    }
    if (firstCriticalIndex < 0 && (events[i]["critical"] | false)) {
      firstCriticalIndex = i;
    }
  }

  return firstCriticalIndex >= 0 ? firstCriticalIndex : firstValidIndex;
}

static void preserveBatteryConfigFields(JsonVariant envelope, JsonVariant localCfg) {
  if (envelope.isNull() || localCfg.isNull()) return;

  JsonVariant cfg = envelope["config"];
  if (cfg.isNull()) return;

  const char* batteryKeys[] = {
      "battery_sense_enabled",
      "battery_adc_pin",
      "battery_divider_top_ohms",
      "battery_divider_bottom_ohms",
      "battery_adc_reference_v",
      "battery_adc_gain_correction",
      "low_battery_shutdown_v",
      "low_battery_shutdown_resolve_v",
  };
  for (const char* key : batteryKeys) {
    if (!cfg.containsKey(key) && localCfg.containsKey(key)) {
      cfg[key] = localCfg[key];
    }
  }
}

static void preserveKeypadConfigFields(JsonVariant envelope, JsonVariant localCfg) {
  if (envelope.isNull() || localCfg.isNull()) return;

  JsonVariant cfg = envelope["config"];
  if (cfg.isNull()) return;

  if (!cfg.containsKey("keypad_input_enabled") && localCfg.containsKey("keypad_input_enabled")) {
    cfg["keypad_input_enabled"] = localCfg["keypad_input_enabled"];
  }
  // Calibration ranges are hardware-local and must never enter config sync.
  cfg.remove("keypad_calibration");
}

static void preserveDerivedFeedLimitFields(JsonVariant envelope, JsonVariant localCfg) {
  if (envelope.isNull() || localCfg.isNull()) return;

  JsonVariant cfg = envelope["config"];
  if (cfg.isNull()) return;

  if (!cfg.containsKey("max_single_feed_kg") && localCfg.containsKey("max_single_feed_kg")) {
    cfg["max_single_feed_kg"] = localCfg["max_single_feed_kg"];
  }
}

static void preserveManualFeedTrackingFields(JsonVariant envelope, JsonVariant localCfg) {
  if (envelope.isNull() || localCfg.isNull()) return;

  JsonVariant cfg = envelope["config"];
  if (cfg.isNull()) return;

  if (!cfg.containsKey("manual_feed_snapshot_id") && localCfg.containsKey("manual_feed_snapshot_id")) {
    cfg["manual_feed_snapshot_id"] = localCfg["manual_feed_snapshot_id"];
  }
  if (!cfg.containsKey("last_feed_now_command_id") && localCfg.containsKey("last_feed_now_command_id")) {
    cfg["last_feed_now_command_id"] = localCfg["last_feed_now_command_id"];
  }
}

static void preserveRuntimeFeedNowCommand(JsonVariant envelope, JsonVariant localCfg) {
  if (envelope.isNull() || localCfg.isNull()) return;

  JsonVariant cfg = envelope["config"];
  if (cfg.isNull()) return;

  if (!cfg.containsKey("feed_now_command") && localCfg.containsKey("feed_now_command")) {
    cfg["feed_now_command"] = localCfg["feed_now_command"];
  }
}

static bool preserveDailyFeedTotalFields(JsonVariant envelope, JsonVariant localCfg) {
  if (envelope.isNull() || localCfg.isNull()) return false;

  JsonVariant cfg = envelope["config"];
  if (cfg.isNull()) return false;

  bool hasLocalTotal = localCfg.containsKey("total_feeds_today_kg");
  bool hasLocalDate = localCfg.containsKey("total_feeds_today_date");
  bool hasServerTotal = cfg.containsKey("total_feeds_today_kg");
  bool hasServerDate = cfg.containsKey("total_feeds_today_date");
  if (!hasLocalTotal && !hasLocalDate) return false;

  bool changed = false;
  float localTotal = localCfg["total_feeds_today_kg"] | 0.0f;
  float serverTotal = cfg["total_feeds_today_kg"] | 0.0f;
  const char* localDate = localCfg["total_feeds_today_date"] | "";
  const char* serverDate = cfg["total_feeds_today_date"] | "";

  if (hasLocalTotal && hasServerTotal && hasLocalDate && hasServerDate && strcmp(localDate, serverDate) == 0) {
    if (localTotal > serverTotal + 0.0005f) {
      cfg["total_feeds_today_kg"] = localTotal;
      cfg["total_feeds_today_date"] = localDate;
      changed = true;
    }
    return changed;
  }

  if (!hasServerTotal || !hasServerDate) {
    if (hasLocalTotal) cfg["total_feeds_today_kg"] = localTotal;
    if (hasLocalDate) cfg["total_feeds_today_date"] = localDate;
    return true;
  }

  return false;
}

static void preserveServerOwnedGrainFields(JsonVariant targetCfg, JsonVariant serverEnvelope) {
  if (targetCfg.isNull() || serverEnvelope.isNull()) return;

  JsonVariant serverCfg = serverEnvelope["config"];
  if (serverCfg.isNull()) return;

  if (serverCfg.containsKey("grain_types")) {
    targetCfg["grain_types"] = serverCfg["grain_types"];
  }
}

static void preserveLocalGrainSelectionFields(JsonVariant envelope, JsonVariant localCfg) {
  if (envelope.isNull() || localCfg.isNull()) return;

  JsonVariant cfg = envelope["config"];
  if (cfg.isNull()) return;

  if (!cfg.containsKey("grain_type_index") && !cfg.containsKey("grain_type") && localCfg.containsKey("grain_type_index")) {
    cfg["grain_type_index"] = localCfg["grain_type_index"];
  }
  if (!cfg.containsKey("grain_type") && !cfg.containsKey("grain_type_index") && localCfg.containsKey("grain_type")) {
    cfg["grain_type"] = localCfg["grain_type"];
  }
  if (!cfg.containsKey("feed_ms_per_kg") && !cfg.containsKey("grain_type_index") && localCfg.containsKey("feed_ms_per_kg")) {
    cfg["feed_ms_per_kg"] = localCfg["feed_ms_per_kg"];
  }
  if (!cfg.containsKey("grain_types") && localCfg.containsKey("grain_types")) {
    cfg["grain_types"] = localCfg["grain_types"];
  }
}

static void normalizeGrainSelectionFields(JsonVariant cfg) {
  if (cfg.isNull()) return;

  int selectedIndex = -1;
  if (cfg.containsKey("grain_type_index")) {
    selectedIndex = getSelectedGrainTypeIndex(cfg);
  } else if (cfg.containsKey("grain_type")) {
    selectedIndex = findGrainTypeIndex(cfg, cfg["grain_type"] | DEFAULT_GRAIN_TYPE);
  }

  if (selectedIndex < 0) selectedIndex = 0;
  cfg["grain_type_index"] = selectedIndex;
  cfg["grain_type"] = getGrainTypeNameByIndex(cfg, selectedIndex);
  cfg["feed_ms_per_kg"] = getGrainTypeMsPerKgByIndex(cfg, selectedIndex);
}

static bool applyServerConfigEnvelope(JsonVariant envelope, const char* fallbackUpdatedBy) {
  if (envelope.isNull()) return false;

  JsonVariant cfg = envelope["config"];
  if (cfg.isNull()) return false;

  String localStr = loadLocalConfig();
  DynamicJsonDocument localDoc(8192);
  DeserializationError localErr = deserializeJson(localDoc, localStr);
  if (!localErr) {
    preserveBatteryConfigFields(envelope, localDoc.as<JsonVariant>());
    preserveKeypadConfigFields(envelope, localDoc.as<JsonVariant>());
    preserveDerivedFeedLimitFields(envelope, localDoc.as<JsonVariant>());
    preserveManualFeedTrackingFields(envelope, localDoc.as<JsonVariant>());
    preserveRuntimeFeedNowCommand(envelope, localDoc.as<JsonVariant>());
    preserveDailyFeedTotalFields(envelope, localDoc.as<JsonVariant>());
    preserveLocalGrainSelectionFields(envelope, localDoc.as<JsonVariant>());
  }

  normalizeGrainSelectionFields(cfg);


  const char* rootTs = envelope["last_updated"] | "";
  if (strlen(rootTs) > 0) {
    cfg["last_updated"] = rootTs;
  }
  cfg["updated_by"] = envelope["updated_by"] | fallbackUpdatedBy;

  String out;
  serializeJson(cfg, out);
  if (out.length() == 0) return false;

  saveLocalConfig(out);
  clearLocalConfigDirty();
  return true;
}

static bool syncRuntimeFeedNowCommand(JsonVariant envelope, JsonVariant localCfg) {
  if (envelope.isNull() || localCfg.isNull()) return false;

  JsonVariant serverCfg = envelope["config"];
  if (serverCfg.isNull() || !serverCfg.containsKey("feed_now_command")) return false;

  JsonVariant serverCommand = serverCfg["feed_now_command"];
  uint32_t lastHandledId = readLastFeedNowCommandId();
  uint32_t serverCommandId = getFeedNowCommandIdFromConfig(serverCfg);
  uint32_t localCommandId = getFeedNowCommandIdFromConfig(localCfg);
  bool changed = false;

  if (serverCommand.isNull() || serverCommandId == 0UL || serverCommandId <= lastHandledId) {
    if (localCfg.containsKey("feed_now_command")) {
      localCfg.remove("feed_now_command");
      changed = true;
    }
  } else if (localCommandId != serverCommandId) {
    localCfg["feed_now_command"] = serverCommand;
    changed = true;
  }

  if (!changed) return false;

  String out;
  serializeJson(localCfg, out);
  saveLocalConfig(out);
  LOG_INFO("Synced runtime feed_now command serverId=%lu lastHandled=%lu",
           (unsigned long)serverCommandId,
           (unsigned long)lastHandledId);
  return true;
}

void sendLog(const char* type, JsonVariant payload) {
  if (type != nullptr && strcmp(type, "heartbeat") == 0 &&
      (getAsyncHttpPendingCount() > 0U ||
       hasPendingAsyncHttpLabel(AsyncHttpKind::LOG_POST, "heartbeat"))) {
    return;
  }

  String endpoint = String("/device/") + DEVICE_ID + "/logs/";
  String url = String(SERVER_BASE) + endpoint;
  StaticJsonDocument<1024> doc;
  doc["log_type"] = type;
  doc["payload"] = payload;
  doc["timestamp"] = getUtcIsoNow();  // Add current UTC timestamp
  emitSystemEventForLog(type, payload);
  String body;
  serializeJson(doc, body);
  if (type != nullptr && (strcmp(type, "feeding") == 0 || strcmp(type, "feed_now") == 0)) {
    LOG_DEBUG("sendLog payload type=%s body=%s", type, body.c_str());
  }
  bool important = isQueuedLogImportant(type);
  if (important) {
    queueBufferedRequest(endpoint, body, type, true);
    return;
  }

  bool queued = enqueueAsyncHttp(
      AsyncHttpMethod::POST, AsyncHttpKind::LOG_POST, url, body, endpoint, type, false);
  if (!queued && strcmp(type, "heartbeat") != 0) {
    queueBufferedRequest(endpoint, body, type, false);
  }
}

void sendAlert(const char* alertType) {
  String endpoint = String("/device/") + DEVICE_ID + "/alerts/";
  StaticJsonDocument<256> doc;
  doc["alert_type"] = alertType;
  LOG_SYSTEM_ALERT("%s", alertType);
  String body;
  serializeJson(doc, body);
  const char* kind =
      alertType != nullptr && strcmp(alertType, "low_battery_shutdown") == 0
          ? "shutdown_alert"
          : "alert";
  queueBufferedRequest(endpoint, body, kind, true);
}

bool sendFeedNowAck(uint32_t commandId, const char* status, const char* reason) {
  String endpoint = String("/device/") + DEVICE_ID + "/feed-now/" + String(commandId) + "/ack/";
  StaticJsonDocument<256> doc;
  doc["status"] = status;
  if (reason != nullptr && strlen(reason) > 0) {
    doc["reason"] = reason;
  }

  String body;
  serializeJson(doc, body);
  return queueBufferedRequest(endpoint, body, "feed_now_ack", true);
}

bool sendSensorState(const String& body) {
  String endpoint = String("/device/") + DEVICE_ID + "/sensor-state/";
  if (getAsyncHttpPendingCount() > 0U) {
    return queueBufferedRequest(endpoint, body, "sensor_state", false);
  }

  String url = String(SERVER_BASE) + endpoint;
  bool queued = enqueueAsyncHttp(
      AsyncHttpMethod::POST, AsyncHttpKind::SENSOR_POST, url, body, endpoint, "sensor_state", false);
  if (!queued) {
    return queueBufferedRequest(endpoint, body, "sensor_state", false);
  }
  return true;
}

static void processConfigGetResponse(const String& body) {
  if (body.length() == 0) {
    LOG_WARN("Config GET returned an empty body; keeping local config and retrying later");
    return;
  }

  DynamicJsonDocument serverDoc(8192);
  DeserializationError err = deserializeJson(serverDoc, body);
  if (err) {
    LOG_ERROR("Invalid config JSON from server: %s (bytes=%u)", err.c_str(), (unsigned int)body.length());
    return;
  }

  const char* serverTs = serverDoc["last_updated"] | "";
  if (strlen(serverTs) == 0) {
    serverTs = serverDoc["config"]["last_updated"] | "";
  }

  String localStr = loadLocalConfig();
  DynamicJsonDocument localDoc(8192);
  DeserializationError localErr = deserializeJson(localDoc, localStr);
  if (localErr) {
    LOG_ERROR("Local config parse failed before sync compare: %s (bytes=%u)",
              localErr.c_str(),
              (unsigned int)localStr.length());
    return;
  }

  syncRuntimeFeedNowCommand(serverDoc.as<JsonVariant>(), localDoc.as<JsonVariant>());

  const char* localTs = localDoc["last_updated"] | "";

  time_t sTs = strlen(serverTs) > 0 ? parseIsoUtc(serverTs) : 0;
  time_t lTs = strlen(localTs) > 0 ? parseIsoUtc(localTs) : 0;
  bool localDirty = isLocalConfigDirty();

  LOG_INFO("LWW compare server=%ld local=%ld dirty=%d", (long)sTs, (long)lTs, localDirty ? 1 : 0);

  if (!localDirty) {
    if (sTs != lTs) {
      if (applyServerConfigEnvelope(serverDoc.as<JsonVariant>(), "server")) {
        if (lTs > sTs) {
          LOG_WARN("Local config newer but not dirty; applied server config to avoid OTA/default overwrite");
        } else {
          LOG_INFO("Applied server config to local");
        }
      } else {
        LOG_WARN("Server config envelope missing fields; skipped local apply");
      }
    } else {
      LOG_DEBUG("Config in sync; no update needed");
    }
  } else if (sTs > lTs) {
    if (applyServerConfigEnvelope(serverDoc.as<JsonVariant>(), "server")) {
      LOG_INFO("Applied server config to local");
    } else {
      LOG_WARN("Server config envelope missing fields; skipped local apply");
    }
  } else if (lTs > sTs) {
    // Config uploads are uncommon. Allocate this large envelope only while it
    // is being built so its RAM is normally available for TLS handshakes.
    DynamicJsonDocument postDoc(12288);
    postDoc["config"] = localDoc.as<JsonVariant>();
    postDoc["config"].remove("feed_now_command");
    postDoc["config"].remove("keypad_calibration");
    preserveServerOwnedGrainFields(postDoc["config"].as<JsonVariant>(), serverDoc.as<JsonVariant>());
    postDoc["last_updated"] = localDoc["last_updated"] | getUtcIsoNow();
    postDoc["updated_by"] = "esp32";
    if (postDoc.overflowed()) {
      LOG_ERROR("Config POST envelope overflowed; cfgBytes=%u", (unsigned int)localStr.length());
      return;
    }

    String postBody;
    size_t postBytes = serializeJson(postDoc, postBody);
    LOG_INFO("Config POST envelope bytes=%u localTs=%s",
             (unsigned int)postBytes,
             (const char*)(postDoc["last_updated"] | ""));
    if (postBytes == 0 || postBody.length() == 0) {
      LOG_ERROR("Config POST envelope serialization failed");
      return;
    }

    stageConfigPost(postBody);
  } else {
    LOG_DEBUG("Config in sync; no update needed");
  }
}

static void processConfigPostResponse(const AsyncHttpSlot& slot) {
  LOG_INFO("Config POST completed ok=%d status=%d", slot.ok ? 1 : 0, slot.statusCode);
  DynamicJsonDocument responseDoc(8192);
  DeserializationError responseErr = deserializeJson(responseDoc, slot.response);

  if (slot.ok) {
    if (!responseErr) {
      if (applyServerConfigEnvelope(responseDoc.as<JsonVariant>(), "server")) {
        LOG_INFO("Applied server canonical config from POST response");
      } else {
        LOG_DEBUG("POST success response has no config envelope; keeping local copy");
      }
    } else if (slot.response.length() > 0) {
      LOG_WARN("POST success response parse failed: %s", responseErr.c_str());
    }
    return;
  }

  if (slot.statusCode == HTTP_CODE_CONFLICT && !responseErr && responseDoc.containsKey("server_config")) {
    JsonVariant serverCopy = responseDoc["server_config"];
    if (applyServerConfigEnvelope(serverCopy, "server")) {
      LOG_WARN("Recovered from conflict by applying server_config to local cache");
    }
  }
}

void syncWithServer() {
  if (!WiFi.isConnected()) return;
  unsigned long nowMs = millis();
  if (!canProbeServerNow(nowMs)) return;
  if (getAsyncHttpPendingCount() > 0U) {
    return;
  }

  if (enqueuePendingConfigPost()) {
    return;
  }

  if (state.bufferedEventCount > 0U) {
    if (gOutboxPendingSinceMs == 0UL) {
      gOutboxPendingSinceMs = nowMs;
    }
    unsigned long pendingAgeMs = nowMs - gOutboxPendingSinceMs;
    if (pendingAgeMs < kConfigMaxBufferedOutboxDeferralMs) {
      LOG_DEBUG("Config GET skipped; buffered events pending=%u age_ms=%lu",
                state.bufferedEventCount,
                pendingAgeMs);
      return;
    }
    LOG_WARN("Config GET allowed despite buffered events pending=%u age_ms=%lu",
             state.bufferedEventCount,
             pendingAgeMs);
    gOutboxPendingSinceMs = nowMs;
  }

  String endpoint = String("/device/") + DEVICE_ID + "/config/";
  String url = String(SERVER_BASE) + endpoint;
  if (!enqueueAsyncHttp(
          AsyncHttpMethod::GET, AsyncHttpKind::CONFIG_GET, url, "", endpoint, "config", false)) {
    LOG_DEBUG("Config GET not queued");
  }
}

void serviceAsyncHttp() {
  if (gAsyncHttpResultQueue == nullptr) return;

  uint8_t slotIndex = 0U;
  while (xQueueReceive(gAsyncHttpResultQueue, &slotIndex, 0) == pdTRUE) {
    if (slotIndex >= kAsyncHttpSlotCount) continue;
    AsyncHttpSlot& slot = gAsyncHttpSlots[slotIndex];

    unsigned long durationMs = slot.queuedAtMs > 0UL ? millis() - slot.queuedAtMs : 0UL;
    LOG_DEBUG("HTTP completed request=%s method=%s endpoint=%s label=%s ok=%d status=%d response_bytes=%u duration_ms=%lu seq=%lu",
              asyncHttpKindName(slot.kind),
              asyncHttpMethodName(slot.method),
              slot.endpoint.length() > 0 ? slot.endpoint.c_str() : "(not-set)",
              slot.label.length() > 0 ? slot.label.c_str() : "-",
              slot.ok ? 1 : 0,
              slot.statusCode,
              (unsigned int)slot.response.length(),
              durationMs,
              (unsigned long)slot.sequence);

    if (slot.statusCode <= 0) {
      LOG_WARN("HTTP transport failure request=%s method=%s endpoint=%s label=%s status=%d error=%s attempts=%u wifi=%d rssi=%d free_before=%u max_before=%u free_now=%u max_now=%u min_free=%u worker_stack_free_words=%u",
               asyncHttpKindName(slot.kind),
               asyncHttpMethodName(slot.method),
               slot.endpoint.length() > 0 ? slot.endpoint.c_str() : "(not-set)",
               slot.label.length() > 0 ? slot.label.c_str() : "-",
               slot.statusCode,
               slot.errorDetail.length() > 0 ? slot.errorDetail.c_str() : "unknown",
               (unsigned int)slot.attemptCount,
               WiFi.isConnected() ? 1 : 0,
               WiFi.isConnected() ? WiFi.RSSI() : 0,
               (unsigned int)slot.freeHeapBefore,
               (unsigned int)slot.maxAllocHeapBefore,
               (unsigned int)ESP.getFreeHeap(),
               (unsigned int)ESP.getMaxAllocHeap(),
               (unsigned int)slot.minFreeHeap,
               (unsigned int)slot.workerStackFreeWords);
    }
    bool memoryPressureFailure =
        slot.errorDetail.startsWith("TLS deferred:") ||
        slot.errorDetail.indexOf("Memory allocation failed") >= 0;
    bool serverErrorFailure = slot.statusCode >= 500 && slot.statusCode < 600;
    if (memoryPressureFailure) {
      gServerReachabilityRetryAfterMs = millis() + kTlsMemoryPressureCooldownMs;
    } else {
      noteHttpTransportResult(slot.statusCode, millis());
    }
    if (slot.statusCode <= 0 || serverErrorFailure) {
      LOG_WARN("Server retry paused failures=%u cooldown_ms=%lu memory_pressure=%d server_error=%d free_heap=%u max_alloc_heap=%u",
               (unsigned int)gConsecutiveTransportFailures,
               memoryPressureFailure ? kTlsMemoryPressureCooldownMs : getServerReachabilityCooldownMs(),
               memoryPressureFailure ? 1 : 0,
               serverErrorFailure ? 1 : 0,
               (unsigned int)ESP.getFreeHeap(),
               (unsigned int)ESP.getMaxAllocHeap());
    }

    switch (slot.kind) {
      case AsyncHttpKind::CONFIG_GET:
        if (slot.ok) {
          processConfigGetResponse(slot.response);
        } else {
          LOG_WARN("Config GET failed status=%d", slot.statusCode);
        }
        break;

      case AsyncHttpKind::CONFIG_POST:
        processConfigPostResponse(slot);
        break;

      case AsyncHttpKind::LOG_POST:
        if (!slot.ok && slot.label != "heartbeat") {
          queueBufferedRequest(slot.endpoint, slot.payload, slot.label.c_str(), slot.critical);
        }
        break;

      case AsyncHttpKind::ALERT_POST:
        if (!slot.ok) {
          queueBufferedRequest(slot.endpoint, slot.payload, "alert", true);
        }
        break;

      case AsyncHttpKind::FEED_ACK_POST:
        if (!slot.ok) {
          queueBufferedRequest(slot.endpoint, slot.payload, "feed_now_ack", true);
        }
        break;

      case AsyncHttpKind::SENSOR_POST:
        if (!slot.ok) {
          queueBufferedRequest(slot.endpoint, slot.payload, "sensor_state", false);
        }
        break;

      case AsyncHttpKind::OUTBOX_POST:
        if (slot.ok) {
          removeBufferedOutboxEvent(slot.sequence);
          if (gShutdownFlushActive && slot.label == "shutdown_alert") {
            gShutdownAlertDelivered = true;
          }
        } else {
          LOG_WARN("Queued event send failed kind=%s status=%d", slot.label.c_str(), slot.statusCode);
          bool shutdownAlertFailed = slot.label == "shutdown_alert";
          if (gShutdownFlushActive &&
              slot.critical &&
              (shutdownAlertFailed || gShutdownAlertDelivered)) {
            gShutdownFlushCriticalFailure = true;
            LOG_ERROR("Shutdown flush critical event failed kind=%s seq=%lu attempts=%u status=%d",
                      slot.label.c_str(),
                      (unsigned long)slot.sequence,
                      (unsigned int)slot.attemptCount,
                      slot.statusCode);
          }
        }
        break;
    }

    clearAsyncHttpSlot(slot);
    enqueuePendingConfigPost();
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  if (gOtaHostname.length() == 0) {
    gOtaHostname = String("smart-feeder-") + DEVICE_ID;
  }
  WiFi.setHostname(gOtaHostname.c_str());
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  LOG_INFO("Connecting WiFi SSID=%s", WIFI_SSID);
  if (WiFi.status() == WL_CONNECTED) {
    LOG_INFO("WiFi connected IP=%s", WiFi.localIP().toString().c_str());
  } else {
    LOG_WARN("WiFi connect initiated; continuing without blocking");
  }
}

void serviceBufferedOutbox() {
  if (gOtaInProgress) return;
  if (!WiFi.isConnected() || !canProbeServerNow(millis())) return;
  if (getAsyncHttpPendingCount() > 0U) return;
  if (enqueuePendingConfigPost()) return;

  String endpoint;
  String body;
  String kind;
  uint32_t sequence = 0UL;
  bool critical = false;

  {
    // Destroy the 8 KB document before the async worker begins its TLS handshake.
    DynamicJsonDocument outboxDoc(8192);
    loadOutboxDoc(outboxDoc);
    JsonArray events = outboxDoc["events"].as<JsonArray>();
    if (events.isNull() || events.size() == 0) {
      state.bufferedEventCount = 0;
      return;
    }

    int selectedIndex = selectBufferedOutboxEventIndex(events);
    if (selectedIndex < 0) return;

    JsonVariant ev = events[selectedIndex];
    endpoint = ev["endpoint"] | "";
    body = ev["body"] | "";
    kind = ev["kind"] | "event";
    sequence = ev["seq"] | 0UL;
    critical = ev["critical"] | false;
  }

  if (endpoint.length() == 0 || body.length() == 0 || sequence == 0UL) return;
  if (kind == "sensor_state" && hasPendingAsyncHttpKind(AsyncHttpKind::SENSOR_POST)) return;

  LOG_DEBUG("Replaying buffered event kind=%s seq=%lu critical=%d queued=%u free_heap=%u max_alloc=%u",
            kind.c_str(),
            (unsigned long)sequence,
            critical ? 1 : 0,
            state.bufferedEventCount,
            (unsigned int)ESP.getFreeHeap(),
            (unsigned int)ESP.getMaxAllocHeap());
  enqueueAsyncHttp(
      AsyncHttpMethod::POST,
      AsyncHttpKind::OUTBOX_POST,
      String(SERVER_BASE) + endpoint,
      body,
      endpoint,
      kind,
      critical,
      sequence);
}

static unsigned int countBufferedCriticalEvents() {
  DynamicJsonDocument outboxDoc(8192);
  loadOutboxDoc(outboxDoc);
  JsonArray events = outboxDoc["events"].as<JsonArray>();
  if (events.isNull()) return 0U;

  unsigned int count = 0U;
  for (JsonVariant event : events) {
    if (event["critical"] | false) count++;
  }
  return count;
}

static bool hasPendingCriticalAsyncHttp() {
  for (uint8_t i = 0; i < kAsyncHttpSlotCount; ++i) {
    if (gAsyncHttpSlots[i].inUse && gAsyncHttpSlots[i].critical) return true;
  }
  return false;
}

bool flushCriticalOutboxBeforeShutdown(uint8_t transportAttempts, unsigned long timeoutMs) {
  if (transportAttempts < 5U) transportAttempts = 5U;
  if (timeoutMs == 0UL) timeoutMs = 1UL;

  gShutdownFlushActive = true;
  gShutdownFlushCriticalFailure = false;
  gShutdownAlertDelivered = false;
  gShutdownFlushTransportAttempts = transportAttempts;

  const unsigned long startedMs = millis();
  unsigned long lastWiFiAttemptMs = 0UL;
  uint8_t wifiAttempts = 0U;
  bool delivered = false;

  LOG_WARN("Shutdown network flush started critical_events=%u transport_attempts=%u timeout_ms=%lu",
           countBufferedCriticalEvents(),
           (unsigned int)transportAttempts,
           timeoutMs);

  while (millis() - startedMs < timeoutMs) {
    serviceAsyncHttp();

    if (gShutdownFlushCriticalFailure) {
      break;
    }

    bool criticalRequestPending = hasPendingCriticalAsyncHttp();
    if (!criticalRequestPending && getAsyncHttpPendingCount() == 0U) {
      unsigned int remainingCritical = countBufferedCriticalEvents();
      if (remainingCritical == 0U) {
        delivered = true;
        break;
      }
    }

    if (!WiFi.isConnected()) {
      unsigned long nowMs = millis();
      if (lastWiFiAttemptMs == 0UL || nowMs - lastWiFiAttemptMs >= 5000UL) {
        if (wifiAttempts >= transportAttempts) {
          LOG_ERROR("Shutdown network flush exhausted WiFi reconnect attempts=%u",
                    (unsigned int)wifiAttempts);
          break;
        }
        wifiAttempts++;
        lastWiFiAttemptMs = nowMs;
        LOG_WARN("Shutdown network flush WiFi reconnect attempt=%u/%u",
                 (unsigned int)wifiAttempts,
                 (unsigned int)transportAttempts);
        WiFi.reconnect();
      }
    } else {
      serviceBufferedOutbox();
    }

    delay(20);
  }

  serviceAsyncHttp();
  unsigned int remainingCritical = countBufferedCriticalEvents();
  unsigned long elapsedMs = millis() - startedMs;
  if (delivered) {
    LOG_INFO("Shutdown network flush completed elapsed_ms=%lu remaining_critical=%u",
             elapsedMs,
             remainingCritical);
  } else {
    LOG_ERROR("Shutdown network flush incomplete elapsed_ms=%lu remaining_critical=%u pending_http=%u; events remain persisted",
              elapsedMs,
              remainingCritical,
              getAsyncHttpPendingCount());
  }

  gShutdownFlushActive = false;
  gShutdownFlushTransportAttempts = kHttpTransportMaxAttempts;
  return delivered;
}

void setupOTA() {
  if (gOtaStarted) return;
  if (WiFi.status() != WL_CONNECTED) {
    LOG_WARN("OTA setup skipped; WiFi not connected yet");
    return;
  }

  if (gOtaHostname.length() == 0) {
    gOtaHostname = String("smart-feeder-") + DEVICE_ID;
  }

  ArduinoOTA.setHostname(gOtaHostname.c_str());
  ArduinoOTA.onStart([]() {
    gOtaInProgress = true;
    gOtaLastProgressBucket = 101U;
    suspendCore0WatchdogForOTA();
    LOG_INFO("OTA start; CPU0 idle watchdog temporarily suspended");
  });
  ArduinoOTA.onEnd([]() {
    restoreCore0WatchdogAfterOTA();
    gOtaInProgress = false;
    LOG_INFO("OTA end; CPU0 idle watchdog restored");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total > 0U) {
      unsigned int pct = (progress * 100U) / total;
      unsigned int bucket = pct / 10U;
      if (bucket != gOtaLastProgressBucket) {
        gOtaLastProgressBucket = bucket;
        LOG_INFO("OTA progress %u%%", pct);
      }
      // ArduinoOTA processes the whole upload in one blocking call. Yield
      // between flash chunks so the CPU 0 idle task can reset its watchdog.
      delay(1);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    restoreCore0WatchdogAfterOTA();
    gOtaInProgress = false;
    LOG_ERROR("OTA error=%u; CPU0 idle watchdog restored", (unsigned int)error);
  });

  ArduinoOTA.begin();
  gOtaStarted = true;
  MDNS.begin(gOtaHostname.c_str());
  LOG_INFO("OTA ready hostname=%s ip=%s", gOtaHostname.c_str(), WiFi.localIP().toString().c_str());
}

void serviceOTA() {
  if (!gOtaStarted) {
    if (WiFi.status() == WL_CONNECTED) {
      setupOTA();
    } else {
      return;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }
}

bool isOTAInProgress() {
  return gOtaInProgress;
}
