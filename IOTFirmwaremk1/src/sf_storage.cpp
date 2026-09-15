#include "sf_storage.h"

#include <ArduinoJson.h>

#include "sf_config.h"
#include "sf_debug.h"
#include "sf_globals.h"
#include "sf_sensors.h"
#include "sf_utils.h"

static String getLocalDateYmd() {
  time_t now = time(nullptr);
  struct tm tmNow;
  localtime_r(&now, &tmNow);
  char buf[16];
  strftime(buf, sizeof(buf), "%Y-%m-%d", &tmNow);
  return String(buf);
}

static const char* PREF_CONFIG_CHUNK_COUNT = "cfg_part_count";
static const char* PREF_CONFIG_CHUNK_PREFIX = "cfg_part_";
static const size_t LOCAL_CONFIG_CHUNK_SIZE = 768;

static String makeConfigChunkKey(uint16_t index) {
  return String(PREF_CONFIG_CHUNK_PREFIX) + String(index);
}

static bool writeLocalConfigChunked(const String& jsonCfg) {
  size_t totalBytes = jsonCfg.length();
  uint16_t chunkCount = (uint16_t)((totalBytes + LOCAL_CONFIG_CHUNK_SIZE - 1U) / LOCAL_CONFIG_CHUNK_SIZE);
  if (chunkCount == 0) {
    chunkCount = 1;
  }

  for (uint16_t index = 0; index < chunkCount; ++index) {
    const size_t start = (size_t)index * LOCAL_CONFIG_CHUNK_SIZE;
    const size_t end = start + LOCAL_CONFIG_CHUNK_SIZE;
    String chunk = jsonCfg.substring(start, end > totalBytes ? totalBytes : end);
    String key = makeConfigChunkKey(index);
    prefs.putString(key.c_str(), chunk);
  }

  prefs.putUInt(PREF_CONFIG_CHUNK_COUNT, chunkCount);
  return true;
}

static bool loadConfigDoc(DynamicJsonDocument& d) {
  String cfg = loadLocalConfig();
  d.clear();
  DeserializationError err = deserializeJson(d, cfg);
  if (err) {
    LOG_ERROR("Local config parse failed in storage helpers: %s bytes=%u", err.c_str(), (unsigned int)cfg.length());
    return false;
  }
  return true;
}

static String formatUtcIso(time_t ts) {
  struct tm timeinfo;
  gmtime_r(&ts, &timeinfo);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

static String nextConfigOwnerTimestamp(JsonVariant cfgObj) {
  String nowIso = getUtcIsoNow();
  time_t nowTs = parseIsoUtc(nowIso.c_str());
  time_t prevTs = parseIsoUtc(cfgObj["last_updated"] | "");

  if (prevTs > 0 && nowTs <= prevTs) {
    return formatUtcIso(prevTs + 1);
  }

  return nowIso;
}

static void stampConfigOwnerFields(JsonVariant cfgObj) {
  cfgObj["last_updated"] = nextConfigOwnerTimestamp(cfgObj);
  cfgObj["updated_by"] = "esp32";
}

static String getEmptyOutboxJson() {
  StaticJsonDocument<128> d;
  d["next_seq"] = 1UL;
  d.createNestedArray("events");
  String out;
  serializeJson(d, out);
  return out;
}

static bool nearlyEqualFloat(float a, float b, float epsilon = 0.01f) {
  float diff = a - b;
  if (diff < 0.0f) diff = -diff;
  return diff <= epsilon;
}

static const char* const DEPRECATED_CONFIG_KEYS[] = {
  "feeder_tank_bottom_distance_cm",
  "feeder_tank_full_distance_cm",
  "feeder_tank_depth_cm",
  "feeder_max_feed_height_cm",
  "water_tank_depth_cm",
  "battery_adc_pin",
  "battery_divider_top_ohms",
  "battery_divider_bottom_ohms",
  "battery_adc_reference_v",
  "battery_adc_gain_correction",
  "keypad_calibration",
};

static bool stripDeprecatedConfigFields(JsonVariant cfgObj) {
  if (cfgObj.isNull()) return false;

  bool changed = false;
  for (const char* key : DEPRECATED_CONFIG_KEYS) {
    if (cfgObj.containsKey(key)) {
      cfgObj.remove(key);
      changed = true;
    }
  }
  return changed;
}

static const char kStoredKeypadKeys[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'};

static const char* const kLegacyKeypadFields[16] = {
    "adc_1", "adc_2", "adc_3", "adc_a",
    "adc_4", "adc_5", "adc_6", "adc_b",
    "adc_7", "adc_8", "adc_9", "adc_c",
    "adc_star", "adc_0", "adc_hash", "adc_d"};

static const int kDefaultKeypadCenters[16] = {
    60, 200, 355, 510,
    675, 850, 1030, 1215,
    1410, 1620, 1845, 2080,
    2340, 2630, 2940, 3275};

static int clampStoredAdc(int value) {
  if (value < 0) return 0;
  if (value > 4095) return 4095;
  return value;
}

static bool hasMonotonicKeypadCenters(const int centers[16]) {
  int trend = 0;
  for (int i = 0; i < 16; i++) {
    if (centers[i] <= 0 || centers[i] >= 4095) return false;
    if (i == 0) continue;
    int delta = centers[i] - centers[i - 1];
    if (delta == 0) return false;
    int stepTrend = delta > 0 ? 1 : -1;
    if (trend == 0) trend = stepTrend;
    if (trend != stepTrend) return false;
  }
  return true;
}

static String buildRangeKeypadCalibration(JsonVariant legacyCalibration) {
  int centers[16];
  for (int i = 0; i < 16; i++) {
    centers[i] = legacyCalibration.isNull()
                     ? kDefaultKeypadCenters[i]
                     : (legacyCalibration[kLegacyKeypadFields[i]] | kDefaultKeypadCenters[i]);
  }
  if (!hasMonotonicKeypadCenters(centers)) {
    for (int i = 0; i < 16; i++) centers[i] = kDefaultKeypadCenters[i];
  }
  for (int i = 0; i < 16; i++) centers[i] = clampStoredAdc(centers[i] + KEYPAD_ADC_OFFSET);

  int noKeyAdc = legacyCalibration.isNull()
                     ? KEYPAD_ADC_NO_KEY_MIN
                     : (legacyCalibration["no_key_adc"] | KEYPAD_ADC_NO_KEY_MIN);
  noKeyAdc = clampStoredAdc(noKeyAdc + KEYPAD_ADC_OFFSET);

  DynamicJsonDocument calibration(3072);
  calibration["version"] = 2;
  calibration["calibrated_at"] = legacyCalibration.isNull()
                                      ? "factory_default"
                                      : (legacyCalibration["calibrated_at"] | "legacy_migration");
  calibration["source"] = legacyCalibration.isNull() ? "factory_default" : "legacy_center_migration";
  calibration["idle_min"] = clampStoredAdc(noKeyAdc - KEYPAD_IDLE_BAND_ADC);
  calibration["idle_max"] = clampStoredAdc(noKeyAdc + KEYPAD_IDLE_BAND_ADC);
  JsonArray keys = calibration.createNestedArray("keys");

  for (int i = 0; i < 16; i++) {
    int minAdc = clampStoredAdc(centers[i] - KEYPAD_ADC_TOLERANCE);
    int maxAdc = clampStoredAdc(centers[i] + KEYPAD_ADC_TOLERANCE);
    if (i > 0) {
      int midpoint = (centers[i] + centers[i - 1]) / 2;
      if (centers[i - 1] < centers[i]) {
        if (minAdc <= midpoint) minAdc = midpoint + KEYPAD_WINDOW_EDGE_PAD_ADC;
      } else if (maxAdc >= midpoint) {
        maxAdc = midpoint - KEYPAD_WINDOW_EDGE_PAD_ADC;
      }
    }
    if (i < 15) {
      int midpoint = (centers[i] + centers[i + 1]) / 2;
      if (centers[i + 1] > centers[i]) {
        if (maxAdc >= midpoint) maxAdc = midpoint - KEYPAD_WINDOW_EDGE_PAD_ADC;
      } else if (minAdc <= midpoint) {
        minAdc = midpoint + KEYPAD_WINDOW_EDGE_PAD_ADC;
      }
    }
    if (minAdc > centers[i]) minAdc = centers[i];
    if (maxAdc < centers[i]) maxAdc = centers[i];

    JsonObject entry = keys.createNestedObject();
    char keyName[2] = {kStoredKeypadKeys[i], '\0'};
    entry["key"] = keyName;
    entry["min"] = minAdc;
    entry["max"] = maxAdc;
  }

  String out;
  serializeJson(calibration, out);
  return out;
}

static bool ensureLocalKeypadCalibration(JsonVariant cfgObj) {
  bool changed = false;
  JsonVariant legacyCalibration = cfgObj["keypad_calibration"];
  if (!prefs.isKey(PREF_KEYPAD_CALIBRATION) && !legacyCalibration.isNull()) {
    String calibration = buildRangeKeypadCalibration(legacyCalibration);
    prefs.putString(PREF_KEYPAD_CALIBRATION, calibration);
    LOG_INFO("Initialized local-only keypad range calibration bytes=%u source=%s",
             (unsigned int)calibration.length(),
             legacyCalibration.isNull() ? "factory_default" : "legacy_migration");
  }
  if (!legacyCalibration.isNull()) {
    cfgObj.remove("keypad_calibration");
    changed = true;
    LOG_INFO("Removed legacy keypad calibration from synchronized config");
  }
  return changed;
}

static float deriveMaxSingleFeedKg(JsonVariant cfg) {
  float maxCap = DEFAULT_MAX_FEEDS_CAPACITY_KG;
  if (!cfg.isNull() && cfg.containsKey("max_feeds_capacity_kg")) {
    maxCap = cfg["max_feeds_capacity_kg"] | DEFAULT_MAX_FEEDS_CAPACITY_KG;
  }
  if (maxCap <= 0.0f) maxCap = DEFAULT_MAX_FEEDS_CAPACITY_KG;

  float remainingKg = getFeederRemainingKg(cfg);
  if (remainingKg < 0.0f) {
    remainingKg = readRemainingKg();
  }
  if (remainingKg < 0.0f) remainingKg = 0.0f;
  if (remainingKg > maxCap) remainingKg = maxCap;
  return remainingKg;
}

static void syncMaxSingleFeedKg(JsonVariant cfgObj) {
  if (cfgObj.isNull()) return;

  float derived = deriveMaxSingleFeedKg(cfgObj);
  float current = cfgObj["max_single_feed_kg"] | -1.0f;
  if (nearlyEqualFloat(current, derived)) return;

  cfgObj["max_single_feed_kg"] = derived;

  String out;
  serializeJson(cfgObj.as<JsonVariant>(), out);
  saveLocalConfig(out);
}

static uint32_t extractFeedNowCommandIdFromVariant(JsonVariant cmd) {
  if (cmd.isNull()) return 0UL;

  long cmdIdRaw = cmd["command_id"] | -1;
  if (cmdIdRaw <= 0) {
    cmdIdRaw = cmd["id"] | -1;
  }
  return cmdIdRaw > 0 ? (uint32_t)cmdIdRaw : 0UL;
}

static uint32_t computeManualFeedSnapshotId(JsonVariant cfgObj, uint32_t lastHandledId) {
  if (cfgObj.isNull()) return lastHandledId;

  uint32_t snapshotId = cfgObj["manual_feed_snapshot_id"] | 0UL;
  uint32_t mirroredLastId = cfgObj["last_feed_now_command_id"] | 0UL;
  uint32_t pendingId = extractFeedNowCommandIdFromVariant(cfgObj["feed_now_command"]);

  if (mirroredLastId > snapshotId) snapshotId = mirroredLastId;
  if (pendingId > snapshotId) snapshotId = pendingId;
  if (lastHandledId > snapshotId) snapshotId = lastHandledId;
  return snapshotId;
}

static bool applyManualFeedTrackingFields(JsonVariant cfgObj,
                                          uint32_t lastHandledId,
                                          uint32_t snapshotId,
                                          bool stampOwner) {
  if (cfgObj.isNull()) return false;

  bool changed = false;
  if ((cfgObj["last_feed_now_command_id"] | 0UL) != lastHandledId) {
    cfgObj["last_feed_now_command_id"] = lastHandledId;
    changed = true;
  }
  if ((cfgObj["manual_feed_snapshot_id"] | 0UL) != snapshotId) {
    cfgObj["manual_feed_snapshot_id"] = snapshotId;
    changed = true;
  }
  if (changed && stampOwner) {
    stampConfigOwnerFields(cfgObj);
  }
  return changed;
}

static bool saveManualFeedTrackingState(uint32_t lastHandledId,
                                        uint32_t snapshotId,
                                        bool clearPendingCommand,
                                        bool stampOwner) {
  DynamicJsonDocument d(8192);
  if (!loadConfigDoc(d)) return false;

  JsonVariant cfgObj = d.as<JsonVariant>();
  bool changed = false;
  if (clearPendingCommand && cfgObj.containsKey("feed_now_command")) {
    cfgObj.remove("feed_now_command");
    changed = true;
  }
  if (applyManualFeedTrackingFields(cfgObj, lastHandledId, snapshotId, stampOwner)) {
    changed = true;
  }
  if (!changed) return true;

  String out;
  serializeJson(d, out);
  saveLocalConfig(out);
  return true;
}

float getMaxSingleFeedKg(JsonVariant cfg) {
  return deriveMaxSingleFeedKg(cfg);
}

uint32_t getFeedNowCommandIdFromConfig(JsonVariant cfg) {
  if (cfg.isNull()) return 0UL;
  return extractFeedNowCommandIdFromVariant(cfg["feed_now_command"]);
}

uint32_t getManualFeedSnapshotId(JsonVariant cfg) {
  if (cfg.isNull()) return 0UL;
  uint32_t lastHandledId = cfg["last_feed_now_command_id"] | 0UL;
  return computeManualFeedSnapshotId(cfg, lastHandledId);
}

bool isManualFeedSnapshotSynced(JsonVariant cfg) {
  return getManualFeedSnapshotId(cfg) == readLastFeedNowCommandId();
}

static void seedDefaultGrainTypes(JsonVariant cfgObj) {
  JsonArray grainTypes = cfgObj.createNestedArray("grain_types");
  for (int i = 0; i < DEFAULT_GRAIN_TYPE_COUNT; i++) {
    JsonObject entry = grainTypes.createNestedObject();
    entry["grain_type"] = DEFAULT_GRAIN_TYPES[i].grain_type;
    entry["feed_ms_per_kg"] = DEFAULT_GRAIN_TYPES[i].feed_ms_per_kg;
  }
}

int getGrainTypeCount(JsonVariant cfg) {
  if (!cfg.isNull() && cfg.containsKey("grain_types")) {
    JsonArray grainTypes = cfg["grain_types"].as<JsonArray>();
    if (!grainTypes.isNull() && grainTypes.size() > 0) {
      return (int)grainTypes.size();
    }
  }
  return DEFAULT_GRAIN_TYPE_COUNT;
}

const char* getGrainTypeNameByIndex(JsonVariant cfg, int index) {
  if (index < 0) return DEFAULT_GRAIN_TYPE;

  if (!cfg.isNull() && cfg.containsKey("grain_types")) {
    JsonArray grainTypes = cfg["grain_types"].as<JsonArray>();
    if (!grainTypes.isNull() && index < (int)grainTypes.size()) {
      int i = 0;
      for (JsonVariant grainTypeCfg : grainTypes) {
        if (i == index) {
          return grainTypeCfg["grain_type"] | DEFAULT_GRAIN_TYPE;
        }
        i++;
      }
    }
  }

  if (index < DEFAULT_GRAIN_TYPE_COUNT) {
    return DEFAULT_GRAIN_TYPES[index].grain_type;
  }
  return DEFAULT_GRAIN_TYPE;
}

float getGrainTypeMsPerKgByIndex(JsonVariant cfg, int index) {
  if (index < 0) return FEED_MS_PER_KG_STANDARD_PELLETS;

  if (!cfg.isNull() && cfg.containsKey("grain_types")) {
    JsonArray grainTypes = cfg["grain_types"].as<JsonArray>();
    if (!grainTypes.isNull() && index < (int)grainTypes.size()) {
      int i = 0;
      for (JsonVariant grainTypeCfg : grainTypes) {
        if (i == index) {
          float rate = grainTypeCfg["feed_ms_per_kg"] | 0.0f;
          if (rate > 0.0f) return rate;
          break;
        }
        i++;
      }
    }
  }

  if (index < DEFAULT_GRAIN_TYPE_COUNT) {
    return DEFAULT_GRAIN_TYPES[index].feed_ms_per_kg;
  }
  return FEED_MS_PER_KG_STANDARD_PELLETS;
}

int findGrainTypeIndex(JsonVariant cfg, const char* grainType) {
  if (grainType == nullptr || strlen(grainType) == 0) return 0;

  if (!cfg.isNull() && cfg.containsKey("grain_types")) {
    JsonArray grainTypes = cfg["grain_types"].as<JsonArray>();
    if (!grainTypes.isNull()) {
      int i = 0;
      for (JsonVariant grainTypeCfg : grainTypes) {
        const char* name = grainTypeCfg["grain_type"] | "";
        if (strcmp(name, grainType) == 0) return i;
        i++;
      }
    }
  }

  for (int i = 0; i < DEFAULT_GRAIN_TYPE_COUNT; i++) {
    if (strcmp(DEFAULT_GRAIN_TYPES[i].grain_type, grainType) == 0) return i;
  }

  return 0;
}

int getSelectedGrainTypeIndex(JsonVariant cfg) {
  int count = getGrainTypeCount(cfg);
  if (count <= 0) count = DEFAULT_GRAIN_TYPE_COUNT;

  if (!cfg.isNull() && cfg.containsKey("grain_type_index")) {
    int index = cfg["grain_type_index"] | 0;
    if (index < 0) index = 0;
    if (count > 0 && index >= count) index %= count;
    return index;
  }

  const char* grainType = cfg.isNull() ? DEFAULT_GRAIN_TYPE : (cfg["grain_type"] | DEFAULT_GRAIN_TYPE);
  return findGrainTypeIndex(cfg, grainType);
}

void saveGrainTypeSelection(JsonVariant cfg, int selectedIndex) {
  int count = getGrainTypeCount(cfg);
  if (count <= 0) count = DEFAULT_GRAIN_TYPE_COUNT;
  if (count <= 0) return;
  if (selectedIndex < 0) selectedIndex = 0;
  if (selectedIndex >= count) selectedIndex %= count;

  const char* selectedType = getGrainTypeNameByIndex(cfg, selectedIndex);
  float selectedRate = getGrainTypeMsPerKgByIndex(cfg, selectedIndex);

  String cfgStr = loadLocalConfig();
  DynamicJsonDocument d(8192);
  DeserializationError err = deserializeJson(d, cfgStr);
  if (err) return;

  JsonVariant root = d.as<JsonVariant>();
  root["grain_type_index"] = selectedIndex;
  root["grain_type"] = selectedType;
  root["feed_ms_per_kg"] = selectedRate;
  stampConfigOwnerFields(root);

  String out;
  serializeJson(d, out);
  saveLocalConfig(out);
  markLocalConfigDirty();
}

String loadLocalConfig() {
  if (!prefs.begin(PREF_NAMESPACE, true)) {
    LOG_ERROR("Preferences begin(readonly) failed");
    return "{}";
  }
  String cfg;
  if (prefs.isKey(PREF_CONFIG_CHUNK_COUNT)) {
    uint16_t chunkCount = (uint16_t)prefs.getUInt(PREF_CONFIG_CHUNK_COUNT, 0U);
    if (chunkCount > 0) {
      cfg.reserve((size_t)chunkCount * LOCAL_CONFIG_CHUNK_SIZE);
      for (uint16_t index = 0; index < chunkCount; ++index) {
        String key = makeConfigChunkKey(index);
        cfg += prefs.getString(key.c_str(), "");
      }
    }
  }

  if (cfg.length() == 0) {
    cfg = prefs.getString(PREF_CONFIG_KEY, "{}");
  }
  prefs.end();
  LOG_DEBUG("Loaded local config bytes=%u", (unsigned int)cfg.length());
  return cfg;
}

void saveLocalConfig(const String& jsonCfg) {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(write) failed");
    return;
  }
  String sanitized = jsonCfg;
  DynamicJsonDocument d(8192);
  if (!deserializeJson(d, jsonCfg)) {
    stripDeprecatedConfigFields(d.as<JsonVariant>());
    uint32_t lastHandledId = prefs.getULong(PREF_LAST_FEED_CMD_ID, 0UL);
    uint32_t snapshotId = computeManualFeedSnapshotId(d.as<JsonVariant>(), lastHandledId);
    applyManualFeedTrackingFields(d.as<JsonVariant>(), lastHandledId, snapshotId, false);
    sanitized = "";
    serializeJson(d, sanitized);
  }
  writeLocalConfigChunked(sanitized);
  prefs.end();
  cachedCfgStr = sanitized;
  LOG_INFO("Saved local config bytes=%u chunks=%u", (unsigned int)sanitized.length(),
           (unsigned int)((sanitized.length() + LOCAL_CONFIG_CHUNK_SIZE - 1U) / LOCAL_CONFIG_CHUNK_SIZE));
}

bool isLocalConfigDirty() {
  if (!prefs.begin(PREF_NAMESPACE, true)) {
    LOG_ERROR("Preferences begin(read config dirty) failed");
    return false;
  }
  bool dirty = prefs.getBool(PREF_CONFIG_DIRTY, false);
  prefs.end();
  return dirty;
}

void markLocalConfigDirty() {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(write config dirty) failed");
    return;
  }
  prefs.putBool(PREF_CONFIG_DIRTY, true);
  prefs.end();
  LOG_INFO("Local config marked dirty for server sync");
}

void clearLocalConfigDirty() {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(clear config dirty) failed");
    return;
  }
  prefs.putBool(PREF_CONFIG_DIRTY, false);
  prefs.end();
  LOG_INFO("Local config dirty flag cleared");
}

String loadLocalKeypadCalibration() {
  if (!prefs.begin(PREF_NAMESPACE, true)) {
    LOG_ERROR("Preferences begin(read keypad calibration) failed");
    return "{}";
  }
  bool hasCalibration = prefs.isKey(PREF_KEYPAD_CALIBRATION);
  String calibration = prefs.getString(PREF_KEYPAD_CALIBRATION, "");
  prefs.end();
  if (hasCalibration && calibration.length() > 0U) return calibration;

  String cfg = loadLocalConfig();
  DynamicJsonDocument configDoc(8192);
  DeserializationError configErr = deserializeJson(configDoc, cfg);
  JsonVariant legacyCalibration;
  if (!configErr) legacyCalibration = configDoc["keypad_calibration"];
  calibration = buildRangeKeypadCalibration(legacyCalibration);
  if (!saveLocalKeypadCalibration(calibration)) return calibration;

  if (!configErr && !legacyCalibration.isNull()) {
    configDoc.remove("keypad_calibration");
    String sanitized;
    serializeJson(configDoc, sanitized);
    saveLocalConfig(sanitized);
    LOG_INFO("Migrated legacy keypad centers into local-only range calibration");
  }
  return calibration;
}

bool saveLocalKeypadCalibration(const String& jsonCalibration) {
  DynamicJsonDocument calibration(3072);
  DeserializationError err = deserializeJson(calibration, jsonCalibration);
  if (err || (calibration["version"] | 0) != 2 || !calibration["keys"].is<JsonArray>()) {
    LOG_ERROR("Rejected invalid local keypad calibration: %s", err ? err.c_str() : "invalid schema");
    return false;
  }

  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(write keypad calibration) failed");
    return false;
  }
  size_t written = prefs.putString(PREF_KEYPAD_CALIBRATION, jsonCalibration);
  prefs.end();
  if (written == 0U && jsonCalibration.length() > 0U) {
    LOG_ERROR("Local keypad calibration write failed");
    return false;
  }
  LOG_INFO("Saved local-only keypad calibration bytes=%u", (unsigned int)jsonCalibration.length());
  return true;
}

float readRemainingKg() {
  if (!prefs.begin(PREF_NAMESPACE, true)) {
    LOG_ERROR("Preferences begin(read remaining) failed");
    return DEFAULT_MAX_FEEDS_CAPACITY_KG;
  }
  float v = prefs.getFloat(PREF_REMAINING_KG, DEFAULT_MAX_FEEDS_CAPACITY_KG);
  prefs.end();
  return v;
}

void writeRemainingKg(float v) {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(write remaining) failed");
    return;
  }
  prefs.putFloat(PREF_REMAINING_KG, v);
  prefs.end();
  LOG_DEBUG("Remaining kg updated: %.3f", v);

  DynamicJsonDocument d(8192);
  if (loadConfigDoc(d)) {
    syncMaxSingleFeedKg(d.as<JsonVariant>());
  }
}

uint32_t readLastFeedNowCommandId() {
  if (!prefs.begin(PREF_NAMESPACE, true)) {
    LOG_ERROR("Preferences begin(read feed cmd id) failed");
    return 0;
  }
  uint32_t v = prefs.getULong(PREF_LAST_FEED_CMD_ID, 0UL);
  prefs.end();
  return v;
}

void writeLastFeedNowCommandId(uint32_t id) {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(write feed cmd id) failed");
    return;
  }
  prefs.putULong(PREF_LAST_FEED_CMD_ID, id);
  prefs.end();
  LOG_INFO("Last feed_now command id updated: %lu", (unsigned long)id);

  if (!saveManualFeedTrackingState(id, id, true, true)) {
    LOG_WARN("Failed to mirror manual feed tracking fields after id=%lu", (unsigned long)id);
  }
}

bool resetManualFeedSnapshotToLastFeedNowCommandId() {
  uint32_t lastHandledId = readLastFeedNowCommandId();
  bool ok = saveManualFeedTrackingState(lastHandledId, lastHandledId, true, true);
  if (ok) {
    LOG_INFO("Manual feed snapshot reset to last handled id: %lu", (unsigned long)lastHandledId);
  } else {
    LOG_WARN("Manual feed snapshot reset failed for id=%lu", (unsigned long)lastHandledId);
  }
  return ok;
}

String loadEventOutbox() {
  if (!prefs.begin(PREF_NAMESPACE, true)) {
    LOG_ERROR("Preferences begin(read outbox) failed");
    return getEmptyOutboxJson();
  }
  String outbox = prefs.getString(PREF_EVENT_OUTBOX, getEmptyOutboxJson());
  prefs.end();
  return outbox;
}

void saveEventOutbox(const String& jsonOutbox) {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(write outbox) failed");
    return;
  }
  prefs.putString(PREF_EVENT_OUTBOX, jsonOutbox);
  prefs.end();
  LOG_DEBUG("Saved event outbox bytes=%u", (unsigned int)jsonOutbox.length());
}

uint32_t readEventSequence() {
  if (!prefs.begin(PREF_NAMESPACE, true)) {
    LOG_ERROR("Preferences begin(read event seq) failed");
    return 1UL;
  }
  uint32_t seq = prefs.getULong(PREF_EVENT_SEQ, 1UL);
  prefs.end();
  return seq;
}

void writeEventSequence(uint32_t seq) {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(write event seq) failed");
    return;
  }
  prefs.putULong(PREF_EVENT_SEQ, seq);
  prefs.end();
}

unsigned int readBufferedEventCount() {
  String raw = loadEventOutbox();
  DynamicJsonDocument d(8192);
  DeserializationError err = deserializeJson(d, raw);
  if (err) return 0U;
  JsonArray events = d["events"].as<JsonArray>();
  return events.isNull() ? 0U : (unsigned int)events.size();
}

void ensureLocalDefaults() {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    LOG_ERROR("Preferences begin(init) failed");
    return;
  }

  if (!prefs.isKey(PREF_CONFIG_KEY)) {
    StaticJsonDocument<1536> d;
    String nowIso = getUtcIsoNow();
    d["last_updated"] = nowIso;
    d["updated_by"] = "esp32";
    d["keypad_input_enabled"] = SF_ENABLE_KEYPAD_INPUT ? true : false;
    d["max_feeds_capacity_kg"] = DEFAULT_MAX_FEEDS_CAPACITY_KG;
    // Initialize max single feed to the capacity by default so UI isn't artificially capped.
    d["max_single_feed_kg"] = d["max_feeds_capacity_kg"] | DEFAULT_MAX_FEEDS_CAPACITY_KG;
    d["grain_type_index"] = 0;
    d["grain_type"] = DEFAULT_GRAIN_TYPE;
    d["feed_ms_per_kg"] = FEED_MS_PER_KG_STANDARD_PELLETS;
    d["feeder_low_threshold_pct"] = DEFAULT_FEEDER_LOW_THRESHOLD_PCT;
    d["feeder_high_threshold_pct"] = DEFAULT_FEEDER_HIGH_THRESHOLD_PCT;
    d["water_low_threshold_pct"] = DEFAULT_WATER_LOW_THRESHOLD_PCT;
    d["water_high_threshold_pct"] = DEFAULT_WATER_HIGH_THRESHOLD_PCT;
    d["battery_sense_enabled"] = true;
    d["low_battery_shutdown_v"] = DEFAULT_LOW_BATTERY_SHUTDOWN_V;
    d["low_battery_shutdown_resolve_v"] = DEFAULT_LOW_BATTERY_SHUTDOWN_RESOLVE_V;
    d["last_feed_now_command_id"] = 0UL;
    d["manual_feed_snapshot_id"] = 0UL;
    d["max_feeds_capacity_updated_at"] = nowIso;
    d["max_feeds_capacity_updated_by"] = "esp32";
    seedDefaultGrainTypes(d.as<JsonVariant>());
    d.createNestedArray("schedules");
    String s;
    serializeJson(d, s);
    prefs.putString(PREF_CONFIG_KEY, s);
    LOG_INFO("Created default local config");
  }

  if (!prefs.isKey(PREF_REMAINING_KG)) {
    prefs.putFloat(PREF_REMAINING_KG, DEFAULT_MAX_FEEDS_CAPACITY_KG);
    LOG_INFO("Initialized remaining feed kg default");
  }

  if (!prefs.isKey(PREF_EVENT_OUTBOX)) {
    prefs.putString(PREF_EVENT_OUTBOX, getEmptyOutboxJson());
    LOG_INFO("Initialized event outbox default");
  }

  if (!prefs.isKey(PREF_EVENT_SEQ)) {
    prefs.putULong(PREF_EVENT_SEQ, 1UL);
    LOG_INFO("Initialized event sequence default");
  }

  if (!prefs.isKey(PREF_LAST_FEED_CMD_ID)) {
    prefs.putULong(PREF_LAST_FEED_CMD_ID, 0UL);
    LOG_INFO("Initialized last feed_now command id default");
  }

  if (prefs.isKey(PREF_CONFIG_KEY)) {
    String cfg = prefs.getString(PREF_CONFIG_KEY, "{}");
    DynamicJsonDocument d(8192);
    DeserializationError err = deserializeJson(d, cfg);
    if (!err) {
      bool changed = false;
      if (ensureLocalKeypadCalibration(d.as<JsonVariant>())) {
        changed = true;
      }
      if (stripDeprecatedConfigFields(d.as<JsonVariant>())) {
        changed = true;
      }
      if (!d.containsKey("keypad_input_enabled")) {
        d["keypad_input_enabled"] = SF_ENABLE_KEYPAD_INPUT ? true : false;
        changed = true;
      }
      if (!d.containsKey("total_feeds_today_kg")) {
        d["total_feeds_today_kg"] = 0.0f;
        changed = true;
      }
      if (!d.containsKey("total_feeds_today_date")) {
        d["total_feeds_today_date"] = getLocalDateYmd();
        changed = true;
      }
      if (!d.containsKey("grain_type")) {
        d["grain_type"] = DEFAULT_GRAIN_TYPE;
        changed = true;
      }
      if (!d.containsKey("grain_types") || !d["grain_types"].is<JsonArray>()) {
        seedDefaultGrainTypes(d.as<JsonVariant>());
        changed = true;
      }
      int selectedGrainIndex = getSelectedGrainTypeIndex(d.as<JsonVariant>());
      const char* selectedGrainType = getGrainTypeNameByIndex(d.as<JsonVariant>(), selectedGrainIndex);
      float selectedGrainRate = getGrainTypeMsPerKgByIndex(d.as<JsonVariant>(), selectedGrainIndex);
      if (!d.containsKey("grain_type_index")) {
        d["grain_type_index"] = selectedGrainIndex;
        changed = true;
      }
      if (!d.containsKey("grain_type") || strcmp(d["grain_type"] | "", selectedGrainType) != 0) {
        d["grain_type"] = selectedGrainType;
        changed = true;
      }
      if (!d.containsKey("feed_ms_per_kg") || !nearlyEqualFloat(d["feed_ms_per_kg"] | 0.0f, selectedGrainRate)) {
        d["feed_ms_per_kg"] = selectedGrainRate;
        changed = true;
      }
      if (!d.containsKey("battery_sense_enabled")) {
        d["battery_sense_enabled"] = true;
        changed = true;
      }
      if (!d.containsKey("low_battery_shutdown_v")) {
        d["low_battery_shutdown_v"] = DEFAULT_LOW_BATTERY_SHUTDOWN_V;
        changed = true;
      }
      if (!d.containsKey("low_battery_shutdown_resolve_v")) {
        d["low_battery_shutdown_resolve_v"] = d["low_battery_shutdown_v"] | DEFAULT_LOW_BATTERY_SHUTDOWN_RESOLVE_V;
        changed = true;
      }
      uint32_t lastHandledId = prefs.getULong(PREF_LAST_FEED_CMD_ID, 0UL);
      uint32_t snapshotId = computeManualFeedSnapshotId(d.as<JsonVariant>(), lastHandledId);
      if (applyManualFeedTrackingFields(d.as<JsonVariant>(), lastHandledId, snapshotId, false)) {
        changed = true;
      }

      // Ensure a reasonable max_single_feed_kg exists and does not exceed the capacity.
      if (!d.containsKey("max_single_feed_kg")) {
        d["max_single_feed_kg"] = d["max_feeds_capacity_kg"] | DEFAULT_MAX_FEEDS_CAPACITY_KG;
        changed = true;
      } else {
        float cap = d["max_feeds_capacity_kg"] | DEFAULT_MAX_FEEDS_CAPACITY_KG;
        float single = d["max_single_feed_kg"] | DEFAULT_MAX_SINGLE_FEED_KG;
        if (single > cap) {
          d["max_single_feed_kg"] = cap;
          changed = true;
        }
      }

      int selectedIndex = getSelectedGrainTypeIndex(d.as<JsonVariant>());
      const char* selectedType = getGrainTypeNameByIndex(d.as<JsonVariant>(), selectedIndex);
      float selectedRate = getGrainTypeMsPerKgByIndex(d.as<JsonVariant>(), selectedIndex);
      if ((d["grain_type_index"] | -1) != selectedIndex) {
        d["grain_type_index"] = selectedIndex;
        changed = true;
      }
      if (strcmp(d["grain_type"] | "", selectedType) != 0) {
        d["grain_type"] = selectedType;
        changed = true;
      }
      if (!nearlyEqualFloat(d["feed_ms_per_kg"] | 0.0f, selectedRate)) {
        d["feed_ms_per_kg"] = selectedRate;
        changed = true;
      }

      if (changed) {
        String out;
        serializeJson(d, out);
        writeLocalConfigChunked(out);
        cachedCfgStr = out;
        LOG_INFO("Normalized local config defaults without claiming server ownership");
      }
    }
  }

  DynamicJsonDocument syncDoc(8192);
  if (loadConfigDoc(syncDoc)) {
    syncMaxSingleFeedKg(syncDoc.as<JsonVariant>());
  }

  prefs.end();
}

void ensureDailyFeedTotalForToday() {
  static DynamicJsonDocument d(8192);
  if (!loadConfigDoc(d)) return;

  JsonVariant cfgObj = d.as<JsonVariant>();
  String today = getLocalDateYmd();
  const char* storedDate = cfgObj["total_feeds_today_date"] | "";
  bool missingTotal = !cfgObj.containsKey("total_feeds_today_kg");
  bool missingDate = strlen(storedDate) == 0;
  bool dayRolled = !missingDate && strcmp(storedDate, today.c_str()) != 0;

  if (!missingTotal && !missingDate && !dayRolled) return;

  cfgObj["total_feeds_today_kg"] = 0.0f;
  cfgObj["total_feeds_today_date"] = today;

  String out;
  serializeJson(d, out);
  saveLocalConfig(out);
  LOG_INFO("Daily feed total reset date=%s", today.c_str());
}

void addToDailyFeedTotalKg(float dispensedKg) {
  if (dispensedKg <= 0.0f) return;

  ensureDailyFeedTotalForToday();

  static DynamicJsonDocument d(8192);
  if (!loadConfigDoc(d)) return;

  JsonVariant cfgObj = d.as<JsonVariant>();
  float cur = cfgObj["total_feeds_today_kg"] | 0.0f;
  float next = cur + dispensedKg;
  if (next < 0.0f) next = 0.0f;

  cfgObj["total_feeds_today_kg"] = next;
  cfgObj["total_feeds_today_date"] = getLocalDateYmd();

  String out;
  serializeJson(d, out);
  saveLocalConfig(out);
  LOG_INFO("Daily feed total updated current=%.3f add=%.3f next=%.3f",
           cur,
           dispensedKg,
           next);
}
