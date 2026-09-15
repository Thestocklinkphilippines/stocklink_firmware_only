#ifndef SF_STORAGE_H
#define SF_STORAGE_H

#include <Arduino.h>
#include <ArduinoJson.h>

auto loadLocalConfig() -> String;
void saveLocalConfig(const String& jsonCfg);
bool isLocalConfigDirty();
void markLocalConfigDirty();
void clearLocalConfigDirty();
auto loadLocalKeypadCalibration() -> String;
bool saveLocalKeypadCalibration(const String& jsonCalibration);
float getMaxSingleFeedKg(JsonVariant cfg);
auto getFeedNowCommandIdFromConfig(JsonVariant cfg) -> uint32_t;
auto getManualFeedSnapshotId(JsonVariant cfg) -> uint32_t;
bool isManualFeedSnapshotSynced(JsonVariant cfg);
int getGrainTypeCount(JsonVariant cfg);
const char* getGrainTypeNameByIndex(JsonVariant cfg, int index);
float getGrainTypeMsPerKgByIndex(JsonVariant cfg, int index);
int findGrainTypeIndex(JsonVariant cfg, const char* grainType);
int getSelectedGrainTypeIndex(JsonVariant cfg);
void saveGrainTypeSelection(JsonVariant cfg, int selectedIndex);
auto readRemainingKg() -> float;
void writeRemainingKg(float v);
auto readLastFeedNowCommandId() -> uint32_t;
void writeLastFeedNowCommandId(uint32_t id);
bool resetManualFeedSnapshotToLastFeedNowCommandId();
auto loadEventOutbox() -> String;
void saveEventOutbox(const String& jsonOutbox);
auto readEventSequence() -> uint32_t;
void writeEventSequence(uint32_t seq);
auto readBufferedEventCount() -> unsigned int;
void ensureDailyFeedTotalForToday();
void addToDailyFeedTotalKg(float dispensedKg);
void ensureLocalDefaults();

#endif
