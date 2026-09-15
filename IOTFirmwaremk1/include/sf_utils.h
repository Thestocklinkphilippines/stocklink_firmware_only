#ifndef SF_UTILS_H
#define SF_UTILS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <time.h>

auto clampf(float v, float minV, float maxV) -> float;
void configureDeviceTimezone();
auto getUtcIsoNow() -> String;
auto parseIsoUtc(const char* iso) -> time_t;
void updateLevelErrorBuzzerAlarm(JsonVariant cfg);
void serviceLevelErrorBuzzer();
void playNetworkConnectedTone();
void playNetworkDisconnectedTone();
void serviceNetworkToneCue(unsigned long nowMs);
bool isNetworkToneCueActive();
void cancelNetworkToneCue();

#endif
