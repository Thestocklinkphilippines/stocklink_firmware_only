#ifndef SF_ACTUATORS_H
#define SF_ACTUATORS_H

#include <stdint.h>

#include <ArduinoJson.h>

auto isFeedSufficient(float requiredKg, JsonVariant cfg) -> bool;
auto isFeedSufficientIncludingQueued(float requiredKg, JsonVariant cfg) -> bool;
void setFeedMotorEnabled(bool enabled);
void setWaterSolenoidEnabled(bool enabled);
void setBatteryShutdownRelayEnabled(bool enabled);
auto computeFeedMotorRunMs(float amountKg, JsonVariant cfg) -> unsigned long;
auto dispenseFeed(float amountKg, JsonVariant cfg) -> bool;
auto dispenseFeedManual(float amountKg, JsonVariant cfg, const char* source) -> bool;
auto dispenseFeedScheduled(float amountKg, JsonVariant cfg, JsonVariant schedule) -> bool;
auto dispenseFeedNow(float amountKg, JsonVariant cfg, uint32_t commandId) -> bool;
auto isFeedNowCommandQueuedOrActive(uint32_t commandId) -> bool;
auto getPendingFeedRequestCount() -> unsigned int;
auto getReservedFeedKg() -> float;
auto requestWaterRefillAttempt(float baselineWaterPct) -> bool;
auto isWaterRefillActuatorActive() -> bool;
void finishWaterRefillAtHighThreshold(float waterPct);
void serviceActuators(JsonVariant cfg, unsigned long nowMs);
void stopActuatorsForSafety(const char* reason);

#endif
