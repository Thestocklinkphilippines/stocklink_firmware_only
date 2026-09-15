#ifndef SF_SENSORS_H
#define SF_SENSORS_H

#include <Arduino.h>
#include <ArduinoJson.h>

auto getConfigOrDefault(JsonVariant cfg, const char* key, float fallback) -> float;
auto measureDistanceCm(int trigPin, int echoPin) -> float;
auto distanceToLevelPct(float distanceCm, float tankDepthCm) -> float;
auto distanceToLevelPct(float distanceCm, float fullDistanceCm, float emptyDistanceCm) -> float;
auto waterLevelPctToLiters(float waterPct) -> float;
void initFeederLoadCells();
void serviceFeederLoadCells();
bool isFeederLoadCellCalibrationValid();
bool getFeederLoadCellLatestRaw(long& rawA, long& rawB, unsigned long& ageMs);
bool getFeederLoadCellAveragedRaw(long& rawA, long& rawB, uint8_t& sampleCount);
float getFeederLoadCellObservedSampleRateSps();
long computeFeederLoadCellSpanDelta(long zeroA, long zeroB, long spanA, long spanB);
bool computeFeederLoadCellKgFromCalibration(long rawA, long rawB, float& outKg);
auto getFeederLevelPct(JsonVariant cfg) -> float;
auto getFeederRemainingKg(JsonVariant cfg) -> float;
auto getWaterLevelPct(JsonVariant cfg) -> float;
auto getBatteryVoltageV(JsonVariant cfg) -> float;
auto decodeKeypadAnalog(int adc) -> char;
bool isKeypadInputEnabled();
void setKeypadInputEnabled(bool enabled);
void reloadKeypadCalibration();
void pollKeypad();
auto consumeKeypadKeyEvent() -> char;
auto readMainsPowerPresent() -> bool;
void reconcilePowerAlertStateOnBoot();
void handlePowerFailMonitoring();
auto handleBatteryShutdownMonitoring(JsonVariant cfg) -> bool;
void reportSensorLevels(JsonVariant cfg);
void setKeypadDiagnosticEnabled(bool enabled);

#endif
