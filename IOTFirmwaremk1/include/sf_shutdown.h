#ifndef SF_SHUTDOWN_H
#define SF_SHUTDOWN_H

bool requestForcedBatteryShutdown(float batteryVoltage, float thresholdVoltage, const char* source);

#endif
