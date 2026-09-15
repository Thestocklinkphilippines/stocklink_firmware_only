#ifndef SF_SERIAL_H
#define SF_SERIAL_H

#include <Arduino.h>

void printSerialHelp();
void dumpLocalConfigToSerial();
void dumpStateToSerial();
void dumpPrefsToSerial();
void startLoadCellCalibrationSession(float knownKg, const char* reason);
bool isLoadCellCalibrationSessionActive();
void serviceLoadCellCalibrationSession();
void executeSerialCommand(const String& rawCmd);
void handleSerialCommands();

#endif
