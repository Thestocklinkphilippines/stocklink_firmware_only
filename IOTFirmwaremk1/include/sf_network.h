#ifndef SF_NETWORK_H
#define SF_NETWORK_H

#include <Arduino.h>
#include <ArduinoJson.h>

void initAsyncHttp();
void serviceAsyncHttp();
auto getAsyncHttpPendingCount() -> unsigned int;
void sendLog(const char* type, JsonVariant payload);
void sendAlert(const char* alertType);
auto sendFeedNowAck(uint32_t commandId, const char* status, const char* reason) -> bool;
bool sendSensorState(const String& body);
bool queueBufferedRequest(const String& endpoint, const String& body, const char* kind, bool critical);
void syncWithServer();
void connectWiFi();
void setupOTA();
void serviceOTA();
bool isOTAInProgress();
void serviceBufferedOutbox();
bool flushCriticalOutboxBeforeShutdown(uint8_t transportAttempts, unsigned long timeoutMs);

#endif
