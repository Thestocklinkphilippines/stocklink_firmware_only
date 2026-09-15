#define SF_MULTI_CONSOLE_IMPL
#include "sf_multi_console.h"

#include <stdarg.h>

namespace {
void drainInputWithHint(WiFiClient* clients, int maxClients, const char* hintMessage) {
  if (clients == nullptr || hintMessage == nullptr) return;
  for (int i = 0; i < maxClients; i++) {
    if (!clients[i] || !clients[i].connected()) continue;
    if (clients[i].available() <= 0) continue;

    while (clients[i].available() > 0) {
      clients[i].read();
    }
    clients[i].println(hintMessage);
  }
}

void drainInputSilently(WiFiClient* clients, int maxClients) {
  if (clients == nullptr) return;
  for (int i = 0; i < maxClients; i++) {
    if (!clients[i] || !clients[i].connected()) continue;
    while (clients[i].available() > 0) {
      clients[i].read();
    }
  }
}

void printClientBanner(WiFiClient& client,
                       const char* title,
                       const char* line1,
                       const char* line2) {
  if (!client || !client.connected()) return;
  client.println();
  client.println(title);
  client.println(line1);
  client.println(line2);
  client.println();
}
}  // namespace

MultiWirelessConsole SFMultiConsole;

MultiWirelessConsole::MultiWirelessConsole()
    : usb_(&::Serial),
      commandServer_(MULTI_CONSOLE_COMMAND_PORT),
      debugServer_(MULTI_CONSOLE_DEBUG_PORT),
    networkServer_(MULTI_CONSOLE_NETWORK_PORT),
      systemServer_(MULTI_CONSOLE_SYSTEM_PORT),
    keypadServer_(MULTI_CONSOLE_KEYPAD_PORT),
      waterServer_(MULTI_CONSOLE_WATER_PORT),
      keyCandidateServer_(MULTI_CONSOLE_KEY_CANDIDATE_PORT),
      commandClientCount_(0),
      debugClientCount_(0),
    networkClientCount_(0),
      systemClientCount_(0),
    keypadClientCount_(0),
      waterClientCount_(0),
      keyCandidateClientCount_(0),
      started_(false),
      serversRunning_(false),
      baudRate_(115200UL),
      lastReadSource_(InputSource::NONE),
      commandResponseActive_(false) {}

void MultiWirelessConsole::begin(unsigned long baudRate) {
  baudRate_ = baudRate;
  if (usb_ != nullptr) {
    usb_->begin(baudRate_);
  }
  started_ = true;
  service();
}

void MultiWirelessConsole::beginCommandResponse() {
  commandResponseActive_ = true;
}

void MultiWirelessConsole::endCommandResponse() {
  commandResponseActive_ = false;
}

bool MultiWirelessConsole::isCommandResponseActive() const {
  return commandResponseActive_;
}

bool MultiWirelessConsole::lastReadWasCommandPort() const {
  return lastReadSource_ == InputSource::COMMAND_PORT;
}

void MultiWirelessConsole::ensureServersStarted() {
  if (!started_ || serversRunning_) return;
  if (WiFi.status() != WL_CONNECTED) return;

  commandServer_.begin();
  commandServer_.setNoDelay(true);
  
  debugServer_.begin();
  debugServer_.setNoDelay(true);

  networkServer_.begin();
  networkServer_.setNoDelay(true);
  
  systemServer_.begin();
  systemServer_.setNoDelay(true);

  keypadServer_.begin();
  keypadServer_.setNoDelay(true);

  waterServer_.begin();
  waterServer_.setNoDelay(true);

  keyCandidateServer_.begin();
  keyCandidateServer_.setNoDelay(true);
  
  serversRunning_ = true;
}

void MultiWirelessConsole::sendCommandBanner() {
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (commandClients_[i] && commandClients_[i].connected()) {
      sendCommandBanner(commandClients_[i]);
    }
  }
}

void MultiWirelessConsole::sendDebugBanner() {
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (debugClients_[i] && debugClients_[i].connected()) {
      sendDebugBanner(debugClients_[i]);
    }
  }
}

void MultiWirelessConsole::sendNetworkBanner() {
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (networkClients_[i] && networkClients_[i].connected()) {
      sendNetworkBanner(networkClients_[i]);
    }
  }
}

void MultiWirelessConsole::sendSystemBanner() {
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (systemClients_[i] && systemClients_[i].connected()) {
      sendSystemBanner(systemClients_[i]);
    }
  }
}

void MultiWirelessConsole::sendKeypadBanner() {
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (keypadClients_[i] && keypadClients_[i].connected()) {
      sendKeypadBanner(keypadClients_[i]);
    }
  }
}

void MultiWirelessConsole::sendWaterBanner() {
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (waterClients_[i] && waterClients_[i].connected()) {
      sendWaterBanner(waterClients_[i]);
    }
  }
}

void MultiWirelessConsole::sendBanners() {
  sendCommandBanner();
  sendDebugBanner();
  sendNetworkBanner();
  sendSystemBanner();
  sendKeypadBanner();
  sendWaterBanner();
}

void MultiWirelessConsole::sendCommandBanner(WiFiClient& client) {
  printClientBanner(client,
                    "=== ESP32 Command Console (Port 2323) ===",
                    "Send commands here. Responses appear on this port.",
                    "Type 'help' for available commands.");
}

void MultiWirelessConsole::sendDebugBanner(WiFiClient& client) {
  printClientBanner(client,
                    "=== ESP32 Debug Console (Port 2324) ===",
                    "Output-only stream. All debug logs and general output appear here.",
                    "Use port 2323 for commands.");
}

void MultiWirelessConsole::sendNetworkBanner(WiFiClient& client) {
  printClientBanner(client,
                    "=== ESP32 Network Console (Port 2326) ===",
                    "Output-only stream. Network-related logs and events appear here.",
                    "Use port 2324 for general debug output.");
}

void MultiWirelessConsole::sendSystemBanner(WiFiClient& client) {
  printClientBanner(client,
                    "=== ESP32 System Console (Port 2325) ===",
                    "Output-only stream. Critical system events and alerts appear here.",
                    "Use port 2323 for commands.");
}

void MultiWirelessConsole::sendKeypadBanner(WiFiClient& client) {
  printClientBanner(client,
                    "=== ESP32 Keypad Console (Port 2327) ===",
                    "Output-only stream. Keypad input and calibration events appear here.",
                    "Use port 2324 for general debug output.");
}

void MultiWirelessConsole::sendWaterBanner(WiFiClient& client) {
  printClientBanner(client,
                    "=== ESP32 Sensor Console (Port 2328) ===",
                    "Output-only stream. Water ultrasonic and feed load-cell readings appear here.",
                    "Use port 2324 for general debug output.");
}

const char* MultiWirelessConsole::portLabel(uint16_t port) const {
  switch (port) {
    case MULTI_CONSOLE_COMMAND_PORT:
      return "command";
    case MULTI_CONSOLE_DEBUG_PORT:
      return "debug";
    case MULTI_CONSOLE_NETWORK_PORT:
      return "network";
    case MULTI_CONSOLE_SYSTEM_PORT:
      return "system";
    case MULTI_CONSOLE_KEYPAD_PORT:
      return "keypad";
    case MULTI_CONSOLE_WATER_PORT:
      return "water";
    case MULTI_CONSOLE_KEY_CANDIDATE_PORT:
      return "key_candidate";
    default:
      return "unknown";
  }
}

void MultiWirelessConsole::logClientEvent(const char* action, uint16_t port, int slot, const WiFiClient& client) {
  if (usb_ == nullptr || action == nullptr) return;
  usb_->printf("[INFO] Console %s port=%u(%s) slot=%d remote=%s:%u\n",
               action,
               port,
               portLabel(port),
               slot,
               client.remoteIP().toString().c_str(),
               client.remotePort());
}

void MultiWirelessConsole::acceptClients() {
  if (!serversRunning_) return;

  // Accept command port clients
  WiFiClient incoming = commandServer_.available();
  if (incoming) {
    // Find empty slot or replace oldest
    for (int i = 0; i < kMaxClientsPerPort; i++) {
      if (!commandClients_[i] || !commandClients_[i].connected()) {
        if (commandClients_[i]) commandClients_[i].stop();
        commandClients_[i] = incoming;
        commandClients_[i].setNoDelay(true);
        commandClientCount_++;
        sendCommandBanner(commandClients_[i]);
        logClientEvent("connect", MULTI_CONSOLE_COMMAND_PORT, i, commandClients_[i]);
        return;
      }
    }
    logClientEvent("reject", MULTI_CONSOLE_COMMAND_PORT, -1, incoming);
    incoming.stop();
  }

  // Accept debug port clients
  incoming = debugServer_.available();
  if (incoming) {
    for (int i = 0; i < kMaxClientsPerPort; i++) {
      if (!debugClients_[i] || !debugClients_[i].connected()) {
        if (debugClients_[i]) debugClients_[i].stop();
        debugClients_[i] = incoming;
        debugClients_[i].setNoDelay(true);
        debugClientCount_++;
        sendDebugBanner(debugClients_[i]);
        logClientEvent("connect", MULTI_CONSOLE_DEBUG_PORT, i, debugClients_[i]);
        return;
      }
    }
    logClientEvent("reject", MULTI_CONSOLE_DEBUG_PORT, -1, incoming);
    incoming.stop();
  }

  // Accept network port clients
  incoming = networkServer_.available();
  if (incoming) {
    for (int i = 0; i < kMaxClientsPerPort; i++) {
      if (!networkClients_[i] || !networkClients_[i].connected()) {
        if (networkClients_[i]) networkClients_[i].stop();
        networkClients_[i] = incoming;
        networkClients_[i].setNoDelay(true);
        networkClientCount_++;
        sendNetworkBanner(networkClients_[i]);
        logClientEvent("connect", MULTI_CONSOLE_NETWORK_PORT, i, networkClients_[i]);
        return;
      }
    }
    logClientEvent("reject", MULTI_CONSOLE_NETWORK_PORT, -1, incoming);
    incoming.stop();
  }

  // Accept system port clients
  incoming = systemServer_.available();
  if (incoming) {
    for (int i = 0; i < kMaxClientsPerPort; i++) {
      if (!systemClients_[i] || !systemClients_[i].connected()) {
        if (systemClients_[i]) systemClients_[i].stop();
        systemClients_[i] = incoming;
        systemClients_[i].setNoDelay(true);
        systemClientCount_++;
        sendSystemBanner(systemClients_[i]);
        logClientEvent("connect", MULTI_CONSOLE_SYSTEM_PORT, i, systemClients_[i]);
        return;
      }
    }
    logClientEvent("reject", MULTI_CONSOLE_SYSTEM_PORT, -1, incoming);
    incoming.stop();
  }

  // Accept keypad port clients
  incoming = keypadServer_.available();
  if (incoming) {
    for (int i = 0; i < kMaxClientsPerPort; i++) {
      if (!keypadClients_[i] || !keypadClients_[i].connected()) {
        if (keypadClients_[i]) keypadClients_[i].stop();
        keypadClients_[i] = incoming;
        keypadClients_[i].setNoDelay(true);
        keypadClientCount_++;
        sendKeypadBanner(keypadClients_[i]);
        logClientEvent("connect", MULTI_CONSOLE_KEYPAD_PORT, i, keypadClients_[i]);
        return;
      }
    }
    logClientEvent("reject", MULTI_CONSOLE_KEYPAD_PORT, -1, incoming);
    incoming.stop();
  }

  // Accept water sensor port clients
  incoming = waterServer_.available();
  if (incoming) {
    for (int i = 0; i < kMaxClientsPerPort; i++) {
      if (!waterClients_[i] || !waterClients_[i].connected()) {
        if (waterClients_[i]) waterClients_[i].stop();
        waterClients_[i] = incoming;
        waterClients_[i].setNoDelay(true);
        waterClientCount_++;
        sendWaterBanner(waterClients_[i]);
        logClientEvent("connect", MULTI_CONSOLE_WATER_PORT, i, waterClients_[i]);
        return;
      }
    }
    logClientEvent("reject", MULTI_CONSOLE_WATER_PORT, -1, incoming);
    incoming.stop();
  }

  // Port 2330 intentionally sends no banner so it remains candidate-only.
  incoming = keyCandidateServer_.available();
  if (incoming) {
    for (int i = 0; i < kMaxClientsPerPort; i++) {
      if (!keyCandidateClients_[i] || !keyCandidateClients_[i].connected()) {
        if (keyCandidateClients_[i]) keyCandidateClients_[i].stop();
        keyCandidateClients_[i] = incoming;
        keyCandidateClients_[i].setNoDelay(true);
        keyCandidateClientCount_++;
        logClientEvent("connect", MULTI_CONSOLE_KEY_CANDIDATE_PORT, i, keyCandidateClients_[i]);
        return;
      }
    }
    logClientEvent("reject", MULTI_CONSOLE_KEY_CANDIDATE_PORT, -1, incoming);
    incoming.stop();
  }
}

void MultiWirelessConsole::dropDisconnectedClients() {
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (commandClients_[i] && !commandClients_[i].connected()) {
      logClientEvent("disconnect", MULTI_CONSOLE_COMMAND_PORT, i, commandClients_[i]);
      commandClients_[i].stop();
      commandClientCount_--;
    }
    if (debugClients_[i] && !debugClients_[i].connected()) {
      logClientEvent("disconnect", MULTI_CONSOLE_DEBUG_PORT, i, debugClients_[i]);
      debugClients_[i].stop();
      debugClientCount_--;
    }
    if (networkClients_[i] && !networkClients_[i].connected()) {
      logClientEvent("disconnect", MULTI_CONSOLE_NETWORK_PORT, i, networkClients_[i]);
      networkClients_[i].stop();
      networkClientCount_--;
    }
    if (systemClients_[i] && !systemClients_[i].connected()) {
      logClientEvent("disconnect", MULTI_CONSOLE_SYSTEM_PORT, i, systemClients_[i]);
      systemClients_[i].stop();
      systemClientCount_--;
    }
    if (keypadClients_[i] && !keypadClients_[i].connected()) {
      logClientEvent("disconnect", MULTI_CONSOLE_KEYPAD_PORT, i, keypadClients_[i]);
      keypadClients_[i].stop();
      keypadClientCount_--;
    }
    if (waterClients_[i] && !waterClients_[i].connected()) {
      logClientEvent("disconnect", MULTI_CONSOLE_WATER_PORT, i, waterClients_[i]);
      waterClients_[i].stop();
      waterClientCount_--;
    }
    if (keyCandidateClients_[i] && !keyCandidateClients_[i].connected()) {
      logClientEvent("disconnect", MULTI_CONSOLE_KEY_CANDIDATE_PORT, i, keyCandidateClients_[i]);
      keyCandidateClients_[i].stop();
      keyCandidateClientCount_--;
    }
  }
}

void MultiWirelessConsole::broadcastToPort(WiFiServer& server, WiFiClient* clients, 
                                          int maxClients, const uint8_t* buffer, size_t size) {
  for (int i = 0; i < maxClients; i++) {
    if (clients[i] && clients[i].connected()) {
      clients[i].write(buffer, size);
    }
  }
}

void MultiWirelessConsole::service() {
  ensureServersStarted();
  if (!serversRunning_) return;

  if (WiFi.status() != WL_CONNECTED) {
    dropDisconnectedClients();
    return;
  }

  dropDisconnectedClients();
  acceptClients();
  drainInputWithHint(debugClients_,
                     kMaxClientsPerPort,
                     "[INFO] Port 2324 is output-only. Use port 2323 for commands.");
  drainInputWithHint(systemClients_,
                     kMaxClientsPerPort,
                     "[INFO] Port 2325 is output-only. Use port 2323 for commands.");
  drainInputWithHint(waterClients_,
                     kMaxClientsPerPort,
                     "[INFO] Port 2328 is output-only. Use port 2323 for commands.");
  drainInputSilently(keyCandidateClients_, kMaxClientsPerPort);
}

int MultiWirelessConsole::available() {
  service();
  // Check command port for input (bi-directional)
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (commandClients_[i] && commandClients_[i].connected() && commandClients_[i].available() > 0) {
      lastReadSource_ = InputSource::COMMAND_PORT;
      return commandClients_[i].available();
    }
  }
  lastReadSource_ = InputSource::USB;
  return usb_ != nullptr ? usb_->available() : 0;
}

int MultiWirelessConsole::read() {
  service();
  // Read from command port (bi-directional)
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (commandClients_[i] && commandClients_[i].connected() && commandClients_[i].available() > 0) {
      lastReadSource_ = InputSource::COMMAND_PORT;
      return commandClients_[i].read();
    }
  }
  lastReadSource_ = InputSource::USB;
  return usb_ != nullptr ? usb_->read() : -1;
}

int MultiWirelessConsole::peek() {
  service();
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (commandClients_[i] && commandClients_[i].connected() && commandClients_[i].available() > 0) {
      lastReadSource_ = InputSource::COMMAND_PORT;
      return commandClients_[i].peek();
    }
  }
  lastReadSource_ = InputSource::USB;
  return usb_ != nullptr ? usb_->peek() : -1;
}

void MultiWirelessConsole::flush() {
  if (usb_ != nullptr) {
    usb_->flush();
  }
  for (int i = 0; i < kMaxClientsPerPort; i++) {
    if (commandClients_[i] && commandClients_[i].connected()) {
      commandClients_[i].flush();
    }
    if (debugClients_[i] && debugClients_[i].connected()) {
      debugClients_[i].flush();
    }
    if (systemClients_[i] && systemClients_[i].connected()) {
      systemClients_[i].flush();
    }
    if (waterClients_[i] && waterClients_[i].connected()) {
      waterClients_[i].flush();
    }
    if (keyCandidateClients_[i] && keyCandidateClients_[i].connected()) {
      keyCandidateClients_[i].flush();
    }
  }
}

size_t MultiWirelessConsole::write(uint8_t byte) {
  if (commandResponseActive_) {
    return writeCommand(&byte, 1);
  }
  // Default write goes to debug port (for backward compatibility with Serial.print)
  return writeDebug(&byte, 1);
}

size_t MultiWirelessConsole::write(const uint8_t* buffer, size_t size) {
  if (commandResponseActive_) {
    return writeCommand(buffer, size);
  }
  // Default write goes to debug port
  return writeDebug(buffer, size);
}

int MultiWirelessConsole::printf(const char* format, ...) {
  if (commandResponseActive_) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len < 0) return len;

    size_t toWrite = (size_t)len;
    if (toWrite > sizeof(buffer) - 1) {
      toWrite = sizeof(buffer) - 1;
    }
    writeCommand(reinterpret_cast<const uint8_t*>(buffer), toWrite);
    return len;
  }

  // Default printf goes to debug port
  return debugPrintf(format);
}

size_t MultiWirelessConsole::writeDebug(const uint8_t* buffer, size_t size) {
  if (usb_ != nullptr) {
    usb_->write(buffer, size);
  }
  broadcastToPort(debugServer_, debugClients_, kMaxClientsPerPort, buffer, size);
  return size;
}

size_t MultiWirelessConsole::writeNetwork(const uint8_t* buffer, size_t size) {
  if (usb_ != nullptr) {
    usb_->write(buffer, size);
  }
  broadcastToPort(networkServer_, networkClients_, kMaxClientsPerPort, buffer, size);
  return size;
}

size_t MultiWirelessConsole::writeSystem(const uint8_t* buffer, size_t size) {
  if (usb_ != nullptr) {
    usb_->write(buffer, size);
  }
  broadcastToPort(systemServer_, systemClients_, kMaxClientsPerPort, buffer, size);
  return size;
}

size_t MultiWirelessConsole::writeKeypad(const uint8_t* buffer, size_t size) {
  broadcastToPort(keypadServer_, keypadClients_, kMaxClientsPerPort, buffer, size);
  return size;
}

size_t MultiWirelessConsole::writeWater(const uint8_t* buffer, size_t size) {
  if (usb_ != nullptr) {
    usb_->write(buffer, size);
  }
  broadcastToPort(waterServer_, waterClients_, kMaxClientsPerPort, buffer, size);
  return size;
}

size_t MultiWirelessConsole::writeCommand(const uint8_t* buffer, size_t size) {
  if (usb_ != nullptr) {
    usb_->write(buffer, size);
  }
  broadcastToPort(commandServer_, commandClients_, kMaxClientsPerPort, buffer, size);
  return size;
}

void MultiWirelessConsole::writeKeyCandidate(char candidate) {
  if (candidate == '\0') return;
  char buffer[17];
  int len = snprintf(buffer, sizeof(buffer), "Key detected: %c\n", candidate);
  if (len <= 0) return;
  broadcastToPort(keyCandidateServer_,
                  keyCandidateClients_,
                  kMaxClientsPerPort,
                  reinterpret_cast<const uint8_t*>(buffer),
                  static_cast<size_t>(len));
}

int MultiWirelessConsole::debugPrintf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len < 0) return len;

  size_t toWrite = (size_t)len;
  if (toWrite > sizeof(buffer) - 1) {
    toWrite = sizeof(buffer) - 1;
  }
  writeDebug(reinterpret_cast<const uint8_t*>(buffer), toWrite);
  return len;
}

int MultiWirelessConsole::networkPrintf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len < 0) return len;

  size_t toWrite = (size_t)len;
  if (toWrite > sizeof(buffer) - 1) {
    toWrite = sizeof(buffer) - 1;
  }
  writeNetwork(reinterpret_cast<const uint8_t*>(buffer), toWrite);
  return len;
}

int MultiWirelessConsole::systemPrintf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len < 0) return len;

  size_t toWrite = (size_t)len;
  if (toWrite > sizeof(buffer) - 1) {
    toWrite = sizeof(buffer) - 1;
  }
  writeSystem(reinterpret_cast<const uint8_t*>(buffer), toWrite);
  return len;
}

int MultiWirelessConsole::keypadPrintf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len < 0) return len;

  size_t toWrite = (size_t)len;
  if (toWrite > sizeof(buffer) - 1) {
    toWrite = sizeof(buffer) - 1;
  }
  writeKeypad(reinterpret_cast<const uint8_t*>(buffer), toWrite);
  return len;
}

int MultiWirelessConsole::waterPrintf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len < 0) return len;

  size_t toWrite = (size_t)len;
  if (toWrite > sizeof(buffer) - 1) {
    toWrite = sizeof(buffer) - 1;
  }
  writeWater(reinterpret_cast<const uint8_t*>(buffer), toWrite);
  return len;
}

int MultiWirelessConsole::commandPrintf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len < 0) return len;

  size_t toWrite = (size_t)len;
  if (toWrite > sizeof(buffer) - 1) {
    toWrite = sizeof(buffer) - 1;
  }
  writeCommand(reinterpret_cast<const uint8_t*>(buffer), toWrite);
  return len;
}

void serviceMultiWirelessConsole() {
  SFMultiConsole.service();
}
