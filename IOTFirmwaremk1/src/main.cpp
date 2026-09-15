#include <Arduino.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>

#include "sf_actuators.h"
#include "sf_adc.h"
#include "sf_config.h"
#include "sf_control_panel.h"
#include "sf_debug.h"
#include "sf_globals.h"
#include "sf_multi_console.h"
#include "sf_network.h"
#include "sf_pins.h"
#include "sf_scheduler.h"
#include "sf_sensors.h"
#include "sf_serial.h"
#include "sf_shutdown.h"
#include "sf_storage.h"
#include "sf_utils.h"

static LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);
// Use compile-time constant from configuration header

static unsigned long gLastGrainTypeVerboseMs = 0;
static unsigned long gLastNtpAttemptMs = 0;
static unsigned long gLastLowBatteryCheckLogMs = 0;
static bool gNtpSyncRequested = false;
static bool gNtpSyncComplete = false;
static bool gWiFiToneStateKnown = false;
static bool gLastWiFiToneConnected = false;
static bool gWiFiToneEverConnected = false;
static bool gActuatorsStoppedForOta = false;

static const time_t kValidNtpEpochThreshold = 100000;
static float gLowBatteryShutdownThresholdV = DEFAULT_LOW_BATTERY_SHUTDOWN_V;
static bool gLowBatteryShutdownForced = false;
static const char* gLowBatteryShutdownSource = "battery_threshold";

static bool isSystemTimeValid() {
  return time(nullptr) >= kValidNtpEpochThreshold;
}

static void kickOffNtpSync() {
  configureDeviceTimezone();
  configTzTime(DEVICE_TZ_POSIX, "pool.ntp.org", "time.nist.gov");
  gNtpSyncRequested = true;
}

static void requestNtpRefresh() {
  gNtpSyncRequested = false;
  gNtpSyncComplete = false;
  gLastNtpAttemptMs = 0;
}

static void logTimeReady() {
  time_t now = time(nullptr);
  struct tm utcInfo;
  struct tm localInfo;
  gmtime_r(&now, &utcInfo);
  localtime_r(&now, &localInfo);

  char utcBuf[32];
  char localBuf[32];
  strftime(utcBuf, sizeof(utcBuf), "%Y-%m-%dT%H:%M:%SZ", &utcInfo);
  strftime(localBuf, sizeof(localBuf), "%Y-%m-%d %H:%M:%S %Z", &localInfo);
  LOG_INFO("NTP time ready epoch=%ld utc=%s local=%s", (long)now, utcBuf, localBuf);
}

static void serviceTimeSync(unsigned long nowMs) {
  configureDeviceTimezone();

  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected && (!gNtpSyncRequested || !gNtpSyncComplete)) {
    if (gLastNtpAttemptMs == 0 || nowMs - gLastNtpAttemptMs >= NTP_SYNC_RETRY_INTERVAL_MS) {
      gLastNtpAttemptMs = nowMs;
      LOG_INFO("NTP sync attempt started");
      kickOffNtpSync();
    }
  }

  if (gNtpSyncComplete) {
    return;
  }

  if (isSystemTimeValid()) {
    gNtpSyncComplete = true;
    logTimeReady();
    return;
  }

  if (!wifiConnected) return;
}

static void serviceNetworkConnectionTone() {
  bool connected = WiFi.status() == WL_CONNECTED;

  if (!gWiFiToneStateKnown) {
    gWiFiToneStateKnown = true;
    gLastWiFiToneConnected = connected;
    if (connected) {
      gWiFiToneEverConnected = true;
      LOG_INFO("Internet connection active; playing connected tone");
      playNetworkConnectedTone();
    }
    return;
  }

  if (connected == gLastWiFiToneConnected) {
    return;
  }

  gLastWiFiToneConnected = connected;
  if (connected) {
    gWiFiToneEverConnected = true;
    requestNtpRefresh();
    LOG_INFO("Internet connection restored; playing connected tone");
    playNetworkConnectedTone();
  } else if (gWiFiToneEverConnected) {
    LOG_WARN("Internet connection lost; playing disconnected tone");
    playNetworkDisconnectedTone();
  }
}

static void setShutdownRelayOpen(bool open) {
  digitalWrite(PIN_BATTERY_SHUTDOWN_RELAY, open ? HIGH : LOW);
}

static void initShutdownRelay() {
  pinMode(PIN_BATTERY_SHUTDOWN_RELAY, OUTPUT);
  setShutdownRelayOpen(false);
  LOG_INFO("Shutdown relay initialized pin=%d activeHigh=1", PIN_BATTERY_SHUTDOWN_RELAY);
}

static float getLowBatteryShutdownThreshold(JsonVariant cfg) {
  float threshold = getConfigOrDefault(cfg, "low_battery_shutdown_v", DEFAULT_LOW_BATTERY_SHUTDOWN_V);
  if (threshold <= 0.0f) {
    threshold = DEFAULT_LOW_BATTERY_SHUTDOWN_V;
  }
  return threshold;
}

static bool isLowBatteryShutdownRequired(JsonVariant cfg, float& batteryVoltage) {
  batteryVoltage = getBatteryVoltageV(cfg);
  if (batteryVoltage < 0.0f) return false;
  gLowBatteryShutdownThresholdV = getLowBatteryShutdownThreshold(cfg);
  unsigned long nowMs = millis();
  if (gLastLowBatteryCheckLogMs == 0UL || nowMs - gLastLowBatteryCheckLogMs >= BATTERY_READ_INTERVAL_MS) {
    gLastLowBatteryCheckLogMs = nowMs;
    LOG_DEBUG("Low battery shutdown check batt=%.2fV threshold=%.2fV",
              batteryVoltage,
              gLowBatteryShutdownThresholdV);
  }
  return batteryVoltage <= gLowBatteryShutdownThresholdV;
}

static void markLowBatteryShutdownPending(float batteryVoltage,
                                          float thresholdVoltage,
                                          unsigned long nowMs,
                                          bool forced,
                                          const char* source) {
  if (state.lowBatteryShutdownPending) return;
  state.lowBatteryShutdownPending = true;
  state.lowBatteryShutdownTriggered = false;
  state.lowBatteryShutdownVoltage = batteryVoltage;
  state.lowBatteryShutdownDetectedMs = nowMs;
  gLowBatteryShutdownThresholdV = thresholdVoltage;
  gLowBatteryShutdownForced = forced;
  gLowBatteryShutdownSource = source != nullptr ? source : "unknown";
  stopActuatorsForSafety(forced ? "forced_shutdown" : "low_battery_shutdown");
  if (forced) {
    LOG_WARN("Forced battery shutdown requested source=%s batt=%.2fV threshold=%.2fV; preparing shutdown",
             gLowBatteryShutdownSource,
             batteryVoltage,
             gLowBatteryShutdownThresholdV);
  } else {
    LOG_WARN("Low battery detected batt=%.2fV threshold=%.2fV; preparing shutdown",
             batteryVoltage,
             gLowBatteryShutdownThresholdV);
  }
}

bool requestForcedBatteryShutdown(float batteryVoltage, float thresholdVoltage, const char* source) {
  if (state.lowBatteryShutdownPending || state.lowBatteryShutdownTriggered) return false;
  if (thresholdVoltage <= 0.0f) thresholdVoltage = DEFAULT_LOW_BATTERY_SHUTDOWN_V;
  markLowBatteryShutdownPending(batteryVoltage, thresholdVoltage, millis(), true, source);
  return true;
}

static bool serviceLowBatteryShutdown(JsonVariant cfg, unsigned long nowMs) {
  float batteryVoltage = -1.0f;
  if (!state.lowBatteryShutdownPending) {
    if (!isLowBatteryShutdownRequired(cfg, batteryVoltage)) return false;
    markLowBatteryShutdownPending(
        batteryVoltage,
        gLowBatteryShutdownThresholdV,
        nowMs,
        false,
        "battery_threshold");
  }

  serviceBufferedOutbox();

  if (!state.lowBatteryShutdownTriggered) {
    StaticJsonDocument<256> payload;
    payload["event"] = "low_battery_shutdown";
    payload["battery_voltage_v"] = state.lowBatteryShutdownVoltage;
    payload["shutdown_threshold_v"] = gLowBatteryShutdownThresholdV;
    payload["shutdown_requested_ms"] = state.lowBatteryShutdownDetectedMs;
    payload["shutdown_requested_at"] = getUtcIsoNow();
    payload["forced"] = gLowBatteryShutdownForced;
    payload["source"] = gLowBatteryShutdownSource;

    sendAlert("low_battery_shutdown");
    sendLog("power", payload.as<JsonVariant>());

    bool shutdownEventsDelivered = flushCriticalOutboxBeforeShutdown(
        LOW_BATTERY_SHUTDOWN_NETWORK_ATTEMPTS,
        LOW_BATTERY_SHUTDOWN_NETWORK_FLUSH_TIMEOUT_MS);
    if (shutdownEventsDelivered) {
      LOG_INFO("Low battery shutdown events delivered before relay assertion");
    } else {
      LOG_WARN("Low battery shutdown proceeding with persisted unsent events");
    }

    setShutdownRelayOpen(true);
    state.lowBatteryShutdownTriggered = true;
    LOG_ERROR("Low battery shutdown relay asserted at %.2fV", state.lowBatteryShutdownVoltage);
  }

  return true;
}

static void serviceWaterRefillControl(JsonVariant cfg, unsigned long nowMs) {
  float waterLow = getConfigOrDefault(cfg, "water_low_threshold_pct", DEFAULT_WATER_LOW_THRESHOLD_PCT);
  float waterHigh = getConfigOrDefault(cfg, "water_high_threshold_pct", DEFAULT_WATER_HIGH_THRESHOLD_PCT);
  float waterPct = state.lastWaterLevelPct;

  LOG_INFO("Water hysteresis check pct=%.1f low=%.1f high=%.1f isRefilling=%d",
           waterPct,
           waterLow,
           waterHigh,
           state.isRefilling ? 1 : 0);

  if (!state.isRefilling) {
    if (waterPct > waterLow) return;
    state.isRefilling = true;
    state.lastWaterRefillAttemptMs = 0UL;
    LOG_WARN("Water low threshold crossed; start refill");
  }

  if (waterPct >= waterHigh) {
    finishWaterRefillAtHighThreshold(waterPct);
    return;
  }

  if (isWaterRefillActuatorActive()) return;

  bool refillAttemptDue = state.lastWaterRefillAttemptMs == 0UL ||
                          nowMs - state.lastWaterRefillAttemptMs >= WATER_CHECK_INTERVAL_MS;
  if (!refillAttemptDue) {
    unsigned long remainingMs = WATER_CHECK_INTERVAL_MS - (nowMs - state.lastWaterRefillAttemptMs);
    LOG_DEBUG("Water refill cooldown active remaining_ms=%lu", remainingMs);
    return;
  }

  if (waterPct < waterHigh) {
    LOG_WARN("Water refill below high threshold; attempt refill pct=%.1f high=%.1f",
             waterPct,
             waterHigh);
    if (!requestWaterRefillAttempt(waterPct)) {
      LOG_WARN("Water refill attempt request rejected; actuator already active");
    }
  }
}

static void serviceGrainTypeVerbose(JsonVariant cfg, unsigned long nowMs) {
  if (nowMs - gLastGrainTypeVerboseMs < 1000UL) return;
  gLastGrainTypeVerboseMs = nowMs;

  int selectedIndex = getSelectedGrainTypeIndex(cfg);
  const char* grainType = getGrainTypeNameByIndex(cfg, selectedIndex);

  SFMultiConsole.systemPrintf("[TEMP] Grain type[%d] set to: %s\n", selectedIndex, grainType);
}

static unsigned long getFeederLoadCellTargetIntervalMs() {
  if (FEED_LC_TARGET_SAMPLE_RATE_SPS == 0U) return MAIN_LOOP_DELAY_MS;
  unsigned long intervalMs = 1000UL / (unsigned long)FEED_LC_TARGET_SAMPLE_RATE_SPS;
  return intervalMs > 0UL ? intervalMs : 1UL;
}

static void idleDelayWithFeederLoadCellService(unsigned long delayMs) {
  unsigned long startMs = millis();
  unsigned long lastServiceMs = startMs;
  unsigned long targetIntervalMs = getFeederLoadCellTargetIntervalMs();

  while (millis() - startMs < delayMs) {
    unsigned long nowMs = millis();
    if (nowMs - lastServiceMs >= targetIntervalMs) {
      lastServiceMs = nowMs;
      serviceFeederLoadCells();
    }
    delay(1);
  }
}

void setupPins() {
  initFeederLoadCells();
  pinMode(PIN_WATER_TRIG, OUTPUT);
  pinMode(PIN_WATER_ECHO, INPUT);

  pinMode(PIN_FEED_MOTOR, OUTPUT);
  pinMode(PIN_WATER_SOLENOID, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  initShutdownRelay();
  pinMode(PIN_MAINS_SENSE_ADC, INPUT);
  pinMode(PIN_BATTERY_ADC, INPUT);

  setWaterSolenoidEnabled(false);
  digitalWrite(PIN_BUZZER, LOW);
  noTone(PIN_BUZZER);
  setFeedMotorEnabled(false);
  // LCD backlight controlled via I2C; keep it enabled via driver
  LOG_INFO("Feed motor polarity activeHigh=%d", FEED_MOTOR_ACTIVE_HIGH ? 1 : 0);
  LOG_INFO("Water solenoid polarity activeHigh=%d", WATER_SOLENOID_ACTIVE_HIGH ? 1 : 0);

  analogReadResolution(12);
  LOG_INFO("Pins initialized");
}

void setupTime() {
  configureDeviceTimezone();
  if (WiFi.status() != WL_CONNECTED) {
    LOG_WARN("NTP sync deferred; WiFi not connected");
    return;
  }

  gLastNtpAttemptMs = millis();
  kickOffNtpSync();
  LOG_INFO("NTP sync started asynchronously");
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  LOG_INFO("Smart feeder booting");
  LOG_INFO("Build mode verbose=%d", SF_VERBOSE_SERIAL ? 1 : 0);

  setupPins();
  initAdcSystem();
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  LOG_INFO("I2C initialized SDA=%d SCL=%d", PIN_LCD_SDA, PIN_LCD_SCL);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Feeder MK1");
  lcd.setCursor(0, 1);
  lcd.print("LCD test: ONLINE");
  lcd.setCursor(0, 2);
  lcd.print("SDA=21 SCL=22");
  lcd.setCursor(0, 3);
  lcd.print("Addr 0x27 20x4");
  LOG_INFO("LCD initialized addr=0x%02X cols=%d rows=%d", LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);
  initControlPanel(&lcd);

  connectWiFi();
  initAsyncHttp();
  reconcilePowerAlertStateOnBoot();
  setupOTA();
  setupTime();
  ensureLocalDefaults();
  reloadKeypadCalibration();

  unsigned long setupNowMs = millis();
  state.lastSyncMs = setupNowMs;
  state.lastScheduleCheckMs = setupNowMs;
  state.lastWaterRefillAttemptMs = 0UL;
  // Phase routine network reports between the 5-second config polling windows.
  state.lastSensorReportMs = setupNowMs - (SENSOR_REPORT_INTERVAL_MS - 2500UL);
  state.lastWiFiReconnectAttemptMs = setupNowMs;
  state.lastMainsCheckMs = setupNowMs;
  state.lastKeypadPollMs = setupNowMs;
  state.lastConfigRefreshMs = 0;
  state.lastHeartbeatLogMs = setupNowMs - (HEARTBEAT_LOG_INTERVAL_MS - 7500UL);
  cachedCfgStr = loadLocalConfig();
  state.bufferedEventCount = readBufferedEventCount();
  state.mainsPowerPresent = SF_ENABLE_MAINS_MONITOR ? readMainsPowerPresent() : true;
  state.lowBatteryShutdownPending = false;
  state.lowBatteryShutdownTriggered = false;
  state.lowBatteryShutdownVoltage = -1.0f;
  state.lowBatteryShutdownDetectedMs = 0UL;

  LOG_INFO("Setup complete mainsPresent=%d", state.mainsPowerPresent ? 1 : 0);
  printSerialHelp();
  if (!isFeederLoadCellCalibrationValid()) {
    startLoadCellCalibrationSession(0.0f, "invalid_hardcoded_load_cell_calibration");
  }
}

void loop() {
  unsigned long nowMs = millis();

  serviceOTA();
  if (isOTAInProgress()) {
    if (!gActuatorsStoppedForOta) {
      stopActuatorsForSafety("ota_update");
      gActuatorsStoppedForOta = true;
    }
    // Keep OTA flash writes isolated from console broadcasts, HTTP/TLS, ADC,
    // and application work. The yield lets both core idle tasks service WDT.
    delay(1);
    return;
  }
  gActuatorsStoppedForOta = false;

  serviceMultiWirelessConsole();
  serviceAsyncHttp();

  // ** CRITICAL PRIORITY: Poll ADC pins first, before any other I/O, to capture fresh data
  // ** with minimal interference from other operations (WiFi, serial, etc).
  pollAdcHighPriority();
  serviceFeederLoadCells();

  handleSerialCommands();

  if (gSerialConsoleExclusive) {
    serviceLoadCellCalibrationSession();
    idleDelayWithFeederLoadCellService(MAIN_LOOP_DELAY_MS);
    return;
  }

  serviceNetworkConnectionTone();
  serviceNetworkToneCue(nowMs);

  if (WiFi.status() != WL_CONNECTED) {
    if (nowMs - state.lastWiFiReconnectAttemptMs >= WIFI_RECONNECT_INTERVAL_MS) {
      state.lastWiFiReconnectAttemptMs = nowMs;
      LOG_WARN("WiFi disconnected; retrying connection");
      connectWiFi();
    }
  }

  serviceTimeSync(nowMs);

  if (SF_ENABLE_MAINS_MONITOR && nowMs - state.lastMainsCheckMs >= MAINS_CHECK_INTERVAL_MS) {
    state.lastMainsCheckMs = nowMs;
    handlePowerFailMonitoring();
  }

  if (isKeypadInputEnabled() && nowMs - state.lastKeypadPollMs >= KEYPAD_POLL_INTERVAL_MS) {
    state.lastKeypadPollMs = nowMs;
    pollKeypad();
  }

  if (nowMs - state.lastSyncMs >= SYNC_INTERVAL_MS) {
    state.lastSyncMs = nowMs;
    if (WiFi.status() == WL_CONNECTED) {
      syncWithServer();
    }
  }

  serviceBufferedOutbox();

  if (nowMs - state.lastConfigRefreshMs >= LOCAL_CONFIG_REFRESH_INTERVAL_MS || cachedCfgStr.length() == 0) {
    state.lastConfigRefreshMs = nowMs;
    cachedCfgStr = loadLocalConfig();
  }

  bool doScheduleCheck = (nowMs - state.lastScheduleCheckMs >= SCHEDULE_CHECK_INTERVAL_MS);
  if (doScheduleCheck) {
    // Ensure device-owned daily total fields stay current by date rollover.
    ensureDailyFeedTotalForToday();
  }

  String cfgStr = cachedCfgStr;
  static DynamicJsonDocument cfgDoc(8192);
  cfgDoc.clear();
  DeserializationError cfgErr = deserializeJson(cfgDoc, cfgStr);
  if (cfgErr) {
    LOG_ERROR("Config JSON parse failed: %s (bytes=%u); using empty defaults",
              cfgErr.c_str(),
              (unsigned int)cfgStr.length());
    cfgDoc.clear();
  }
  JsonVariant cfg = cfgDoc.as<JsonVariant>();
  JsonArray schedules = cfg["schedules"].as<JsonArray>();

  serviceActuators(cfg, millis());

  serviceGrainTypeVerbose(cfg, nowMs);

  updateControlPanel(cfg, schedules);

  processFeedNowCommand(cfg);

  if (doScheduleCheck) {
    state.lastScheduleCheckMs = nowMs;
    checkLowFeedPrediction(schedules, cfg);
    checkSchedulesAndExecute(schedules, cfg);
  }

  if (nowMs - state.lastSensorReportMs >= SENSOR_REPORT_INTERVAL_MS) {
    state.lastSensorReportMs = nowMs;
    reportSensorLevels(cfg);
    if (!isNetworkToneCueActive()) {
      updateLevelErrorBuzzerAlarm(cfg);
    }
    serviceWaterRefillControl(cfg, millis());
  }

  if (SF_SEND_HEARTBEAT_LOGS && nowMs - state.lastHeartbeatLogMs >= HEARTBEAT_LOG_INTERVAL_MS) {
    state.lastHeartbeatLogMs = nowMs;
    StaticJsonDocument<192> hb;
    hb["event"] = "alive";
    hb["uptime_ms"] = nowMs;
    hb["wifi_connected"] = WiFi.status() == WL_CONNECTED;
    sendLog("heartbeat", hb.as<JsonVariant>());
  }

  if (!isNetworkToneCueActive()) {
    serviceLevelErrorBuzzer();
  }

  if (serviceLowBatteryShutdown(cfg, nowMs)) {
    return;
  }

  idleDelayWithFeederLoadCellService(MAIN_LOOP_DELAY_MS);
}
