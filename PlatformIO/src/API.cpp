#include "API.h"

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Update.h>
#include <WiFi.h>

#include "CAN.h"
#include "SavvyCAN.h"
#include "defs.h"
#include "globals.h"
#include "io.h"
#include "power_manager.h"
#include "tasks.h"
#include "wifi_manager.h"
#include "ota_manager.h"

static const char* kKeyAirOut        = "airOut";
static const char* kKeyZeroPreset    = "zeroPreset";
static const char* kKeyAirUpFob       = "airUpFob";
static const char* kKeyAirDownFob     = "airDownFob";
static const char* kKeyAirUpPreset    = "airUpPreset";
static const char* kKeyAirDownPreset  = "airDownPreset";
static const char* kKeyAirUpFront     = "airUpFront";
static const char* kKeyAirUpRear      = "airUpRear";
static const char* kKeyAirDownFront   = "airDownFront";
static const char* kKeyAirDownRear    = "airDownRear";
static const char* kKeyIgnSense      = "ignSense";
static const char* kKeyPresetNames   = "presetNames";
static const char* kKeyPassThrough   = "passthru";
static const char* kKeyIntercept     = "intercept";
static const char* kKeyCanSilenceSec = "canSilSec";
static const char* kKeyCanMinFps     = "canMinFps";
static const char* kKeyBootDelay     = "bootDelay";
static const char* kKeyCanBcEn       = "canBcEn";
static const char* kKeyCanBcId       = "canBcId";
static const char* kKeySavvyWifi      = "savvyWifi";
static const char* kKeySavvySerial    = "savvySerial";
static const char* kKeyPowertrainCan  = "powertrainCan";
static const char* kKeyComfortCan     = "comfortCan";
static const char* kKeyPresetFront   = "pFront";
static const char* kKeyPresetRear    = "pRear";

static String frameHex(const uint8_t* d, uint8_t n) {
  String s; s.reserve(n * 3);
  for (uint8_t i = 0; i < n; i++) {
    if (i) s += ' ';
    if (d[i] < 16) s += '0';
    s += String(d[i], HEX);
  }
  s.toUpperCase();
  return s;
}

void loadPreferences() {
  airOutOnIgnOff = preferences.getBool(kKeyAirOut, false);
  zeroPsiPreset  = preferences.getUChar(kKeyZeroPreset, kZeroPresetManual);
  airUpOnFobDouble = preferences.getBool(kKeyAirUpFob, false);
  airDownOnFobDouble = preferences.getBool(kKeyAirDownFob, false);
  airUpPreset = preferences.getUChar(kKeyAirUpPreset, kZeroPresetManual);
  airDownPreset = preferences.getUChar(kKeyAirDownPreset, zeroPsiPreset);
  airUpFrontPsi = preferences.getUChar(kKeyAirUpFront, 0);
  airUpRearPsi = preferences.getUChar(kKeyAirUpRear, 0);
  airDownFrontPsi = preferences.getUChar(kKeyAirDownFront, 0);
  airDownRearPsi = preferences.getUChar(kKeyAirDownRear, 0);
  ignitionSenseGpio = preferences.getBool(kKeyIgnSense, false);
  passThroughMode = preferences.getBool(kKeyPassThrough, false);
  interceptMode   = !passThroughMode;

  {
    uint16_t s = preferences.getUShort(kKeyCanSilenceSec, kCanSilenceSecDefault);
    if (s > kCanSilenceSecMax) s = kCanSilenceSecMax;
    canSilenceSec = s;
    uint16_t f = preferences.getUShort(kKeyCanMinFps, kCanMinFpsDefault);
    if (f > kCanMinFpsMax) f = kCanMinFpsMax;
    canMinFps = f;
    uint16_t bd = preferences.getUShort(kKeyBootDelay, kControllerBootDelayMsDefault);
    if (bd > kControllerBootDelayMsMax) bd = kControllerBootDelayMsMax;
    controllerBootDelayMs = bd;
  }

  if (preferences.isKey(kKeyPresetFront)) {
    preferences.getBytes(kKeyPresetFront, presetFrontPsi, sizeof(presetFrontPsi));
  }
  if (preferences.isKey(kKeyPresetRear)) {
    preferences.getBytes(kKeyPresetRear, presetRearPsi, sizeof(presetRearPsi));
  }

  if (preferences.isKey(kKeyPresetNames)) {
    preferences.getBytes(kKeyPresetNames, presetNames, sizeof(presetNames));
  }

  canBroadcastEnabled = preferences.getBool(kKeyCanBcEn, false);
  {
    uint32_t id = preferences.getUInt(kKeyCanBcId, kCanBroadcastIdDefault);
    if (id > kCanBroadcastIdMax) id = kCanBroadcastIdDefault;
    canBroadcastId = id;
  }
  savvyCanWifiEnabled = preferences.getBool(kKeySavvyWifi, false);
  savvyCanSerialEnabled = preferences.getBool(kKeySavvySerial, false);
  if (savvyCanSerialEnabled) savvyCanWifiEnabled = false;
  usePowertrainCan = preferences.getBool(kKeyPowertrainCan, true);
  useComfortCan = preferences.getBool(kKeyComfortCan, false);
  if (useComfortCan) usePowertrainCan = false;
}

void setupWiFi() {
  // Common SoftAP + mDNS + LittleFS front-end (reachable at airlift.local).
  wifimgr_config_t wcfg = wifiDefaultConfig();
  wcfg.hostName  = wifiHostName;
  wcfg.mdnsName  = "airlift";   // -> http://airlift.local
  wcfg.fwVersion = FW_VERSION;  // injected into index.html for cache-busting
  wifiManagerInit(&wcfg);
  DEBUG_WIFI("AP up: SSID=%s  IP=%s", wifiHostName, WiFi.softAPIP().toString().c_str());
}

static String takeBody(AsyncWebServerRequest* request) {
  String* p = static_cast<String*>(request->_tempObject);
  String s = p ? *p : String();
  delete p;
  request->_tempObject = nullptr;
  return s;
}

// ESPAsyncWebServer only delivers the body to the matched
// route's own handler; the global onRequestBody() catch-all is NOT called for
// routes registered with server.on().
static void bodyAccum(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                      size_t index, size_t total) {
  if (index == 0) {
    if (request->_tempObject) delete static_cast<String*>(request->_tempObject);
    request->_tempObject = new String();
    if (total > 0) static_cast<String*>(request->_tempObject)->reserve(total);
  }
  if (request->_tempObject)
    static_cast<String*>(request->_tempObject)->concat(reinterpret_cast<const char*>(data), len);
}

void setupApiServer() {
  // The web UI filesystem was mounted (guarded) by wifiManagerInit(); if it is
  // missing or broken, wifiManagerAttachStatic() serves a recovery page at "/".

  // Shared OTA + Home WiFi routes FIRST: ota_manager's first route carries the
  // filter that notes web activity for every request (otaWebClientActive()),
  // and /api/wifi/sta must precede any /api/wifi... route of our own.
  ota_config_t ocfg = otaDefaultConfig();
  ocfg.fwVersion  = FW_VERSION;
  ocfg.product    = "AirLift Controller";
  ocfg.githubRepo = "adamforbes92/AirLift-V2-Controller"; // Releases/ + releases.json for "Check for updates"
  otaManagerInit(&ocfg);
  otaManagerAttach(server);
  wifiManagerAttachSta(server);

  // ----- Status (live pressures + state) -----
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    doc["fwVersion"] = FW_VERSION;
    doc["pFL"]      = pressureFL;
    doc["pFR"]      = pressureFR;
    doc["pRL"]      = pressureRL;
    doc["pRR"]      = pressureRR;
    doc["pTank"]    = pressureTank;
    doc["psiFL"]    = psiFromRaw(pressureFL);
    doc["psiFR"]    = psiFromRaw(pressureFR);
    doc["psiRL"]    = psiFromRaw(pressureRL);
    doc["psiRR"]    = psiFromRaw(pressureRR);
    doc["psiTank"]  = (uint16_t)pressureTank; // already in PSI
    doc["compressor"]     = (bool)compressorOn;
    doc["mode"]     = (uint8_t)currentMode;
    doc["modeStr"]  = (currentMode == MODE_MANUAL) ? "manual"
                       : (currentMode == MODE_PRESET) ? "preset" : "unknown";
    {
      const uint32_t nowMs = millis();
      doc["linActive"]       = (uint32_t)(nowMs - lastLinRxMs)         < 5000;
      doc["handheldHealthy"] = (uint32_t)(nowMs - lastHandheldFrameMs) < 3000;
      doc["manifoldHealthy"] = (uint32_t)(nowMs - lastManifoldFrameMs) < 3000;
      doc["busReversed"]     = (bool)busReversed;
    }
    doc["interceptMode"]     = (bool)interceptMode;
    doc["passThroughMode"]   = (bool)passThroughMode;
    doc["ignition"]          = (bool)ignitionOn;
    doc["controllerPowered"] = (bool)controllerPowered;
    doc["highSideForced"]    = (bool)highSideOverrideActive;
    doc["highSideForcedOn"]  = (bool)highSideOverrideOn;
    doc["canDriverRunning"]  = canDriverRunning();
    doc["canActive"]         = canActive();
    doc["canFramesSeen"]     = canFramesSeen;
    doc["canFps"]            = canCurrentFps;
    doc["canSilenceSec"]     = canSilenceSec;
    doc["canMinFps"]         = canMinFps;
    doc["canBroadcastEnabled"] = (bool)canBroadcastEnabled;
    doc["canBroadcastId"]    = canBroadcastId;
    doc["canBroadcastSent"]  = canBroadcastSent;
    doc["canBroadcastErrors"] = canBroadcastErrors;
    doc["savvyCanWifiEnabled"] = (bool)savvyCanWifiEnabled;
    doc["savvyCanSerialEnabled"] = (bool)savvyCanSerialEnabled;
    doc["savvyCanFramesDropped"] = savvyCanFramesDropped;
    doc["usePowertrainCan"] = (bool)usePowertrainCan;
    doc["useComfortCan"] = (bool)useComfortCan;
    doc["comfortLockState"] = comfortLockState == LOCK_STATE_LOCKED ? "Locked"
                 : comfortLockState == LOCK_STATE_UNLOCKED ? "Unlocked" : "Unknown";
    doc["rxBytesC"]          = rxBytesController;
    doc["rxBytesM"]          = rxBytesManifold;
    doc["framesC2M"]         = framesFromController;
    doc["framesM2C"]         = framesFromManifold;
    doc["lastBcastMs"]       = lastPressureBroadcastMs;
    doc["lastReplyMs"]       = lastManifoldReplyMs;
    doc["lastTargetFront"]   = lastPresetTargetFrontPsi;
    doc["lastTargetRear"]    = lastPresetTargetRearPsi;
    doc["lastBtn"]           = frameHex(lastFromControllerFrame, lastFromControllerLen);
    doc["lastMan"]           = frameHex(lastFromManifoldFrame, lastFromManifoldLen);
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  // ----- Diagnostics: directly force the controller high-side GPIO -----
  // body: {"on":true|false}; runtime-only, reset on reboot.
  server.on("/api/diagnostics/high-side", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      String body = takeBody(req);
      JsonDocument doc;
      if (deserializeJson(doc, body) || doc["on"].isNull()) {
        req->send(400, "application/json", "{\"ok\":false}");
        return;
      }
      const bool on = doc["on"].as<bool>();
      // Checked = force the high-side ON; unchecked = RELEASE the override and
      // return to auto ignition/CAN control (not a force-OFF latch).
      if (on) setHighSideOverride(true);
      else    clearHighSideOverride();
      logLine("diagnostics: high-side %s", on ? "forced ON" : "released (auto)");
      req->send(200, "application/json", "{\"ok\":true}");
    }, nullptr, bodyAccum);

  // ----- Setup (settings) -----
  server.on("/api/setup", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    doc["airOutOnIgnOff"]  = (bool)airOutOnIgnOff;
    doc["ignitionSenseGpio"] = (bool)ignitionSenseGpio;
    doc["zeroPsiPreset"]   = zeroPsiPreset;
    doc["airUpOnFobDouble"] = (bool)airUpOnFobDouble;
    doc["airDownOnFobDouble"] = (bool)airDownOnFobDouble;
    doc["airUpPreset"] = airUpPreset;
    doc["airDownPreset"] = airDownPreset;
    doc["airUpFrontPsi"] = airUpFrontPsi;
    doc["airUpRearPsi"] = airUpRearPsi;
    doc["airDownFrontPsi"] = airDownFrontPsi;
    doc["airDownRearPsi"] = airDownRearPsi;
    doc["passThroughMode"] = (bool)passThroughMode;
    doc["interceptMode"]   = (bool)interceptMode;
    doc["canSilenceSec"]   = canSilenceSec;
    doc["canMinFps"]       = canMinFps;
    doc["controllerBootDelayMs"] = controllerBootDelayMs;
    doc["canBroadcastEnabled"] = (bool)canBroadcastEnabled;
    doc["canBroadcastId"]      = canBroadcastId;
    doc["savvyCanWifiEnabled"] = (bool)savvyCanWifiEnabled;
    doc["savvyCanSerialEnabled"] = (bool)savvyCanSerialEnabled;
    doc["usePowertrainCan"] = (bool)usePowertrainCan;
    doc["useComfortCan"] = (bool)useComfortCan;

    JsonArray presets = doc["presets"].to<JsonArray>();
    for (uint8_t i = 0; i < kPresetCount; i++) {
      JsonObject row = presets.add<JsonObject>();
      row["index"] = i;
      row["name"]  = presetNames[i];
      row["frontPsi"] = presetFrontPsi[i];
      row["rearPsi"]  = presetRearPsi[i];
    }
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  // ----- Save settings -----
  server.on("/api/settings", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      String body = takeBody(req);
      DEBUG_API("settings handler: bodyLen=%u body=%s", (unsigned)body.length(), body.c_str());
      JsonDocument doc;
      if (deserializeJson(doc, body)) {
        DEBUG_API("settings: JSON parse failed");
        req->send(400, "application/json", "{\"ok\":false}");
        return;
      }
      if (!doc["airOutOnIgnOff"].isNull()) {
        airOutOnIgnOff = doc["airOutOnIgnOff"].as<bool>();
      }
      if (!doc["ignitionSenseGpio"].isNull()) {
        ignitionSenseGpio = doc["ignitionSenseGpio"].as<bool>();
      }
      int z = doc["zeroPsiPreset"] | (int)zeroPsiPreset;
      if (z < 0 || (z >= kPresetCount && z != kZeroPresetManual)) z = kZeroPresetManual;
      zeroPsiPreset = static_cast<uint8_t>(z);
      if (!doc["airUpOnFobDouble"].isNull()) {
        airUpOnFobDouble = doc["airUpOnFobDouble"].as<bool>();
      }
      if (!doc["airDownOnFobDouble"].isNull()) {
        airDownOnFobDouble = doc["airDownOnFobDouble"].as<bool>();
      }
      int airUpSlot = doc["airUpPreset"] | (int)airUpPreset;
      int airDownSlot = doc["airDownPreset"] | (int)airDownPreset;
      if (airUpSlot < 0 || (airUpSlot >= kPresetCount && airUpSlot != kZeroPresetManual)) airUpSlot = kZeroPresetManual;
      if (airDownSlot < 0 || (airDownSlot >= kPresetCount && airDownSlot != kZeroPresetManual)) airDownSlot = kZeroPresetManual;
      airUpPreset = static_cast<uint8_t>(airUpSlot);
      airDownPreset = static_cast<uint8_t>(airDownSlot);
      airUpFrontPsi = min<uint8_t>(doc["airUpFrontPsi"] | airUpFrontPsi, 200);
      airUpRearPsi = min<uint8_t>(doc["airUpRearPsi"] | airUpRearPsi, 200);
      airDownFrontPsi = min<uint8_t>(doc["airDownFrontPsi"] | airDownFrontPsi, 200);
      airDownRearPsi = min<uint8_t>(doc["airDownRearPsi"] | airDownRearPsi, 200);
      if (!doc["passThroughMode"].isNull()) {
        passThroughMode = doc["passThroughMode"].as<bool>();
        interceptMode   = !passThroughMode;
      }
      DEBUG_API("settings POST: passthru=%d inter=%d body=%s", (int)(bool)passThroughMode, (int)(bool)interceptMode, body.c_str());

      int silSec = doc["canSilenceSec"] | (int)canSilenceSec;
      if (silSec < kCanSilenceSecMin) silSec = kCanSilenceSecMin;
      if (silSec > kCanSilenceSecMax) silSec = kCanSilenceSecMax;
      canSilenceSec = static_cast<uint16_t>(silSec);

      int minFps = doc["canMinFps"] | (int)canMinFps;
      if (minFps < kCanMinFpsMin) minFps = kCanMinFpsMin;
      if (minFps > kCanMinFpsMax) minFps = kCanMinFpsMax;
      canMinFps = static_cast<uint16_t>(minFps);

      int bootDelay = doc["controllerBootDelayMs"] | (int)controllerBootDelayMs;
      if (bootDelay < kControllerBootDelayMsMin) bootDelay = kControllerBootDelayMsMin;
      if (bootDelay > kControllerBootDelayMsMax) bootDelay = kControllerBootDelayMsMax;
      controllerBootDelayMs = static_cast<uint16_t>(bootDelay);

      const bool oldCanTx = canBroadcastEnabled || savvyCanRequiresCanTransmit();
      const uint32_t oldBcId = canBroadcastId;
      const bool oldUseComfortCan = useComfortCan;
      if (!doc["canBroadcastEnabled"].isNull()) {
        canBroadcastEnabled = doc["canBroadcastEnabled"].as<bool>();
      }
      uint32_t bcId = doc["canBroadcastId"] | canBroadcastId;
      if (bcId > kCanBroadcastIdMax) bcId = kCanBroadcastIdMax;
      canBroadcastId = bcId;

      if (!doc["savvyCanWifiEnabled"].isNull()) {
        savvyCanSetWifiEnabled(doc["savvyCanWifiEnabled"].as<bool>());
      }
      if (!doc["savvyCanSerialEnabled"].isNull()) {
        savvyCanSetSerialEnabled(doc["savvyCanSerialEnabled"].as<bool>());
      }
      if (!doc["usePowertrainCan"].isNull()) {
        usePowertrainCan = doc["usePowertrainCan"].as<bool>();
        useComfortCan = !usePowertrainCan;
      }
      if (!doc["useComfortCan"].isNull()) {
        useComfortCan = doc["useComfortCan"].as<bool>();
        usePowertrainCan = !useComfortCan;
      }
      preferences.putBool(kKeyAirOut, (bool)airOutOnIgnOff);
      preferences.putBool(kKeyIgnSense, (bool)ignitionSenseGpio);
      preferences.putUChar(kKeyZeroPreset, zeroPsiPreset);
      preferences.putBool(kKeyAirUpFob, (bool)airUpOnFobDouble);
      preferences.putBool(kKeyAirDownFob, (bool)airDownOnFobDouble);
      preferences.putUChar(kKeyAirUpPreset, airUpPreset);
      preferences.putUChar(kKeyAirDownPreset, airDownPreset);
      preferences.putUChar(kKeyAirUpFront, airUpFrontPsi);
      preferences.putUChar(kKeyAirUpRear, airUpRearPsi);
      preferences.putUChar(kKeyAirDownFront, airDownFrontPsi);
      preferences.putUChar(kKeyAirDownRear, airDownRearPsi);
      preferences.putBool(kKeyPassThrough, (bool)passThroughMode);
      preferences.putBool(kKeyIntercept,   (bool)interceptMode);
      preferences.putUShort(kKeyCanSilenceSec, canSilenceSec);
      preferences.putUShort(kKeyCanMinFps,     canMinFps);
      preferences.putUShort(kKeyBootDelay,     controllerBootDelayMs);
      preferences.putBool(kKeyCanBcEn, (bool)canBroadcastEnabled);
      preferences.putUInt(kKeyCanBcId, canBroadcastId);
      preferences.putBool(kKeySavvyWifi, (bool)savvyCanWifiEnabled);
      preferences.putBool(kKeySavvySerial, (bool)savvyCanSerialEnabled);
      preferences.putBool(kKeyPowertrainCan, (bool)usePowertrainCan);
      preferences.putBool(kKeyComfortCan, (bool)useComfortCan);

      // Driver configuration changes require a full reinitialization:
      // LISTEN_ONLY <-> NORMAL, broadcast ID, or 500 <-> 125 kbit/s source.
      const bool newCanTx = canBroadcastEnabled || savvyCanRequiresCanTransmit();
      if (oldCanTx != newCanTx || oldUseComfortCan != useComfortCan ||
          (canBroadcastEnabled && oldBcId != canBroadcastId)) {
        canReinit();
      }
      req->send(200, "application/json", "{\"ok\":true}");
    }, nullptr, bodyAccum);

  // ----- Intercept mode: configure per-preset target pressures (PSI) -----
  // body: {"index":N, "frontPsi":F, "rearPsi":R}
  server.on("/api/preset/target", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      String body = takeBody(req);
      JsonDocument doc;
      if (deserializeJson(doc, body)) { req->send(400, "application/json", "{\"ok\":false}"); return; }
      int idx = doc["index"] | -1;
      int front = doc["frontPsi"] | -1;
      int rear  = doc["rearPsi"]  | -1;
      if (idx < 0 || idx >= kPresetCount || front < 0 || front > 127 ||
          rear < 0 || rear > 127) {
        req->send(400, "application/json", "{\"ok\":false}");
        return;
      }
      presetFrontPsi[idx] = (uint8_t)front;
      presetRearPsi[idx]  = (uint8_t)rear;
      preferences.putBytes(kKeyPresetFront, presetFrontPsi, sizeof(presetFrontPsi));
      preferences.putBytes(kKeyPresetRear,  presetRearPsi,  sizeof(presetRearPsi));
      req->send(200, "application/json", "{\"ok\":true}");
    }, nullptr, bodyAccum);

  // ----- Preset learn: start a capture session -----
  server.on("/api/learn/start", HTTP_POST, [](AsyncWebServerRequest* req) {
    portENTER_CRITICAL(&airliftMux);
    learnMask    = 0;
    learnSlot    = -1;
    for (uint8_t i = 0; i < kPresetCount; i++) { learnFrontPsi[i] = 0; learnRearPsi[i] = 0; }
    learnActive  = true;
    learnUntilMs = millis() + 90000;   // 90 s window
    portEXIT_CRITICAL(&airliftMux);
    logLine("preset learn: started");
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // ----- Preset learn: arm a specific slot; the next physical preset press is
  //       stored into it. body: {"slot":N}  (0-based) -----
  server.on("/api/learn/arm", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      String body = takeBody(req);
      JsonDocument doc;
      if (deserializeJson(doc, body)) { req->send(400, "application/json", "{\"ok\":false}"); return; }
      const int slot = doc["slot"] | -1;
      if (slot < 0 || slot >= kPresetCount) { req->send(400, "application/json", "{\"ok\":false}"); return; }
      portENTER_CRITICAL(&airliftMux);
      if (!learnActive || (int32_t)(millis() - learnUntilMs) >= 0) {
        learnActive  = true;                 // (re)open the window if it lapsed
        learnUntilMs = millis() + 90000;
      }
      learnSlot = (int8_t)slot;
      portEXIT_CRITICAL(&airliftMux);
      logLine("preset learn: armed slot %d", slot + 1);
      req->send(200, "application/json", "{\"ok\":true}");
    }, nullptr, bodyAccum);

  // ----- Preset learn: stop early -----
  server.on("/api/learn/stop", HTTP_POST, [](AsyncWebServerRequest* req) {
    portENTER_CRITICAL(&airliftMux);
    learnActive = false;
    learnSlot   = -1;
    portEXIT_CRITICAL(&airliftMux);
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // ----- Preset learn: poll capture state -----
  server.on("/api/learn", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    const uint32_t nowMs = millis();
    const bool active = learnActive && (int32_t)(nowMs - learnUntilMs) < 0;
    doc["active"]    = active;
    doc["remainMs"]  = active ? (uint32_t)(learnUntilMs - nowMs) : 0;
    doc["armedSlot"] = (int)learnSlot;
    JsonArray arr = doc["slots"].to<JsonArray>();
    portENTER_CRITICAL(&airliftMux);
    const uint8_t mask = learnMask;
    for (uint8_t i = 0; i < kPresetCount; i++) {
      JsonObject o = arr.add<JsonObject>();
      o["slot"]     = i;
      o["front"]    = learnFrontPsi[i];
      o["rear"]     = learnRearPsi[i];
      o["captured"] = (bool)(mask & (1u << i));
    }
    portEXIT_CRITICAL(&airliftMux);
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  // ----- Preset learn: write captured slots into the preset slots + NVS -----
  server.on("/api/learn/save", HTTP_POST, [](AsyncWebServerRequest* req) {
    uint8_t n = 0;
    portENTER_CRITICAL(&airliftMux);
    learnActive = false;
    learnSlot   = -1;
    const uint8_t mask = learnMask;
    for (uint8_t i = 0; i < kPresetCount; i++) {
      if (mask & (1u << i)) {                // only overwrite slots that were learned
        presetFrontPsi[i] = learnFrontPsi[i];
        presetRearPsi[i]  = learnRearPsi[i];
        n++;
      }
    }
    portEXIT_CRITICAL(&airliftMux);
    preferences.putBytes(kKeyPresetFront, presetFrontPsi, sizeof(presetFrontPsi));
    preferences.putBytes(kKeyPresetRear,  presetRearPsi,  sizeof(presetRearPsi));
    logLine("preset learn: saved %u presets", (unsigned)n);
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // ----- Intercept: send a preset target NOW -----
  // body: {"index":N} (uses stored target) OR {"frontPsi":F, "rearPsi":R}
  // optional: {"holdMs":N}  default 3000 ms
  server.on("/api/intercept/preset", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      String body = takeBody(req);
      JsonDocument doc;
      if (deserializeJson(doc, body)) { req->send(400, "application/json", "{\"ok\":false}"); return; }
      uint8_t f = 0, r = 0;
      if (doc["index"].is<int>()) {
        int idx = doc["index"];
        if (idx < 0 || idx >= kPresetCount) { req->send(400, "application/json", "{\"ok\":false}"); return; }
        f = presetFrontPsi[idx];
        r = presetRearPsi[idx];
        // NB: f==0 && r==0 IS a legitimate preset (full air-out).
        // The real handheld emits `01 16 47 00 00 0F 00 00 99` for it
        // (verified in 'manual,up,down,preset,all 8' capture).
      } else {
        f = (uint8_t)(int)(doc["frontPsi"] | 0);
        r = (uint8_t)(int)(doc["rearPsi"]  | 0);
      }
      uint32_t holdMs = doc["holdMs"] | kInterceptPresetHoldMsDefault;
      portENTER_CRITICAL(&airliftMux);
      // Hold PRESET: every handheld poll is now rewritten into preset traffic,
      // pushing this target for `holdMs` then settling to the heartbeat.
      desiredMode                = MODE_PRESET;
      if (currentMode != MODE_PRESET) presetEnterUntilMs = millis() + 400;
      pendingPresetFrontPsi      = f;
      pendingPresetRearPsi       = r;
      pendingPresetRepeatUntilMs = millis() + holdMs;
      portEXIT_CRITICAL(&airliftMux);
      logLine("intercept preset %u/%u psi hold=%lums", f, r, (unsigned long)holdMs);
      req->send(200, "application/json", "{\"ok\":true}");
    }, nullptr, bodyAccum);

  // ----- Intercept: hold a manual button -----
  // body: {"corner":"FL"|"FR"|"RL"|"RR", "dir":"up"|"down",
  //        "action":"press"|"hold"|"release"|"tap"   (default "tap"),
  //        "holdMs":N   (used by "tap"/"hold"; default 250) }
  //
  // Press/hold/release semantics let the UI mirror the real handheld:
  //   * "press"   start emitting the manual-button poll; auto-release if
  //               the UI doesn't ping again within ~1500 ms.
  //   * "hold"    same as "press" but caller-specified safety window.
  //   * "release" stop emitting and queue one explicit 01 41 00 button-up.
  //   * "tap"     hold for holdMs (default 250) then auto-release.
  server.on("/api/intercept/manual", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      String body = takeBody(req);
      JsonDocument doc;
      if (deserializeJson(doc, body)) { req->send(400, "application/json", "{\"ok\":false}"); return; }
      const char* c      = doc["corner"] | "";
      const char* d      = doc["dir"]    | "";
      const char* action = doc["action"] | "tap";
      uint32_t    holdMs = doc["holdMs"] | kInterceptManualTapMsDefault;

      uint8_t code = 0;
      if (!strcmp(c, "FL") && !strcmp(d, "up"))        code = BTN_MANUAL_FL_UP;
      else if (!strcmp(c, "FL") && !strcmp(d, "down")) code = BTN_MANUAL_FL_DOWN;
      else if (!strcmp(c, "FR") && !strcmp(d, "up"))   code = BTN_MANUAL_FR_UP;
      else if (!strcmp(c, "FR") && !strcmp(d, "down")) code = BTN_MANUAL_FR_DOWN;
      else if (!strcmp(c, "RL") && !strcmp(d, "up"))   code = BTN_MANUAL_RL_UP;
      else if (!strcmp(c, "RL") && !strcmp(d, "down")) code = BTN_MANUAL_RL_DOWN;
      else if (!strcmp(c, "RR") && !strcmp(d, "up"))   code = BTN_MANUAL_RR_UP;
      else if (!strcmp(c, "RR") && !strcmp(d, "down")) code = BTN_MANUAL_RR_DOWN;
      else if (!strcmp(c, "ALL") && !strcmp(d, "up"))   code = BTN_MANUAL_ALL_UP;
      else if (!strcmp(c, "ALL") && !strcmp(d, "down")) code = BTN_MANUAL_ALL_DOWN;

      if (!strcmp(action, "release")) {
        portENTER_CRITICAL(&airliftMux);
        pendingButtonCode    = 0;
        pendingButtonRelease = true;
        portEXIT_CRITICAL(&airliftMux);
        logLine("intercept manual release");
        req->send(200, "application/json", "{\"ok\":true}");
        return;
      }

      if (!code) { req->send(400, "application/json", "{\"ok\":false}"); return; }

      // press / hold / tap all share the same machinery; the difference is
      // the safety window: press defaults to 1500 ms (UI heartbeats extend
      // it), tap uses the caller's holdMs (default 250 ms).
      uint32_t window;
      if (!strcmp(action, "press"))      window = kInterceptManualPressWindowMs;
      else if (!strcmp(action, "hold"))  window = holdMs ? holdMs : kInterceptManualPressWindowMs;
      else                                window = holdMs ? holdMs : kInterceptManualTapMsDefault;  // "tap"

      portENTER_CRITICAL(&airliftMux);
      // A manual button implies MANUAL mode; transformHandheldFrame rewrites the
      // handheld's polls into this button press for the hold window.
      desiredMode                = MODE_MANUAL;
      presetEnterUntilMs         = 0;
      pendingButtonCode          = code;
      pendingButtonRepeatUntilMs = millis() + window;
      pendingButtonRelease       = false;
      portEXIT_CRITICAL(&airliftMux);
      logLine("intercept manual %s %s 0x%02X %s win=%lums",
              c, d, code, action, (unsigned long)window);
      req->send(200, "application/json", "{\"ok\":true}");
    }, nullptr, bodyAccum);

  // ----- Intercept: queue a mode switch -----
  server.on("/api/intercept/modeswitch", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      String body = takeBody(req);
      JsonDocument doc;
      deserializeJson(doc, body);
      const char* modeStr = doc["mode"] | "";
      DEBUG_API("modeswitch: requested mode='%s' interceptMode=%d passThroughMode=%d",
            modeStr, (int)(bool)interceptMode, (int)(bool)passThroughMode);
      portENTER_CRITICAL(&airliftMux);
      if (!strcmp(modeStr, "preset")) {
        // Hold PRESET by rewriting every handheld poll into preset heartbeats.
        desiredMode = MODE_PRESET;
        if (currentMode != MODE_PRESET) presetEnterUntilMs = millis() + 400;
      } else if (!strcmp(modeStr, "manual")) {
        // Leave preset. This manifold latches preset and only releases on the
        // 01 14 41 frame, so inject that + a manual poll (see transformHandheldFrame).
        desiredMode           = MODE_MANUAL;
        presetEnterUntilMs    = 0;
        pendingButtonCode     = 0;
        pendingButtonRelease  = false;
        pendingPresetFrontPsi = 0xFF;
        pendingPresetRearPsi  = 0xFF;
        manualEnterUntilMs    = (currentMode == MODE_PRESET) ? (millis() + 2000) : 0;
      }
      portEXIT_CRITICAL(&airliftMux);
      req->send(200, "application/json", "{\"ok\":true}");
    }, nullptr, bodyAccum);

  // ----- Bulk save preset PSI targets + names -----
  // body: {"presets":[{"name":"...", "frontPsi":N, "rearPsi":N}, ... 8 entries]}
  server.on("/api/presets", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      String body = takeBody(req);
      JsonDocument doc;
      if (deserializeJson(doc, body)) { req->send(400, "application/json", "{\"ok\":false}"); return; }
      JsonArray arr = doc["presets"];
      if (arr.isNull()) { req->send(400, "application/json", "{\"ok\":false}"); return; }
      uint8_t i = 0;
      for (JsonObject row : arr) {
        if (i >= kPresetCount) break;
        const char* name = row["name"] | presetNames[i];
        int front = row["frontPsi"] | 0;
        int rear  = row["rearPsi"]  | 0;
        if (front < 0)   front = 0;   if (front > 255) front = 255;
        if (rear  < 0)   rear  = 0;   if (rear  > 255) rear  = 255;
        strlcpy(presetNames[i], name, sizeof(presetNames[i]));
        presetFrontPsi[i] = (uint8_t)front;
        presetRearPsi[i]  = (uint8_t)rear;
        i++;
      }
      preferences.putBytes(kKeyPresetNames, presetNames,    sizeof(presetNames));
      preferences.putBytes(kKeyPresetFront, presetFrontPsi, sizeof(presetFrontPsi));
      preferences.putBytes(kKeyPresetRear,  presetRearPsi,  sizeof(presetRearPsi));
      req->send(200, "application/json", "{\"ok\":true}");
    }, nullptr, bodyAccum);

  // ----- Diagnostic log -----
  // Query string `since` = last seen writeIndex; only newer entries are returned.
  server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* req) {
    uint32_t since = 0;
    if (req->hasParam("since")) {
      since = strtoul(req->getParam("since")->value().c_str(), nullptr, 10);
    }
    const uint32_t writeIdx = logWriteIndex;
    uint32_t start = since;
    // Cap how far back we can rewind to the ring window.
    if (writeIdx > kLogLineCount && start < writeIdx - kLogLineCount) {
      start = writeIdx - kLogLineCount;
    }
    if (start > writeIdx) start = writeIdx;

    JsonDocument doc;
    doc["writeIndex"] = writeIdx;
    JsonArray arr = doc["entries"].to<JsonArray>();
    for (uint32_t i = start; i < writeIdx; i++) {
      const LogEntry& e = logBuffer[i % kLogLineCount];
      JsonObject row = arr.add<JsonObject>();
      row["i"] = i;
      row["ms"] = e.ms;
      row["t"] = e.text;
    }
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server.on("/api/log/clear", HTTP_POST, [](AsyncWebServerRequest* req) {
    portENTER_CRITICAL(&airliftMux);
    logWriteIndex = 0;
    memset(logBuffer, 0, sizeof(logBuffer));
    portEXIT_CRITICAL(&airliftMux);
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // ----- "/" (the UI, or the recovery page when the filesystem holds no
  // usable UI) + static files with no-cache revalidation -----
  wifiManagerAttachStatic(server);

  server.begin();
  DEBUG_API("HTTP server up");
}

// ----------------------------------------------------------------------------
// power_manager integration (universal reduced-power module)
// ----------------------------------------------------------------------------
// These override the weak hooks in power_manager.cpp. The device stays fully
// awake while ANY client is associated to the AP. Once the last client leaves,
// the manager's idle timer runs, then turns the radio off and drops the CPU
// clock. A fresh burst of CAN traffic (see pollCanRx wake-on-burst) or an
// ignition power-cycle brings WiFi back.

// Spec: reduced power is entered 1 minute after the last WiFi CLIENT
// disconnects — independent of CAN activity. CAN traffic must NOT hold WiFi on
// (that would keep the radio up, and the regulator hot, the whole time the car
// is running). CAN only WAKES us from reduced power via pollCanRx(); it does
// not keep us awake here.
bool powerIsBusy() {
  // ... or a browser has hit us in the last 30 s (a phone on the home router
  // in bridge mode is not an AP station).
  return WiFi.softAPgetStationNum() > 0 || otaInProgress() || otaWebClientActive();
}

// ACTIVE -> REDUCED: before the radio drops, close the web server AND drop any
// active web override so the MITM can't be left holding a mode the user can no
// longer change from the (now offline) UI. Returning to "match the controller"
// means pure pass-through — no lingering comms issues while WiFi is off.
void powerOnEnterReduced() {
  server.end();
  portENTER_CRITICAL(&airliftMux);
  desiredMode           = handheldMode;   // stop overriding — follow the controller
  presetEnterUntilMs    = 0;
  manualEnterUntilMs    = 0;
  pendingButtonCode     = 0;
  pendingButtonRelease  = false;
  pendingPresetFrontPsi = 0xFF;
  pendingPresetRearPsi  = 0xFF;
  portEXIT_CRITICAL(&airliftMux);
  DEBUG_PWR("reduced — web override cleared, pass-through");
}

// REDUCED -> ACTIVE: bring the AP and web server back. Routes are already
// registered (no need to re-run setupApiServer()), so we only restart the
// radio and the listener.
//
// The bringup is DEFERRED: waking also re-powers the handheld from cold, which
// then re-handshakes with the manifold over LIN. Bringing the WiFi radio up in
// that exact window (RF calibration briefly steals interrupt time) corrupts the
// fragile handshake and the handheld shows comms errors — even though we change
// no frames. So powerOnExitReduced() only ARMS the request; serviceDeferredWifi()
// completes it once the LIN relay is healthy again (or a hard cap elapses).
static volatile bool     s_wifiWakePending = false;
static volatile uint32_t s_wifiWakeAtMs    = 0;

void powerOnExitReduced() {
  s_wifiWakeAtMs    = millis();
  s_wifiWakePending = true;
  DEBUG_PWR("active — WiFi bringup deferred until LIN settles");
}

void serviceDeferredWifi() {
  if (!s_wifiWakePending) return;
  const uint32_t now       = millis();
  const uint32_t sinceWake = now - s_wifiWakeAtMs;
  const bool linHealthy = (uint32_t)(now - lastManifoldFrameMs) < kWifiWakeLinFreshMs;
  const bool settled    = sinceWake >= kWifiWakeSettleMs && linHealthy;
  const bool capped     = sinceWake >= kWifiWakeMaxDeferMs;
  if (!settled && !capped) return;
  s_wifiWakePending = false;
  wifiManagerStartAP();
  server.begin();
  DEBUG_PWR("WiFi restored %lums after wake (%s)",
        (unsigned long)sinceWake, capped ? "cap" : "lin-ok");
}

