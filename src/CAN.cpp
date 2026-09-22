#include "CAN.h"

#include <driver/twai.h>
#include <Arduino.h>

#include "defs.h"
#include "globals.h"
#include "power_manager.h"
#include "SavvyCAN.h"

static bool s_driverInstalled = false;

void canInit() {
  // NORMAL mode is required to transmit. When we're purely sniffing we stay in
  // LISTEN_ONLY so we never emit ACK bits or error frames onto a foreign bus.
  const twai_mode_t mode = (canBroadcastEnabled || savvyCanRequiresCanTransmit())
                               ? TWAI_MODE_NORMAL
                               : TWAI_MODE_LISTEN_ONLY;
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(pinCAN_TX),
      static_cast<gpio_num_t>(pinCAN_RX),
      mode);
  // VW MQB powertrain CAN is 500 kbit/s; Comfort CAN is 100 kbit/s.
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  if (useComfortCan) t = TWAI_TIMING_CONFIG_100KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    DEBUG_CAN("TWAI install failed");
    return;
  }
  if (twai_start() != ESP_OK) {
    DEBUG_CAN("TWAI start failed");
    return;
  }
  s_driverInstalled = true;
  DEBUG_CAN("TWAI started: %s, %u kbit/s",
      mode == TWAI_MODE_NORMAL ? "NORMAL" : "LISTEN_ONLY",
      useComfortCan ? 100u : 500u);
}

void canReinit() {
  if (s_driverInstalled) {
    twai_stop();
    twai_driver_uninstall();
    s_driverInstalled = false;
  }
  canInit();
}

void pollCanRx() {
  twai_message_t frame;

  // Wake-on-burst state. We only wake WiFi out of reduced power when the bus
  // transitions from quiet -> active (a "fresh run") AND enough frames arrive
  // to prove it's real traffic rather than a stray frame. A run that is already
  // in progress while we are ACTIVE is marked consumed, so that when the WiFi
  // idle timer later drops us to REDUCED we do NOT immediately re-wake on the
  // same continuous stream (e.g. while driving with no phone connected).
  static uint32_t s_wakeRunMs       = 0;
  static uint32_t s_wakeRunStartCnt = 0;
  static bool     s_wakeConsumedRun = true;

  while (twai_receive(&frame, 0) == ESP_OK) {
    canProcessFrame(frame);
    savvyCanQueueFrame(frame);
    const uint32_t now = millis();
    uint32_t trafficSince, framesNow;
    portENTER_CRITICAL(&airliftMux);
    if (lastCanFrameMs == 0 || now - lastCanFrameMs > kCanTrafficGapMs) {
      canTrafficSinceMs = now;
    }
    canFramesSeen++;
    lastCanFrameMs = now;
    trafficSince = canTrafficSinceMs;
    framesNow    = canFramesSeen;
    portEXIT_CRITICAL(&airliftMux);

    // A new traffic run begins whenever canTrafficSinceMs advances.
    if (trafficSince != s_wakeRunMs) {
      s_wakeRunMs       = trafficSince;
      s_wakeRunStartCnt = framesNow;
      s_wakeConsumedRun = false;
    }
    if (powerGetState() != POWER_STATE_REDUCED) {
      // Already awake — this run needs no wake-up (prevents re-waking on the
      // same continuous stream once the idle timer sleeps us again).
      s_wakeConsumedRun = true;
    } else if (!s_wakeConsumedRun &&
               framesNow - s_wakeRunStartCnt >= kCanWakeFrameMin) {
      // Genuine burst while asleep: restore WiFi for the idle window. Lock/
      // unlock and air-up/down parsing already ran above and is independent of
      // WiFi, so this only brings the web UI back.
      powerForceActive();
      s_wakeConsumedRun = true;
      DEBUG_CAN("wake burst (%lu frames) -> restoring WiFi",
                (unsigned long)(framesNow - s_wakeRunStartCnt));
    }
  }

  // Recompute live frame rate once per second (delta of canFramesSeen).
  static uint32_t fpsWindowStartMs = 0;
  static uint32_t fpsWindowStartCount = 0;
  const uint32_t now = millis();
  if (fpsWindowStartMs == 0) {
    fpsWindowStartMs = now;
    fpsWindowStartCount = canFramesSeen;
  } else if (now - fpsWindowStartMs >= 1000) {
    const uint32_t delta = canFramesSeen - fpsWindowStartCount;
    canCurrentFps = (delta > 0xFFFFu) ? 0xFFFFu : (uint16_t)delta;
    fpsWindowStartMs = now;
    fpsWindowStartCount = canFramesSeen;
  }
}

void canProcessFrame(const twai_message_t& frame) {
  if (!useComfortCan || frame.extd) return;

  switch (frame.identifier) {
    case kComfortFobCanId: {
      if (frame.data_length_code < 1) return;
      const uint8_t code = frame.data[0];
      if (code != kComfortFobUnlockCode && code != kComfortFobLockCode) return;

      static uint8_t lastCode = 0;
      static uint32_t lastCodeMs = 0;
      const uint32_t now = millis();
      if (code == lastCode && now - lastCodeMs < kComfortFobDebounceMs) return;
      lastCode = code;
      lastCodeMs = now;

      portENTER_CRITICAL(&airliftMux);
      if (code == kComfortFobUnlockCode) {
        if (lastComfortFobUnlockMs != 0 && now - lastComfortFobUnlockMs <= kComfortFobDoublePressMs) {
          comfortFobDoubleUnlock = true;
          lastComfortFobUnlockMs = 0;
        } else {
          lastComfortFobUnlockMs = now;
        }
        comfortLockState = LOCK_STATE_UNLOCKED;
      } else {
        if (lastComfortFobLockMs != 0 && now - lastComfortFobLockMs <= kComfortFobDoublePressMs) {
          comfortFobDoubleLock = true;
          lastComfortFobLockMs = 0;
        } else {
          lastComfortFobLockMs = now;
        }
        comfortLockState = LOCK_STATE_LOCKED;
      }
      lastComfortLockFrameMs = now;
      portEXIT_CRITICAL(&airliftMux);
      logLine("comfort fob: %s", code == kComfortFobUnlockCode ? "unlock" : "lock");
      DEBUG_CAN("comfort fob %s (id 0x%03X code 0x%02X)",
                code == kComfortFobUnlockCode ? "unlock" : "lock",
                (unsigned)frame.identifier, (unsigned)code);
      break;
    }

    default:
      break;
  }
}

bool canActive() {
  const uint32_t last = lastCanFrameMs;
  if (last == 0) return false;
  const uint32_t silentMs = millis() - last;
  const uint32_t thresholdMs = (uint32_t)canSilenceSec * 1000u;
  // "Off" only when bus is BOTH quiet long enough AND running below the fps floor.
  const bool quiet  = silentMs >= thresholdMs;
  const bool slow   = canCurrentFps < canMinFps;
  return !(quiet && slow);
}

bool canTrafficContinuousFor(uint32_t durationMs) {
  const uint32_t last = lastCanFrameMs;
  const uint32_t since = canTrafficSinceMs;
  const uint32_t now = millis();
  return last != 0 && now - last <= kCanTrafficGapMs && since != 0 && now - since >= durationMs;
}

bool canDriverRunning() {
  if (!s_driverInstalled) return false;
  twai_status_info_t status = {};
  return twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_RUNNING;
}

void canBroadcastTick() {
  if (!s_driverInstalled || !canBroadcastEnabled) return;

  static uint32_t lastSentMs = 0;
  static uint8_t  seq        = 0;
  const uint32_t now = millis();
  if (now - lastSentMs < kCanBroadcastPeriodMs) return;
  lastSentMs = now;

  const uint8_t mode = currentMode & 0x03;
  const uint8_t flags = (compressorOn      ? 0x01 : 0x00)
                      | (ignitionOn        ? 0x02 : 0x00)
                      | (interceptMode     ? 0x04 : 0x00)
                      | (passThroughMode   ? 0x08 : 0x00)
                      | (uint8_t)(mode << 4);

  twai_message_t m = {};
  m.identifier       = canBroadcastId & 0x7FF;
  m.extd             = 0;
  m.rtr              = 0;
  m.data_length_code = 8;
  m.data[0] = pressureFL;
  m.data[1] = pressureFR;
  m.data[2] = pressureRL;
  m.data[3] = pressureRR;
  m.data[4] = pressureTank;
  m.data[5] = flags;
  m.data[6] = seq++;
  m.data[7] = 0;

  if (twai_transmit(&m, 0) == ESP_OK) {
    canBroadcastSent++;
  } else {
    canBroadcastErrors++;
  }
}
