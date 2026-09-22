#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "airlift.h"
#include "API.h"
#include "CAN.h"
#include "defs.h"
#include "globals.h"
#include "io.h"

// ---------------------------------------------------------------------------
// Transform-in-flight MITM.
//
// The firmware never seizes the manifold bus. The real handheld keeps polling
// the manifold at its own perfect cadence; we simply REWRITE its frames as they
// pass through (see transformHandheldFrame / transformManifoldFrame in
// airlift.cpp). The manifold therefore only ever sees ONE continuous, correctly
// framed master, so it can never desync — and a web-commanded mode PERSISTS
// because every subsequent handheld poll is rewritten to hold it.
//
// `passThroughMode` (user setting / debug switch) is a HARD override: while it
// is on we behave as a pure byte-for-byte relay and apply NO transforms, so a
// tech can watch the untouched OEM traffic on the bus.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Pumps. Default is a real-time byte-for-byte relay (the fast path). Injection
// is done WITHOUT buffering: when the web UI is driving a command, at the
// handheld's next frame boundary we send OUR frame instead and drop the
// handheld's own frame for that one poll slot. Correct timing, no echo juggling.
// ---------------------------------------------------------------------------

// Log a frame, de-duplicating immediate repeats so heartbeats don't bury the
// transitions. Skips the plain idle poll (01 00 05).
static void logWire(const char* tag, const uint8_t* p, uint8_t n) {
  static uint8_t lastTx[12]; static uint8_t lastTxN = 0;
  const bool isIdle = (n >= 3 && p[0] == 0x01 && p[1] == 0x00 && p[2] == 0x05);
  if (isIdle) return;
  if (n == lastTxN && memcmp(p, lastTx, n) == 0) return;
  lastTxN = n < 12 ? n : 12;
  memcpy(lastTx, p, lastTxN);

  char b[48]; int k = 0;
  for (uint8_t i = 0; i < n && i < 12 && k < (int)sizeof(b) - 4; i++)
    k += snprintf(b + k, sizeof(b) - k, "%02X ", p[i]);
  DEBUG_LIN("%s %s", tag, b);
}

// Logical bus <-> physical UART mapping. If the installer swapped the two plugs
// we detect it (see below) and transparently swap the roles so nothing else in
// the code — parsing, injection, echo accounting — has to care.
static inline HardwareSerial& handheldUart() { return busReversed ? manifoldSerial : controllerSerial; }
static inline HardwareSerial& manifoldUart() { return busReversed ? controllerSerial : manifoldSerial; }

// While a command is active we drive the manifold fast (~20/s) but the handheld
// only polls ~5/s. Relaying every reply back to it floods it and trips an OEM
// comm-error. This credit lets exactly one manifold "response set" through to the
// handheld per real handheld poll: h2m bumps it on each poll, m2h spends it.
static volatile int16_t handheldReplyCredit = 0;

static void handheldToManifoldTask(void* arg) {
  (void)arg;
  FrameAssembler assembler;   // side-channel: parse the handheld's real traffic
  AirliftFrame frame;
  AirliftFrame inj;           // scratch for a synthesised frame
  uint32_t lastInjectMs = 0;
  bool wasReversed = busReversed;
  uint8_t wrongTypeRun = 0;   // consecutive non-poll frames seen on the handheld UART

  // While a web command is active we DRIVE the manifold ourselves at ~this rate
  // (reply-paced) instead of once per slow handheld poll, so buttons and mode
  // switches respond quickly instead of crawling at the handheld's ~5/s cadence.
  constexpr uint32_t kInjectMinGapMs  = 45;   // ~20/s — the OEM's active cadence
  constexpr uint32_t kInjectTimeoutMs = 90;   // poll anyway if the manifold is quiet

  for (;;) {
    if (busReversed != wasReversed) {          // wiring just got re-detected
      controllerEchoSkip = 0;
      manifoldEchoSkip   = 0;
      assembler = FrameAssembler{};
      wasReversed = busReversed;
    }
    const bool cmd = !passThroughMode && webOverrideActive();

    while (handheldUart().available()) {
      const uint8_t b = (uint8_t)handheldUart().read();
      rxBytesController++;

      if (controllerEchoSkip > 0) { controllerEchoSkip--; continue; }

      // Side-channel parse of the handheld's real traffic (keeps counters,
      // lastButton*, pressures current even while we drive the manifold).
      if (assembler.feed(b, frame)) {
        portENTER_CRITICAL(&airliftMux);
        framesFromController++;
        memcpy(lastFromControllerFrame, frame.payload, frame.len);
        lastFromControllerLen = frame.len;
        lastLinRxMs = millis();
        portEXIT_CRITICAL(&airliftMux);

        // Wiring auto-detect: the handheld UART should carry POLL frames (0x01).
        // If we keep seeing RESPONSE frames (0x10) instead, the plugs are swapped
        // — flip the role mapping (a one-shot; it self-stabilises after).
        if (frame.framed && frame.len >= 1) {
          if (frame.payload[0] == FT_POLL) {
            wrongTypeRun = 0;
            lastHandheldFrameMs = millis();
            // Each real handheld poll earns one response set back from us.
            if (cmd && handheldReplyCredit < 3) handheldReplyCredit++;
          } else if (frame.payload[0] == FT_RESPONSE) {
            if (++wrongTypeRun >= 6) {
              busReversed = !busReversed;
              wrongTypeRun = 0;
              logLine("bus auto-detect: plugs swapped -> reversed=%d", (int)busReversed);
            }
          }
        }

        parseHandheldPoll(frame);
        if (!cmd) logWire(">M:", frame.payload, frame.len);
      }

      if (cmd) continue;                 // command active: we drive; drop handheld polls

      // RESTING: real-time byte relay (the fast path).
      manifoldUart().write(b);
      manifoldEchoSkip++;
    }

    if (cmd) {
      // Fast, reply-paced drive of the manifold while a command is active.
      const uint32_t now = millis();
      const bool replied = (lastInjectMs != 0) &&
                           ((int32_t)(lastManifoldReplyMs - lastInjectMs) > 0);
      const bool minGap  = (uint32_t)(now - lastInjectMs) >= kInjectMinGapMs;
      const bool timeout = (uint32_t)(now - lastInjectMs) >= kInjectTimeoutMs;
      if (lastInjectMs == 0 || (replied && minGap) || timeout) {
        inj.framed = true;
        inj.len    = 1;
        inj.payload[0] = FT_POLL;
        if (transformHandheldFrame(inj)) {
          logWire(">M:", inj.payload, inj.len);
          manifoldEchoSkip += inj.len + (inj.framed ? 2 : 0);
          sendFrame(manifoldUart(), inj.payload, inj.len, inj.framed);
          lastInjectMs = now;
        }
      }
    } else {
      lastInjectMs = 0;
    }
    vTaskDelay(1);
  }
}

static void manifoldToHandheldTask(void* arg) {
  (void)arg;
  FrameAssembler assembler;
  AirliftFrame frame;
  bool wasReversed = busReversed;
  bool wasCmd = false;

  for (;;) {
    if (busReversed != wasReversed) {
      controllerEchoSkip = 0;
      manifoldEchoSkip   = 0;
      assembler = FrameAssembler{};
      wasReversed = busReversed;
    }
    const bool cmd = !passThroughMode && webOverrideActive();
    if (cmd != wasCmd) {
      assembler = FrameAssembler{};
      wasCmd = cmd;
    }

    while (manifoldUart().available()) {
      const uint8_t b = (uint8_t)manifoldUart().read();
      rxBytesManifold++;

      if (manifoldEchoSkip > 0) { manifoldEchoSkip--; continue; }

      // -------------------------------------------------------------------
      // RESTING: real-time byte relay (fast path), verbatim.
      // -------------------------------------------------------------------
      if (!cmd) {
        handheldUart().write(b);
        controllerEchoSkip++;
        if (assembler.feed(b, frame)) {
          portENTER_CRITICAL(&airliftMux);
          framesFromManifold++;
          memcpy(lastFromManifoldFrame, frame.payload, frame.len);
          lastFromManifoldLen = frame.len;
          lastLinRxMs = millis();
          if (frame.framed && frame.len >= 1 && frame.payload[0] == FT_RESPONSE)
            lastManifoldFrameMs = millis();
          portEXIT_CRITICAL(&airliftMux);
          parsePressureBroadcast(frame);
          parseManifoldReply(frame);
          parseStatusFrame(frame);
        }
        continue;
      }

      // -------------------------------------------------------------------
      // COMMAND: buffer manifold frames and relay only ~one response set per
      // real handheld poll (credit-gated) so our fast drive doesn't flood the
      // handheld and trip an OEM comm-error. Mode-ack bytes are masked to the
      // handheld's own poll type so its view stays consistent.
      // -------------------------------------------------------------------
      if (!assembler.feed(b, frame)) continue;

      portENTER_CRITICAL(&airliftMux);
      framesFromManifold++;
      memcpy(lastFromManifoldFrame, frame.payload, frame.len);
      lastFromManifoldLen = frame.len;
      lastLinRxMs = millis();
      if (frame.framed && frame.len >= 1 && frame.payload[0] == FT_RESPONSE)
        lastManifoldFrameMs = millis();
      portEXIT_CRITICAL(&airliftMux);
      parsePressureBroadcast(frame);
      parseManifoldReply(frame);
      parseStatusFrame(frame);

      const bool isReply = frame.framed && frame.len >= 2 &&
                           frame.payload[0] == FT_RESPONSE &&
                           frame.payload[1] == ST_CTRL_REPLY;

      // Forward ONLY the control reply, masked to look like the handheld's own
      // poll (idle while it idle-polls). Everything else the manifold emits
      // while we drive it — the 17-byte status frame and the pressure broadcast
      // — is parsed for our own tracking but NOT relayed: a handheld that thinks
      // it is idle must never see the compressor running or pressures climbing,
      // or it flags a comms error. It just sees a quiet, do-nothing manifold
      // until control returns. Credit-gated to ~one reply per real handheld poll.
      if (isReply && handheldReplyCredit > 0) {
        handheldReplyCredit--;
        AirliftFrame o = frame;
        if (o.len >= 4) {   // mask mode-ack to the handheld's poll type
          switch (lastButtonB2) {
            case 0x41: o.payload[2] = 0x04; o.payload[3] = 0x01; break;
            case 0x14: o.payload[2] = 0x01; o.payload[3] = 0x04; break;
            default:   o.payload[2] = 0x00; o.payload[3] = 0x05; break;
          }
        }
        controllerEchoSkip += o.len + (o.framed ? 2 : 0);
        sendFrame(handheldUart(), o.payload, o.len, o.framed);
      }
    }

    AirliftFrame stale;
    if (assembler.flushOnIdle(millis(), stale)) {
      parsePressureBroadcast(stale);
      // While commanding, the format-B pressure broadcast is parsed for our own
      // tracking but NOT forwarded to the handheld (see above) — it would reveal
      // the pressures changing under an "idle" manifold and trip a comms error.
    }
    vTaskDelay(1);
  }
}

// Ignition / power state machine. The ignition signal comes from one of two
// sources (user-selectable): CAN presence (default) OR a hard-wired aux GPIO
// (ignitionSenseGpio) which is instant and doesn't wait on a CAN silence
// timeout. When the aux GPIO is used, a grace buffer (ignitionOffGraceMs)
// debounces a brief dropout before air-out is triggered.
static void ignitionTask(void* arg) {
  (void)arg;
  enum State { ST_OFF, ST_ON, ST_OFFGRACE, ST_POSTDELAY };
  State state = ST_OFF;
  uint32_t stateEnter = 0;

  // Ignition present? Aux GPIO (HIGH = on) when enabled, else CAN presence.
  auto ignPresent = []() -> bool {
    if (ignitionSenseGpio) return digitalRead(pinIgnitionSense) == HIGH;
    return canActive();
  };

  bool manualTargetActive = false;
  bool manualTargetUp = false;
  uint8_t manualTargetFront = 0;
  uint8_t manualTargetRear = 0;
  uint32_t manualTargetUntilMs = 0;
  uint32_t manualTargetNextPulseMs = 0;

  // A fob air-up/down request, held until controllerBootDelayMs after the
  // handheld powered on so its boot / mode handshake can finish first.
  bool     pendingFobActive = false;
  bool     pendingFobUp     = false;
  uint32_t pendingFobFireMs = 0;

  auto queueTarget = [&](bool up, uint8_t preset, uint8_t front, uint8_t rear, const char* source) {
    if (passThroughMode) return;
    manualTargetActive = false;
    if (preset < kPresetCount) {
      portENTER_CRITICAL(&airliftMux);
      desiredMode                = MODE_PRESET;
      if (currentMode != MODE_PRESET) presetEnterUntilMs = millis() + 400;
      pendingPresetFrontPsi      = presetFrontPsi[preset];
      pendingPresetRearPsi       = presetRearPsi[preset];
      pendingPresetRepeatUntilMs = millis() + airOutDurationMs;
      portEXIT_CRITICAL(&airliftMux);
      logLine("%s: preset %u", source, (unsigned)(preset + 1));
      return;
    }

    manualTargetActive = true;
    manualTargetUp = up;
    manualTargetFront = front;
    manualTargetRear = rear;
    manualTargetUntilMs = millis() + kAutoLevelTimeoutMs;
    manualTargetNextPulseMs = 0;
    logLine("%s: manual %s %u/%u psi", source, up ? "up" : "down",
            (unsigned)front, (unsigned)rear);
  };

  auto tickManualTarget = [&](uint32_t now) {
    if (!manualTargetActive || now < manualTargetNextPulseMs) return;
    if ((int32_t)(now - manualTargetUntilMs) >= 0) {
      manualTargetActive = false;
      logLine("auto target: timed out");
      return;
    }

    const uint8_t front = (psiFromRaw(pressureFL) + psiFromRaw(pressureFR)) / 2;
    const uint8_t rear = (psiFromRaw(pressureRL) + psiFromRaw(pressureRR)) / 2;
    const bool frontNeedsMove = manualTargetUp
      ? front + kAutoLevelTolerancePsi < manualTargetFront
      : front > manualTargetFront + kAutoLevelTolerancePsi;
    const bool rearNeedsMove = manualTargetUp
      ? rear + kAutoLevelTolerancePsi < manualTargetRear
      : rear > manualTargetRear + kAutoLevelTolerancePsi;
    if (!frontNeedsMove && !rearNeedsMove) {
      manualTargetActive = false;
      logLine("auto target: reached %u/%u psi", (unsigned)front, (unsigned)rear);
      return;
    }

    // Manual button encoding supports a pair of corners per axle.
    const uint8_t axleBits = frontNeedsMove ? 0x03 : 0x0C;
    const uint8_t code = (manualTargetUp ? 0x50 : 0x60) | axleBits;
    portENTER_CRITICAL(&airliftMux);
    desiredMode                = MODE_MANUAL;
    presetEnterUntilMs         = 0;
    pendingButtonCode          = code;
    pendingButtonRepeatUntilMs = now + kAutoLevelPulseMs;
    pendingButtonRelease       = false;
    portEXIT_CRITICAL(&airliftMux);
    manualTargetNextPulseMs = now + kAutoLevelIntervalMs;
  };

  auto triggerAirOut = [&](uint32_t now, bool force) {
    if ((!force && !airOutOnIgnOff) || passThroughMode) return;
    queueTarget(false, airDownPreset, airDownFrontPsi, airDownRearPsi, "air-out");
  };

  for (;;) {
    pollCanRx();
    canBroadcastTick();
    serviceDeferredWifi();
    const bool ign = ignPresent();
    const uint32_t now = millis();
    tickManualTarget(now);

    if (useComfortCan) {
      // CAN activity already applies the configured silence timeout. Do not
      // require an extra Comfort-CAN traffic/fob condition before driving the
      // high-side output.
      const bool canIgnitionActive = canActive();
      setControllerPower(canIgnitionActive);
      ignitionOn = canIgnitionActive;

      // Schedule (don't immediately run) fob air actions. On a cold wake the
      // handheld needs a few seconds to boot and finish its Preset/Manual
      // handshake; a preset sent before then trips a comms error. Hold the
      // action until controllerBootDelayMs after the controller powered on
      // (fires immediately if it was already booted long enough).
      if (comfortFobDoubleUnlock) {
        comfortFobDoubleUnlock = false;
        if (airUpOnFobDouble) {
          pendingFobActive = true;
          pendingFobUp     = true;
          pendingFobFireMs = controllerPoweredAtMs + controllerBootDelayMs;
          const int32_t waitMs = (int32_t)(pendingFobFireMs - now);
          logLine("fob double unlock: air-up in %ldms (boot settle)",
                  (long)(waitMs > 0 ? waitMs : 0));
        } else {
          logLine("comfort fob: double unlock ignored (disabled)");
        }
      }
      if (comfortFobDoubleLock) {
        comfortFobDoubleLock = false;
        if (airDownOnFobDouble) {
          pendingFobActive = true;
          pendingFobUp     = false;
          pendingFobFireMs = controllerPoweredAtMs + controllerBootDelayMs;
          const int32_t waitMs = (int32_t)(pendingFobFireMs - now);
          logLine("fob double lock: air-down in %ldms (boot settle)",
                  (long)(waitMs > 0 ? waitMs : 0));
        } else {
          logLine("comfort fob: double lock ignored (disabled)");
        }
      }

      // Fire the scheduled action once the boot-settle window has elapsed.
      if (pendingFobActive && (int32_t)(now - pendingFobFireMs) >= 0) {
        pendingFobActive = false;
        if (pendingFobUp)
          queueTarget(true, airUpPreset, airUpFrontPsi, airUpRearPsi, "fob double unlock");
        else
          queueTarget(false, airDownPreset, airDownFrontPsi, airDownRearPsi, "fob double lock");
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    switch (state) {
      case ST_OFF:
        if (ign) {
          setControllerPower(true);
          ignitionOn = true;
          state = ST_ON;
          stateEnter = now;
          logLine("ignition ON — handheld powered");
        }
        break;

      case ST_ON:
        if (!ign) {
          ignitionOn = false;
          if (ignitionSenseGpio) {
            // Aux-GPIO ignition: buffer before airing out.
            state = ST_OFFGRACE;
            stateEnter = now;
            logLine("ignition OFF (aux) — %us buffer before air-out",
                    (unsigned)(ignitionOffGraceMs / 1000));
          } else {
            logLine("ignition OFF");
            triggerAirOut(now, false);
            state = ST_POSTDELAY;
            stateEnter = now;
          }
        }
        break;

      case ST_OFFGRACE:
        if (ign) {
          ignitionOn = true;
          state = ST_ON;
          stateEnter = now;
          logLine("ignition back within buffer — air-out cancelled");
        } else if (now - stateEnter >= ignitionOffGraceMs) {
          triggerAirOut(now, false);
          state = ST_POSTDELAY;
          stateEnter = now;
        }
        break;

      case ST_POSTDELAY:
        if (ign) {
          ignitionOn = true;
          state = ST_ON;
          stateEnter = now;
        } else if (now - stateEnter >= postAirOutDelayMs) {
          setControllerPower(false);
          state = ST_OFF;
          stateEnter = now;
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void debugTask(void* arg) {
  (void)arg;
  for (;;) {
    DEBUG("ign=%u pwr=%u passthru=%u inter=%u mode=%u  rxC=%lu rxM=%lu  C->M=%lu M->C=%lu CAN=%lu",
          (unsigned)ignitionOn, (unsigned)controllerPowered,
          (unsigned)passThroughMode, (unsigned)interceptMode,
          (unsigned)currentMode,
          (unsigned long)rxBytesController,
          (unsigned long)rxBytesManifold,
          (unsigned long)framesFromController,
          (unsigned long)framesFromManifold,
          (unsigned long)canFramesSeen);
    DEBUG("P FL=%u(%uPSI) FR=%u(%uPSI) RL=%u(%uPSI) RR=%u(%uPSI) T=%u",
          pressureFL, psiFromRaw(pressureFL),
          pressureFR, psiFromRaw(pressureFR),
          pressureRL, psiFromRaw(pressureRL),
          pressureRR, psiFromRaw(pressureRR),
          pressureTank);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void startTasks() {
  xTaskCreatePinnedToCore(handheldToManifoldTask, "h2m", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(manifoldToHandheldTask, "m2h", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(ignitionTask,           "ign", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(debugTask,              "dbg", 4096, nullptr, 1, nullptr, 0);
}
