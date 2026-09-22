#include "airlift.h"

#include <string.h>

#include "globals.h"

// ---------------------------------------------------------------------------
// FrameAssembler — handles both FA-framed and 14-byte broadcast frames.
// ---------------------------------------------------------------------------
bool FrameAssembler::feed(uint8_t b, AirliftFrame& out) {
  lastByteMs_ = millis();

  switch (state_) {
    case IDLE:
      if (b == SOF_BYTE) {
        state_ = IN_FA;
        len_   = 0;
        preambleMatchIdx_ = 0;
        return false;
      }
      // Start matching format-B preamble.
      if (b == kBcastPreamble[0]) {
        state_ = IN_BCAST;
        len_ = 0;
        buf_[len_++] = b;
        preambleMatchIdx_ = 1;
      }
      return false;

    case IN_FA:
      if (b == EOF_BYTE) {
        return emit(out, /*framed=*/true);
      }
      if (b == SOF_BYTE) {
        // Resync inside a frame.
        len_ = 0;
        return false;
      }
      if (len_ >= kMaxFrameLen) {
        portENTER_CRITICAL(&airliftMux);
        framesDroppedBadLen++;
        portEXIT_CRITICAL(&airliftMux);
        state_ = IDLE;
        len_ = 0;
        return false;
      }
      buf_[len_++] = b;
      return false;

    case IN_BCAST:
      buf_[len_++] = b;
      // While we're still within the preamble window, every byte must match.
      if (preambleMatchIdx_ < sizeof(kBcastPreamble)) {
        if (b != kBcastPreamble[preambleMatchIdx_]) {
          // Mismatch — abandon.
          state_ = IDLE;
          len_ = 0;
          preambleMatchIdx_ = 0;
          return false;
        }
        preambleMatchIdx_++;
      }
      if (len_ >= kBcastFrameLen) {
        return emit(out, /*framed=*/false);
      }
      if (len_ >= kMaxFrameLen) {
        state_ = IDLE;
        len_ = 0;
        return false;
      }
      return false;
  }
  return false;
}

bool FrameAssembler::flushOnIdle(uint32_t nowMs, AirliftFrame& out) {
  if (state_ == IDLE || len_ == 0) return false;
  if (nowMs - lastByteMs_ < kFrameIdleMs) return false;
  // Idle-flush: only emit if it looks meaningful. For FA frames we drop a
  // partial (didn't see EOF). For broadcast frames we emit what we have.
  if (state_ == IN_BCAST && len_ >= 8) {
    return emit(out, /*framed=*/false);
  }
  state_ = IDLE;
  len_ = 0;
  preambleMatchIdx_ = 0;
  return false;
}

bool FrameAssembler::emit(AirliftFrame& out, bool framed) {
  out.framed = framed;
  out.len = len_;
  memcpy(out.payload, buf_, len_);
  state_ = IDLE;
  len_ = 0;
  preambleMatchIdx_ = 0;
  return out.len > 0;
}

// ---------------------------------------------------------------------------
// Parsers
// ---------------------------------------------------------------------------
bool parsePressureBroadcast(const AirliftFrame& f) {
  if (f.framed) return false;
  if (f.len < kBcastFrameLen) return false;
  if (memcmp(f.payload, kBcastPreamble, sizeof(kBcastPreamble)) != 0) return false;

  portENTER_CRITICAL(&airliftMux);
  pressureFL = f.payload[kBcastIdxFL]; // front left pressure 
  pressureFR = f.payload[kBcastIdxFR]; // front right pressure 
  pressureRL = f.payload[kBcastIdxRL]; // rear left pressure 
  pressureRR = f.payload[kBcastIdxRR]; // rear right pressure 
  if (kBcastIdxTank != 0xFF && kBcastIdxTank < f.len) {
    pressureTank = f.payload[kBcastIdxTank]; // tank pressure 
  }
  lastPressureBroadcastMs = millis();
  portEXIT_CRITICAL(&airliftMux);
  return true;
}

bool parseManifoldReply(const AirliftFrame& f) {
  if (!f.framed) return false;
  if (f.len < 4) return false;
  if (f.payload[0] != FT_RESPONSE) return false;
  if (f.payload[1] != ST_CTRL_REPLY) return false;

  // Mode discriminator: bytes [2][3] = 04 01 (manual) or 01 04 (preset)
  AirliftMode m = MODE_UNKNOWN;
  if (f.payload[2] == 0x04 && f.payload[3] == 0x01) m = MODE_MANUAL;
  else if (f.payload[2] == 0x01 && f.payload[3] == 0x04) m = MODE_PRESET;
  else if (f.payload[2] == 0x00 && f.payload[3] == 0x05) {
    // Idle reply. Normally keep the previous mode. BUT if we're actively
    // winding the manifold out of preset (a manual request is in progress),
    // an idle reply is the confirmation that preset has been released — this
    // manifold answers idle (not a 10 F1 04 01 manual ack) to our wind-down
    // polls — so report MANUAL.
    m = (manualEnterUntilMs != 0) ? MODE_MANUAL : (AirliftMode)currentMode;
  }
  // Log distinct non-idle replies (mode acks) so the manifold's real state is
  // visible without the steady-state chatter burying it.
  if (f.payload[2] != 0x00 || f.payload[3] != 0x05) {
    static uint8_t lastB2 = 0xFF, lastB3 = 0xFF;
    if (f.payload[2] != lastB2 || f.payload[3] != lastB3) {
      lastB2 = f.payload[2];
      lastB3 = f.payload[3];
      logLine("M<: %02X %02X %02X %02X -> mode=%u",
              f.payload[0], f.payload[1], f.payload[2], f.payload[3], (unsigned)m);
      DEBUG_LIN("M<: %02X %02X %02X %02X -> mode=%u",
            f.payload[0], f.payload[1], f.payload[2], f.payload[3], (unsigned)m);
    }
  }
  portENTER_CRITICAL(&airliftMux);
  currentMode = m;
  lastManifoldReplyMs = millis();
  portEXIT_CRITICAL(&airliftMux);
  return true;
}

bool parseStatusFrame(const AirliftFrame& f) {
  if (!f.framed) return false;
  if (f.len != kStatusFrameLen) return false;
  if (f.payload[0] != ST_PRESSURES_HDR0) return false;
  if (f.payload[1] != ST_PRESSURES_HDR1) return false;

  const bool comp = (f.payload[kStatusIdxModeFlags] & kCompressorOnBits)
                    == kCompressorOnBits;
  const uint32_t now = millis();

  portENTER_CRITICAL(&airliftMux);
  // Status-frame pressures are raw PSI (not PSI*2 like format B). We store
  // them in the same globals after scaling so psiFromRaw() keeps working.
  pressureFL = rawFromPsi(f.payload[kStatusIdxFL]);
  pressureRL = rawFromPsi(f.payload[kStatusIdxRL]);
  pressureFR = rawFromPsi(f.payload[kStatusIdxFR]);
  pressureRR = rawFromPsi(f.payload[kStatusIdxRR]);
  pressureTank = f.payload[kStatusIdxTank]; // stored directly as PSI, not PSI*2
  lastPressureBroadcastMs = now;
  if (comp != compressorOn) {
    compressorOn = comp;
    lastCompressorChangeMs = now;
  }
  portEXIT_CRITICAL(&airliftMux);
  return true;
}


bool parseHandheldPoll(const AirliftFrame& f) {
  if (!f.framed) return false;
  if (f.len < 3) return false;
  if (f.payload[0] != FT_POLL) return false;

  portENTER_CRITICAL(&airliftMux);
  lastButtonB2 = f.payload[1];
  lastButtonB3 = f.payload[2];
  lastButtonB4 = f.len > 3 ? f.payload[3] : 0;
  lastButtonTimestampMs = millis();
  portEXIT_CRITICAL(&airliftMux);

  const bool physicalButton = (f.payload[1] == 0x41 && f.payload[2] != 0) ||
                              (f.payload[1] == 0x16 && f.payload[2] == 0x47);
  // Track the physical controller's live mode from its poll type
  // (0x41 = manual, 0x14 = preset; 
  // A CHANGE is the user physically using the buttons: last-touched-wins, so hand control back
  // to the controller (adopt its mode, drop any web override — its own transition
  // frames will drive the manifold through the pass-through path).
  const uint8_t b2 = f.payload[1];
  if (b2 == 0x41 || b2 == 0x14) {
    const uint8_t pm = (b2 == 0x41) ? MODE_MANUAL : MODE_PRESET;
    if (pm != handheldMode) {
      portENTER_CRITICAL(&airliftMux);
      handheldMode          = pm;
      desiredMode           = pm;    // physical button just changed
      presetEnterUntilMs    = 0;
      manualEnterUntilMs    = 0;
      pendingButtonCode     = 0;
      pendingButtonRelease  = false;
      pendingPresetFrontPsi = 0xFF;
      pendingPresetRearPsi  = 0xFF;
      portEXIT_CRITICAL(&airliftMux);
    }
  }

  // Detect preset target broadcast (`01 16 47 TL TR 0F TL TR CHK`) and
  // cache the target pressures. Values are direct PSI (NOT PSI*2).
  if (f.len >= 9 && f.payload[1] == 0x16 && f.payload[2] == 0x47) {
    const uint8_t front = f.payload[3];
    const uint8_t rear  = f.payload[4];
    portENTER_CRITICAL(&airliftMux);
    lastPresetTargetFrontPsi = front;
    lastPresetTargetRearPsi  = rear;
    // Preset learn mode: the user armed a specific slot in the UI; store this
    // target into THAT slot so the slot number matches the physical button they
    // just pressed (the wire carries no preset index):
    // only the controller knows the pressure request, so we need to 'learn' what that is.
    if (learnActive && (int32_t)(millis() - learnUntilMs) < 0
        && learnSlot >= 0 && learnSlot < kPresetCount) {
      learnFrontPsi[learnSlot] = front;
      learnRearPsi[learnSlot]  = rear;
      learnMask |= (uint8_t)(1u << learnSlot);
      learnSlot = -1;
    }
    portEXIT_CRITICAL(&airliftMux);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------
void sendFrame(HardwareSerial& port, const uint8_t* payload, uint8_t len, bool framed) {
  if (framed) port.write(SOF_BYTE);
  port.write(payload, len);
  if (framed) port.write(EOF_BYTE);
  // NOTE: no port.flush() — with a TX buffer configured, write() queues the
  // bytes and the UART sends them in the background. Flushing here would block
  // the task for ~6-9 ms at 9600 baud on every frame,
  // which starves the controller and throws an error.
}

// ---------------------------------------------------------------------------
// Frame Builders
// ---------------------------------------------------------------------------
uint8_t buildIdlePoll(uint8_t* out) {
  out[0] = FT_POLL;
  out[1] = 0x00;
  out[2] = 0x05;
  return 3;
}

uint8_t buildManualButtonPoll(uint8_t btnCode, uint8_t* out) {
  // FA 01 41 XX YY F3, where YY = 0xC4 - XX (verified across captures).
  out[0] = FT_POLL;
  out[1] = 0x41;
  out[2] = btnCode;
  out[3] = (uint8_t)(0xC4 - btnCode);
  return 4;
}

uint8_t buildPresetTargetPoll(uint8_t frontPsi, uint8_t rearPsi, uint8_t* out) {
  // FA 01 16 47 TL TR 0F TL TR CHK F3
  // Verified from captures: TL/TR are plain PSI (NOT PSI*2).
  // Sum of all 9 payload bytes = 0x206 (verified across 6 captured frames).
  out[0] = FT_POLL;
  out[1] = 0x16;
  out[2] = 0x47;
  out[3] = frontPsi;
  out[4] = rearPsi;
  out[5] = 0x0F;
  out[6] = frontPsi;
  out[7] = rearPsi;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += out[i];
  out[8] = (uint8_t)((0x206 - sum) & 0xFF);
  return 9;
}

uint8_t buildEnterManualPoll(uint8_t* out) {
  // FA 01 41 80 44 F3 — mode-switch button; enters MANUAL from any state.
  return buildManualButtonPoll(BTN_MANUAL_MODE_SWITCH, out);
}

uint8_t buildSteadyPresetPoll(uint8_t* out) {
  // FA 01 14 43 00 00 00 AE F3 — steady PRESET heartbeat the controller
  // sends while in PRESET mode. Must be established before the manifold
  // will accept a mode-switch back to MANUAL.
  out[0] = FT_POLL;
  out[1] = 0x14;
  out[2] = 0x43;
  out[3] = 0x00;
  out[4] = 0x00;
  out[5] = 0x00;
  out[6] = 0xAE;
  return 7;
}

uint8_t buildEnterPresetPoll(uint8_t* out) {
  // FA 01 14 40 00 00 00 B1 F3
  // Checksum rule: all payload bytes sum mod 256 = 0x06
  // 0x01+0x14+0x40+0x00+0x00+0x00 = 0x55; CHK = (0x06-0x55)&0xFF = 0xB1
  out[0] = FT_POLL;
  out[1] = 0x14;
  out[2] = 0x40;
  out[3] = 0x00;
  out[4] = 0x00;
  out[5] = 0x00;
  out[6] = 0xB1;
  return 7;
}

uint8_t buildPresetReleasePoll(uint8_t* out) {
  // FA 01 14 41 00 00 00 B0 F3 — the "release preset" poll the handheld sends
  // when leaving preset. This manifold LATCHES preset and ignores manual polls
  // until it receives this frame; the next 01 41 poll then flips it to MANUAL.
  // Sum rule: 0x01+0x14+0x41 = 0x56; CHK = (0x06-0x56)&0xFF = 0xB0.
  out[0] = FT_POLL;
  out[1] = 0x14;
  out[2] = 0x41;
  out[3] = 0x00;
  out[4] = 0x00;
  out[5] = 0x00;
  out[6] = 0xB0;
  return 7;
}

// ---------------------------------------------------------------------------
// Pass-through Logic
// ---------------------------------------------------------------------------
bool webOverrideActive() {
  if (passThroughMode) return false;          // Any queued web action forces an override.

  if (pendingButtonCode != 0 || pendingButtonRelease || pendingPresetFrontPsi != 0xFF)
    return true;
  if (manualEnterUntilMs != 0 && (int32_t)(millis() - manualEnterUntilMs) < 0)
    return true;
  if (presetEnterUntilMs != 0 && (int32_t)(millis() - presetEnterUntilMs) < 0)
    return true;
  // Otherwise: hold the web-selected mode ONLY while it differs from the mode the
  // physical controller is currently driving. When they agree we rest in 
  // pass-through (fast path; the controller's real buttons work).
  return desiredMode != handheldMode;
}

bool transformHandheldFrame(AirliftFrame& f) {
  if (passThroughMode) return false;
  if (!f.framed) return false;
  if (f.len < 1 || f.payload[0] != FT_POLL) return false;

  const uint32_t now = millis();

  // ----- Held PRESET: rewrite EVERY controller poll into preset traffic. -----
  if (desiredMode == MODE_PRESET) {
    if (presetEnterUntilMs != 0 && (int32_t)(now - presetEnterUntilMs) < 0) {
      // Establishment window: send the "enter preset" poll (01 14 40).
      f.len = buildEnterPresetPoll(f.payload);
    } else if (pendingPresetFrontPsi != 0xFF && pendingPresetRearPsi != 0xFF
               && (int32_t)(now - pendingPresetRepeatUntilMs) < 0) {
      // Freshly selected target: push the target-PSI frame (01 16 47 ...).
      f.len = buildPresetTargetPoll(pendingPresetFrontPsi, pendingPresetRearPsi,
                                    f.payload);
    } else {
      // Clear an expired target and hold with the steady heartbeat (01 14 43).
      if (pendingPresetFrontPsi != 0xFF
          && (int32_t)(now - pendingPresetRepeatUntilMs) >= 0) {
        portENTER_CRITICAL(&airliftMux);
        pendingPresetFrontPsi = 0xFF;
        pendingPresetRearPsi  = 0xFF;
        portEXIT_CRITICAL(&airliftMux);
      }
      f.len = buildSteadyPresetPoll(f.payload);
    }
    f.framed = true;
    return true;
  }

  // ----- MANUAL: only override while the web UI drives a button. -----
  // Exit-preset sequence: manifold LATCHES preset
  // and ignores manual polls until it gets the "release preset" frame 01 14 41.
  //   1) send 01 14 41 (release the preset latch)
  //   2) then 01 41 61 (button 5 / FL-down — a no-op at low PSI) which now flips
  //      it to MANUAL (10 F1 04 01).
  // Completes when the manifold acks manual (currentMode == MANUAL).
  if (manualEnterUntilMs != 0) {
    if (currentMode == MODE_MANUAL) {
      portENTER_CRITICAL(&airliftMux);
      manualEnterUntilMs = 0;                 // manifold confirmed manual — done
      portEXIT_CRITICAL(&airliftMux);
      DEBUG_LIN("manual-enter: switched -> MANUAL");
    } else if ((int32_t)(now - manualEnterUntilMs) < 0) {
      if ((uint32_t)(manualEnterUntilMs - now) > 1600UL) {
        f.len = buildPresetReleasePoll(f.payload);                   // 01 14 41 release latch
      } else {
        f.len = buildManualButtonPoll(BTN_MANUAL_FL_DOWN, f.payload); // 01 41 61 flip to manual
      }
      f.framed = true;
      return true;
    } else {
      // Window expired without a manual ack — trust that we've left preset.
      portENTER_CRITICAL(&airliftMux);
      manualEnterUntilMs = 0;
      currentMode = MODE_MANUAL;
      portEXIT_CRITICAL(&airliftMux);
      DEBUG_LIN("manual-enter: window expired -> forcing MANUAL");
    }
  }

  if (pendingButtonCode != 0) {
    if ((int32_t)(now - pendingButtonRepeatUntilMs) < 0) {
      f.len = buildManualButtonPoll(pendingButtonCode, f.payload);
      f.framed = true;
      return true;
    }
    // Hold window expired without a UI release — queue one so the manifold
    // doesn't think the button is still held.
    portENTER_CRITICAL(&airliftMux);
    pendingButtonCode    = 0;
    pendingButtonRelease = true;
    portEXIT_CRITICAL(&airliftMux);
  }
  if (pendingButtonRelease) {
    portENTER_CRITICAL(&airliftMux);
    pendingButtonRelease = false;
    portEXIT_CRITICAL(&airliftMux);
    f.len = buildManualButtonPoll(BTN_MANUAL_RELEASE, f.payload);
    f.framed = true;
    return true;
  }

  // Steady MANUAL hold: the transition is done but the physical controller is
  // still driving a DIFFERENT mode. Keep the
  // manifold in manual by continuously sending the 01 41 00 manual hold poll,
  // otherwise the controller's own preset polls would drag it back into preset.
  if (desiredMode == MODE_MANUAL && handheldMode != MODE_MANUAL) {
    f.len = buildManualButtonPoll(BTN_MANUAL_RELEASE, f.payload);
    f.framed = true;
    return true;
  }

  // Resting in MANUAL with nothing queued: pass the handheld frame so
  // the original buttons keep working.
  return false;
}

bool transformManifoldFrame(AirliftFrame& f) {
  if (!webOverrideActive()) return false;
  if (!f.framed) return false;
  if (f.len < 4) return false;
  if (f.payload[0] != FT_RESPONSE || f.payload[1] != ST_CTRL_REPLY) return false;

  // Rewrite the mode ack so it matches the poll the handheld actually sent
  // (lastButtonB2 = handheld payload[1]), so the handheld never sees a reply
  // that disagrees with its own request.
  uint8_t b2, b3;
  switch (lastButtonB2) {
    case 0x41: b2 = 0x04; b3 = 0x01; break;   // handheld manual -> manual ack
    case 0x14: b2 = 0x01; b3 = 0x04; break;   // handheld preset -> preset ack
    default:   b2 = 0x00; b3 = 0x05; break;   // idle
  }
  if (f.payload[2] == b2 && f.payload[3] == b3) return false;
  f.payload[2] = b2;
  f.payload[3] = b3;
  return true;
}

