#pragma once

#include <Arduino.h>
#include "defs.h"

struct AirliftFrame {
  uint8_t payload[kMaxFrameLen];
  uint8_t len;
  bool    framed;   // true = FA..F3, false = idle-terminated broadcast
};

// Frame assembler that recognises BOTH formats:
//   * FA <payload> F3                 (poll / response)
//   * 00 0A E6 7E ... (14 B total)    (pressure broadcast — see defs.h)
// Plus an idle-timeout flush for partial frames.
class FrameAssembler {
public:
  bool feed(uint8_t b, AirliftFrame& out);
  bool flushOnIdle(uint32_t nowMs, AirliftFrame& out);

private:
  enum State : uint8_t { IDLE, IN_FA, IN_BCAST };
  State    state_ = IDLE;
  uint8_t  buf_[kMaxFrameLen] = {0};
  uint8_t  len_ = 0;
  uint32_t lastByteMs_ = 0;

  // Used while sniffing the format-B preamble out of the stream.
  uint8_t  preambleMatchIdx_ = 0;

  bool emit(AirliftFrame& out, bool framed);
};

// Parse a received frame and update the global state. Returns true if the
// frame was understood (and the appropriate `pressure*` / `current*` globals
// have been updated).
bool parsePressureBroadcast(const AirliftFrame& f);
bool parseManifoldReply(const AirliftFrame& f);
bool parseHandheldPoll(const AirliftFrame& f);

// 17-byte FA-framed status frame from manifold (format A). Updates corner +
// tank pressures and the compressorOn flag.
bool parseStatusFrame(const AirliftFrame& f);

// Low-level frame transmit. For format-A frames, pass `framed = true` (the
// SOF/EOF bytes will be wrapped automatically). For format-B broadcasts pass
// `framed = false` (bytes are sent verbatim).
void sendFrame(HardwareSerial& port, const uint8_t* payload, uint8_t len, bool framed = true);

// ---------------------------------------------------------------------------
// Synthesised frame builders — used by the intercept-mode tasks to push our
// own commands to the manifold while ignoring the real handheld.
// ---------------------------------------------------------------------------
// Build the body of the four poll variants. `out` must be at least
// kMaxFrameLen bytes; returns the payload length (excluding FA/F3).

uint8_t buildIdlePoll(uint8_t* out);
uint8_t buildManualButtonPoll(uint8_t btnCode, uint8_t* out);
uint8_t buildPresetTargetPoll(uint8_t frontPsi, uint8_t rearPsi, uint8_t* out);
uint8_t buildEnterManualPoll(uint8_t* out);   // FA 01 41 80 44 F3
uint8_t buildEnterPresetPoll(uint8_t* out);   // FA 01 14 40 00 00 00 B1 F3
uint8_t buildSteadyPresetPoll(uint8_t* out);  // FA 01 14 43 00 00 00 AE F3
uint8_t buildPresetReleasePoll(uint8_t* out); // FA 01 14 41 00 00 00 B0 F3 (exit preset)

// ---------------------------------------------------------------------------
// Transform-in-flight (MITM). Instead of seizing the manifold bus, we let the
// real handheld keep driving it and REWRITE frames as they pass through, so the
// manifold always sees one continuous, correctly-framed master.
// ---------------------------------------------------------------------------
// True while the web UI is actively overriding the handheld (held PRESET, or a
// queued manual button / preset target). False = pure follow-the-handheld.
bool webOverrideActive();

// Rewrite a handheld->manifold poll in place to carry the web UI's current
// intent (preset heartbeat/target or manual button). Returns true if modified;
// leaves the frame untouched when resting in MANUAL with nothing queued.
bool transformHandheldFrame(AirliftFrame& f);

// Rewrite a manifold->handheld control reply (FA 10 F1 XX YY F3) so it matches
// the poll the handheld actually sent, preventing an OEM comm-error while we
// are overriding. No-op for status/pressure frames and when not overriding.
bool transformManifoldFrame(AirliftFrame& f);
