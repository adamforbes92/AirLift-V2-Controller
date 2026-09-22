#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "defs.h"

struct StoredFrame {
  uint8_t bytes[kMaxFrameLen];
  uint8_t len;   // 0 = empty
};

extern HardwareSerial controllerSerial;
extern HardwareSerial manifoldSerial;
extern Preferences preferences;
extern AsyncWebServer server;

// Raw 0..255 pressure bytes (PSI*2). Use psiFromRaw() to convert.
// pressureTank is stored directly in PSI (not PSI*2) as tank can exceed 127 PSI.
extern volatile uint8_t pressureFL;
extern volatile uint8_t pressureFR;
extern volatile uint8_t pressureRL;
extern volatile uint8_t pressureRR;
extern volatile uint16_t pressureTank;
extern volatile uint32_t lastPressureBroadcastMs;

// Compressor running state, decoded from status-frame payload[3] & 0x05.
extern volatile bool     compressorOn;
extern volatile uint32_t lastCompressorChangeMs;

// Current mode reported by the manifold (see AirliftMode in defs.h).
extern volatile uint8_t  currentMode;
extern volatile uint32_t lastManifoldReplyMs;

// Cached preset target pressures last seen on the wire (PSI integer).
extern volatile uint8_t lastPresetTargetFrontPsi;
extern volatile uint8_t lastPresetTargetRearPsi;

// Per-preset target pressures (front/rear PSI) configured via the web UI.
// 0 = unset / don't trigger.
extern uint8_t presetFrontPsi[kPresetCount];
extern uint8_t presetRearPsi[kPresetCount];

extern volatile uint8_t lastButtonB2;
extern volatile uint8_t lastButtonB3;
extern volatile uint8_t lastButtonB4;
extern volatile uint32_t lastButtonTimestampMs;

extern volatile uint32_t rxBytesController;     // raw bytes received from controller UART
extern volatile uint32_t rxBytesManifold;       // raw bytes received from manifold UART
extern volatile uint32_t txBytesToManifold;     // bytes forwarded/sent to manifold UART
extern volatile uint32_t txBytesToController;   // bytes forwarded/sent to controller UART
extern volatile uint32_t framesFromController;
extern volatile uint32_t framesFromManifold;
extern volatile uint32_t framesDroppedBadLen;
extern volatile uint32_t canFramesSeen;
extern volatile uint32_t lastCanFrameMs;
extern volatile uint32_t canTrafficSinceMs;
// Live CAN frame rate, recomputed once per second from canFramesSeen.
extern volatile uint16_t canCurrentFps;
// Runtime-configurable thresholds (see defs.h). canActive() returns false
// only when silence > canSilenceSec AND live fps < canMinFps.
extern volatile uint16_t canSilenceSec;
extern volatile uint16_t canMinFps;
// Broadcast: if enabled, the firmware emits an 8-byte status frame on the
// configured ID at ~10 Hz. Driver must be (re)initialised in NORMAL (not
// LISTEN_ONLY) mode for transmits to leave the chip.
extern volatile bool     canBroadcastEnabled;
extern volatile uint32_t canBroadcastId;
extern volatile uint32_t canBroadcastSent;
extern volatile uint32_t canBroadcastErrors;

extern volatile bool     savvyCanWifiEnabled;
extern volatile bool     savvyCanSerialEnabled;
extern volatile uint32_t savvyCanFramesDropped;

extern volatile bool     usePowertrainCan;
extern volatile bool     useComfortCan;
extern volatile uint8_t  comfortLockState;
extern volatile uint32_t lastComfortLockFrameMs;
extern volatile uint32_t lastComfortFobUnlockMs;
extern volatile uint32_t lastComfortFobLockMs;
extern volatile bool     comfortFobDoubleUnlock;
extern volatile bool     comfortFobDoubleLock;

extern char        presetNames[kPresetCount][24];

extern volatile bool    airOutOnIgnOff;
extern volatile uint8_t zeroPsiPreset;     // 0..7 = preset, 0xFF = Manual
extern volatile bool    airUpOnFobDouble;
extern volatile bool    airDownOnFobDouble;
extern volatile uint8_t airUpPreset;
extern volatile uint8_t airDownPreset;
extern volatile uint8_t airUpFrontPsi;
extern volatile uint8_t airUpRearPsi;
extern volatile uint8_t airDownFrontPsi;
extern volatile uint8_t airDownRearPsi;
// When true, ignition is sensed from the aux-input GPIO (pinIgnitionSense)
// instead of CAN presence: an instant, hard-wired signal with a fixed grace
// buffer (ignitionOffGraceMs) before air-out. Alternative to CAN detection.
extern volatile bool    ignitionSenseGpio;
extern volatile bool    passThroughMode;   // true = pure MITM, no captures/injects/air-out

// Intercept mode: when true, frames from the real handheld are dropped and
// the firmware acts as the bus master itself, injecting synthesised polls.
extern volatile bool    interceptMode;

// Passthrough echo-skip counters.
// In passthrough mode we forward every byte from one bus to the other.
// Because TX and RX share the same LIN wire, every byte we transmit is
// echoed back on RX. For every byte written to manifoldSerial we skip
// one byte from manifoldSerial.RX (manifoldEchoSkip), and vice-versa.
// No blocking needed — bytes arrive in FIFO order so the count is exact.
extern volatile int16_t manifoldEchoSkip;
extern volatile int16_t controllerEchoSkip;

// Web-UI command state. These no longer "seize the bus": instead the
// handheld->manifold pump REWRITES the handheld's own polls in-flight to
// carry whatever is queued here (see transformHandheldFrame()), so the
// manifold only ever sees one continuous, correctly-framed master.
//
// `pendingButtonCode` = 0 means "no pending manual button".
extern volatile uint8_t pendingButtonCode;
extern volatile uint32_t pendingButtonRepeatUntilMs;
// Set true on UI button release: emit one explicit 01 41 00 (button-up)
// rewrite on the next handheld poll, then clear the flag.
extern volatile bool     pendingButtonRelease;
// Pending preset target: front/rear in PSI; 0xFF/0xFF = nothing pending.
extern volatile uint8_t  pendingPresetFrontPsi;
extern volatile uint8_t  pendingPresetRearPsi;
extern volatile uint32_t pendingPresetRepeatUntilMs;
// Web-UI's desired mode (the mode we continuously HOLD by rewriting polls).
//   MODE_MANUAL  = follow the handheld verbatim (rewrite only for web buttons)
//   MODE_PRESET  = rewrite every handheld poll into preset heartbeats/targets
extern volatile uint8_t  desiredMode;
// The physical controller's CURRENT mode, inferred from its live poll stream
// (0x41 poll => MANUAL, 0x14 poll => PRESET). A CHANGE here is a "knob moved"
// event that reclaims control for the handheld (last-touched-wins arbitration).
extern volatile uint8_t  handheldMode;
// Brief window after entering PRESET during which we emit the 01 14 40
// "enter preset" poll before settling to the 01 14 43 heartbeat (0 = none).
extern volatile uint32_t presetEnterUntilMs;
// Brief window after returning to MANUAL during which we wind the manifold out
// of PRESET (idle polls) then emit a 01 41 00 release so it reports MANUAL,
// before falling back to verbatim pass-through (0 = none).
extern volatile uint32_t manualEnterUntilMs;

extern volatile bool controllerPowered;
extern volatile uint32_t controllerPoweredAtMs;   // millis() when the high-side last turned ON
extern volatile uint16_t controllerBootDelayMs;   // wait after power-on before a fob air action
extern volatile bool ignitionOn;
// Runtime-only diagnostic override for the controller high-side output.
// It is intentionally not persisted, so reboot restores automatic control.
extern volatile bool highSideOverrideActive;
extern volatile bool highSideOverrideOn;

// ---------------------------------------------------------------------------
// Diagnostics / bus health / auto-wiring.
// ---------------------------------------------------------------------------
// Last time ANY valid LIN frame was seen on either bus (millis). Used by the UI
// to blank all live values back to "--" after a few seconds of silence.
extern volatile uint32_t lastLinRxMs;
// Per-bus liveness: last time we saw a valid handheld poll / manifold reply
// (whichever physical UART currently carries it — see busReversed).
extern volatile uint32_t lastHandheldFrameMs;
extern volatile uint32_t lastManifoldFrameMs;
// Auto-detected wiring. false = normal (handheld on controllerSerial), true =
// the two plugs are swapped and we transparently swap the logical roles so the
// user never has to re-pin. See handheldUart()/manifoldUart() in tasks.cpp.
extern volatile bool     busReversed;

// ---------------------------------------------------------------------------
// Preset learn mode: the UI "arms" a specific slot, then the user presses that
// preset on the physical controller. The next 01 16 47 target broadcast is
// stored into the armed slot (so the slot number always matches the physical
// button — the wire carries no preset index, so the operator maps it). A bit in
// learnMask marks each captured slot; learnSlot is the armed slot (-1 = none).
// ---------------------------------------------------------------------------
extern volatile bool     learnActive;
extern volatile uint32_t learnUntilMs;
extern uint8_t           learnFrontPsi[kPresetCount];
extern uint8_t           learnRearPsi[kPresetCount];
extern volatile int8_t   learnSlot;    // armed slot index, -1 = none armed
extern volatile uint8_t  learnMask;    // bit i set once slot i has been captured

extern uint8_t lastFromControllerFrame[kMaxFrameLen];
extern uint8_t lastFromControllerLen;
extern uint8_t lastFromManifoldFrame[kMaxFrameLen];
extern uint8_t lastFromManifoldLen;

extern portMUX_TYPE airliftMux;

// ---------------------------------------------------------------------------
// Diagnostic log — lock-free-ish ring buffer of fixed-size lines.
// Producers (tasks) call logLine(); consumers (HTTP) call snapshotLog().
// ---------------------------------------------------------------------------
constexpr size_t kLogLineLen   = 80;
constexpr size_t kLogLineCount = 64;

struct LogEntry {
  uint32_t ms;
  char     text[kLogLineLen];
};

extern LogEntry         logBuffer[kLogLineCount];
extern volatile uint32_t logWriteIndex;   // monotonically increasing

void logLine(const char* fmt, ...);
