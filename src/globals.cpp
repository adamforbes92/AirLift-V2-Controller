#include "globals.h"

HardwareSerial controllerSerial(1);
HardwareSerial manifoldSerial(2);
Preferences preferences;
AsyncWebServer server(80);

volatile uint8_t pressureFL = 0;
volatile uint8_t pressureFR = 0;
volatile uint8_t pressureRL = 0;
volatile uint8_t pressureRR = 0;
volatile uint16_t pressureTank = 0;
volatile uint32_t lastPressureBroadcastMs = 0;

volatile bool     compressorOn = false;
volatile uint32_t lastCompressorChangeMs = 0;

volatile uint8_t  currentMode = MODE_UNKNOWN;
volatile uint32_t lastManifoldReplyMs = 0;

volatile uint8_t lastPresetTargetFrontPsi = 0;
volatile uint8_t lastPresetTargetRearPsi  = 0;

uint8_t presetFrontPsi[kPresetCount] = {0,0,0,0,0,0,0,0};
uint8_t presetRearPsi[kPresetCount]  = {0,0,0,0,0,0,0,0};

volatile uint8_t lastButtonB2 = 0;
volatile uint8_t lastButtonB3 = 0;
volatile uint8_t lastButtonB4 = 0;
volatile uint32_t lastButtonTimestampMs = 0;

volatile uint32_t rxBytesController   = 0;
volatile uint32_t rxBytesManifold     = 0;
volatile uint32_t txBytesToManifold   = 0;
volatile uint32_t txBytesToController = 0;
volatile uint32_t framesFromController = 0;
volatile uint32_t framesFromManifold   = 0;
volatile uint32_t framesDroppedBadLen  = 0;
volatile uint32_t canFramesSeen        = 0;
volatile uint32_t lastCanFrameMs       = 0;
volatile uint32_t canTrafficSinceMs    = 0;
volatile uint16_t canCurrentFps        = 0;
volatile uint16_t canSilenceSec        = kCanSilenceSecDefault;
volatile uint16_t canMinFps            = kCanMinFpsDefault;
volatile bool     canBroadcastEnabled  = false;
volatile uint32_t canBroadcastId       = kCanBroadcastIdDefault;
volatile uint32_t canBroadcastSent     = 0;
volatile uint32_t canBroadcastErrors   = 0;
volatile bool     savvyCanWifiEnabled  = false;
volatile bool     savvyCanSerialEnabled = false;
volatile uint32_t savvyCanFramesDropped = 0;
volatile bool     usePowertrainCan = true;
volatile bool     useComfortCan = false;
volatile uint8_t  comfortLockState = LOCK_STATE_UNKNOWN;
volatile uint32_t lastComfortLockFrameMs = 0;
volatile uint32_t lastComfortFobUnlockMs = 0;
volatile uint32_t lastComfortFobLockMs = 0;
volatile bool     comfortFobDoubleUnlock = false;
volatile bool     comfortFobDoubleLock = false;

char        presetNames[kPresetCount][24]  = {
  "Preset 1", "Preset 2", "Preset 3", "Preset 4",
  "Preset 5", "Preset 6", "Preset 7", "Preset 8",
};

volatile bool    airOutOnIgnOff = false;
volatile uint8_t zeroPsiPreset  = kZeroPresetManual;
volatile bool    airUpOnFobDouble = false;
volatile bool    airDownOnFobDouble = false;
volatile uint8_t airUpPreset = kZeroPresetManual;
volatile uint8_t airDownPreset = kZeroPresetManual;
volatile uint8_t airUpFrontPsi = 0;
volatile uint8_t airUpRearPsi = 0;
volatile uint8_t airDownFrontPsi = 0;
volatile uint8_t airDownRearPsi = 0;
volatile bool    ignitionSenseGpio = false;
volatile bool    passThroughMode = true;

volatile bool    interceptMode = false;

volatile int16_t manifoldEchoSkip   = 0;
volatile int16_t controllerEchoSkip = 0;

volatile uint8_t  pendingButtonCode = 0;
volatile uint32_t pendingButtonRepeatUntilMs = 0;
volatile bool     pendingButtonRelease = false;
volatile uint8_t  pendingPresetFrontPsi = 0xFF;
volatile uint8_t  pendingPresetRearPsi  = 0xFF;
volatile uint32_t pendingPresetRepeatUntilMs = 0;
volatile uint8_t  desiredMode        = MODE_MANUAL;
volatile uint8_t  handheldMode       = MODE_MANUAL;
volatile uint32_t presetEnterUntilMs = 0;
volatile uint32_t manualEnterUntilMs = 0;

volatile bool controllerPowered = false;
volatile uint32_t controllerPoweredAtMs = 0;
volatile uint16_t controllerBootDelayMs = kControllerBootDelayMsDefault;
volatile bool ignitionOn        = false;
volatile bool highSideOverrideActive = false;
volatile bool highSideOverrideOn = false;

volatile uint32_t lastLinRxMs         = 0;
volatile uint32_t lastHandheldFrameMs = 0;
volatile uint32_t lastManifoldFrameMs = 0;
volatile bool     busReversed         = false;

volatile bool     learnActive   = false;
volatile uint32_t learnUntilMs  = 0;
uint8_t           learnFrontPsi[kPresetCount] = {0};
uint8_t           learnRearPsi[kPresetCount]  = {0};
volatile int8_t   learnSlot     = -1;
volatile uint8_t  learnMask     = 0;

uint8_t lastFromControllerFrame[kMaxFrameLen] = {0};
uint8_t lastFromControllerLen = 0;
uint8_t lastFromManifoldFrame[kMaxFrameLen]   = {0};
uint8_t lastFromManifoldLen   = 0;

portMUX_TYPE airliftMux = portMUX_INITIALIZER_UNLOCKED;

LogEntry          logBuffer[kLogLineCount] = {};
volatile uint32_t logWriteIndex            = 0;
static portMUX_TYPE logMux                 = portMUX_INITIALIZER_UNLOCKED;

void logLine(const char* fmt, ...) {
  char tmp[kLogLineLen];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  portENTER_CRITICAL(&logMux);
  const uint32_t i = logWriteIndex % kLogLineCount;
  logBuffer[i].ms = millis();
  strlcpy(logBuffer[i].text, tmp, sizeof(logBuffer[i].text));
  logWriteIndex++;
  portEXIT_CRITICAL(&logMux);
}
