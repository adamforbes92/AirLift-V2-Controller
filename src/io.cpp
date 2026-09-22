#include "io.h"

#include <Arduino.h>

#include "API.h"
#include "defs.h"
#include "globals.h"

void setupPins()
{
  pinMode(pinWake_LIN, OUTPUT);          // LIN WAKE on TJA1020 — held HIGH for normal operation (same as MFSW controller)
  pinMode(pinCS_LIN, OUTPUT);            // LIN transceivers chip select / power enable — HIGH = enabled
  pinMode(pinControllerPower, OUTPUT);   // controller power to handheld
  pinMode(pinIgnitionSense, INPUT);      // aux input — optional ignition sense
  digitalWrite(pinWake_LIN, HIGH);       // held HIGH = normal operation (same as MFSW controller)
  digitalWrite(pinCS_LIN, HIGH);         // HIGH = transceivers enabled
  digitalWrite(pinControllerPower, LOW); // controller power off at boot (the handheld's own power switch is the master)  — HIGH = +12 V to handheld during air-out (ignition off)
  controllerPowered = false;
}

void setupSerial()
{
  // setRxBufferSize / setTxBufferSize MUST be called before begin() on ESP32.
  // A TX buffer makes write() non-blocking (it queues instead of waiting for the
  // wire), so the tasks never stall mid-forward at 9600 baud.
  controllerSerial.setRxBufferSize(1024);
  manifoldSerial.setRxBufferSize(1024);
  controllerSerial.setTxBufferSize(512);
  manifoldSerial.setTxBufferSize(512);
  controllerSerial.begin(airliftBaud, SERIAL_8N1, pinRX_Controller, pinTX_Controller);
  manifoldSerial.begin(airliftBaud, SERIAL_8N1, pinRX_Manifold, pinTX_Manifold);
}

// The auto (ignition / CAN) power intent, cached so a diagnostic override can be
// RELEASED and immediately return the pin to whatever the ignition logic wants
// — even in powertrain mode, where the state machine only calls
// setControllerPower() on transitions.
static bool s_autoPowerWanted = false;

static void applyControllerPower()
{
  const bool phys = highSideOverrideActive ? highSideOverrideOn : s_autoPowerWanted;
  if (phys == controllerPowered) return;
  digitalWrite(pinControllerPower, phys ? HIGH : LOW);
  controllerPowered = phys;
  if (phys) controllerPoweredAtMs = millis();   // stamp the cold-boot moment
  DEBUG_IO("Controller power -> %s%s", phys ? "ON" : "OFF",
        highSideOverrideActive ? " (forced)" : "");
}

void setControllerPower(bool on)
{
  s_autoPowerWanted = on;   // remember the auto intent, then apply (override wins)
  applyControllerPower();
}

void setHighSideOverride(bool on)
{
  highSideOverrideOn = on;
  highSideOverrideActive = true;
  applyControllerPower();
  DEBUG_IO("High-side override -> %s", on ? "ON" : "OFF");
}

// Release the diagnostic override and hand control straight back to the
// ignition / CAN logic (re-applies the cached auto intent at once). Without
// this, turning the force toggle "off" would latch the high-side output OFF and
// the controller would never wake on CAN traffic / unlock.
void clearHighSideOverride()
{
  if (!highSideOverrideActive) return;
  highSideOverrideActive = false;
  applyControllerPower();
  DEBUG_IO("High-side override released -> auto");
}

void basicInit()
{
#if enableDebug
  Serial.setTxBufferSize(2048); // keep DEBUG() logging from blocking the pumps
  Serial.begin(serialDebugBaud);
  delay(50);
#endif
  DEBUG("AirLift V2 ESP32 controller booting...");
  preferences.begin("airlift", false);
  loadPreferences();
  setupPins();
  setupSerial();
  DEBUG_IO("UARTs up @ %lu baud", (unsigned long)airliftBaud);
}
