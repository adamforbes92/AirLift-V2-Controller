#pragma once

// ---------------------------------------------------------------------------
// Firmware version
// ---------------------------------------------------------------------------
// V2.11 - shared UI theme; common wifi_manager (airlift.local) + ota_manager
//         (firmware + filesystem OTA, /api/ota + /api/ota/fs); cache-busting.
#define FW_VERSION "2.11"

/*

V1.00 - ESP32 (DevKit V1 / WROOM-32) rewrite of the original Mega original attempt.
        Dual hardware-UART rewrite-in-flight Man-in-the-Middle between the AirLift controller
        and manifold
      - checksums and FA/F3 (frame acknowledgment) control + format-B wire decoder
      - WiFi AP web UI
      - learn buttons, ignition-aware high-side driver and auto air-out.
V2.01 - CAN / TWAI bridge: Powertrain quiet-bus ignition OR Comfort-CAN
        lock/unlock, ~10 Hz pressure broadcast, SavvyCAN GVRET (WiFi / Serial).

V2.10 - power management + wake fixes:
        * Reduced power (WiFi off, CPU 240->80 MHz) 1 min after the last WiFi
          CLIENT disconnects — independent of CAN activity (regulator runs cool
          while driving).
        * CAN wake-on-burst: a fresh quiet->active run of >= kCanWakeFrameMin
          frames restores WiFi
        * Deferred WiFi bring-up after wake (kWifiWakeSettleMs): the controller
          re-handshakes with the manifold on an undisturbed LIN relay first, so
          RF calibration no longer corrupts the cold-boot handshake.
        * Fob air-up/down waits a boot delay so a preset isn't sent mid-handshake.
        * Command mode sends the handheld only idle-looking acks and suppresses
          the manifold's status / pressure frames, so it no longer flags a comms
          error while we drive a preset.

*/
