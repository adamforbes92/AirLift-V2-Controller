#pragma once

// ---------------------------------------------------------------------------
// Firmware version
// ---------------------------------------------------------------------------
// V2.12 - OTA overhaul: shared ota_manager/wifi_manager v2 + ota.js (guided
//         GitHub update, Home WiFi bridge mode, recovery page).
#define FW_VERSION "2.12"

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

V2.12 - OTA overhaul (shared ota_manager / wifi_manager v2 + data/ota.js, ported
        from OpenHaldex 9.00): upload callbacks no longer answer mid-body (the
        old per-chunk "200 OK" made the browser drop the connection after the
        first 1.4 kB - a crash in AsyncTCP and a half-written partition, so no
        OTA through the UI had ever completed); filesystem updates unmount
        first, check the announced size, verify the mount and wipe on failure;
        boot only mounts a sane superblock and the web server always starts -
        with no usable UI "/" is a recovery page with the two uploads.
        "Update from GitHub" on the OTA tab (Releases/releases.json via
        tools/make_release.py) plus a Home WiFi (bridge mode) card; power_manager
        holds WiFi up while any browser is active. Assets served no-cache (ETag)
        instead of the hand-bumped ?v=.
*/
