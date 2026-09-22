#pragma once

void setupWiFi();
void setupApiServer();
void loadPreferences();

// Completes a deferred WiFi bringup after a wake from reduced power. Pumped
// from ignitionTask so the WiFi radio only comes back once the LIN relay has
// re-established, keeping the handheld's cold-boot handshake undisturbed.
void serviceDeferredWifi();
