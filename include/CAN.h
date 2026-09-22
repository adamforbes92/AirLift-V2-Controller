#pragma once

#include <stdint.h>
#include <driver/twai.h>

void canInit();
void pollCanRx();
bool canActive();    // true if a CAN frame was seen recently
bool canTrafficContinuousFor(uint32_t durationMs);
bool canDriverRunning();

// Feed every received CAN frame to vehicle-state consumers.
void canProcessFrame(const twai_message_t& frame);

// Reinitialise the TWAI driver. Call after canBroadcastEnabled changes so the
// driver mode (LISTEN_ONLY vs NORMAL) is updated.
void canReinit();

// Send the periodic 8-byte pressures+status broadcast if enabled and due.
// Safe to call from any task loop; internally rate-limited.
void canBroadcastTick();
