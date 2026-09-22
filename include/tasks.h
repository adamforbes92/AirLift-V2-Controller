#pragma once

#include "globals.h"

void startTasks();

// Queue a stored frame for injection on the manifold UART.
// holdMs > 0 ⇒ repeatedly inject the frame at ~50 ms intervals for that
// duration (mimics a press-and-hold button such as air-out).
