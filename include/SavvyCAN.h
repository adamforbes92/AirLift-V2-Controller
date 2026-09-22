#pragma once

#include <driver/twai.h>

constexpr uint8_t kSavvyCanProtocolGvret = 0;

void savvyCanInit();
void savvyCanSetWifiEnabled(bool enabled);
void savvyCanSetSerialEnabled(bool enabled);
bool savvyCanRequiresCanTransmit();
void savvyCanQueueFrame(const twai_message_t& frame);