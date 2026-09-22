#include "SavvyCAN.h"

#include <Arduino.h>
#include <WiFi.h>

#include "globals.h"

namespace {

constexpr uint16_t kPort = 23;
constexpr size_t kQueueDepth = 8;
constexpr uint32_t kControlWriteTimeoutMs = 250;

struct QueuedFrame {
  twai_message_t frame;
  uint32_t timestampUs;
};

enum GvretState { WAIT_FOR_COMMAND, READ_COMMAND, READ_PAYLOAD };

QueueHandle_t queue = nullptr;
WiFiServer server(kPort);
WiFiClient client;
bool serverStarted = false;
bool serialStarted = false;
GvretState parserState = WAIT_FOR_COMMAND;
uint8_t command = 0;
uint8_t payload[16] = {};
size_t payloadIndex = 0;
size_t payloadExpected = 0;
uint8_t enableSequence = 0;

void resetParser() {
  parserState = WAIT_FOR_COMMAND;
  command = 0;
  payloadIndex = 0;
  payloadExpected = 0;
  enableSequence = 0;
}

bool writeControl(const uint8_t* data, size_t length) {
  if (savvyCanSerialEnabled) {
    return Serial.write(data, length) == length;
  }
  if (!client || !client.connected()) return false;

  const uint32_t started = millis();
  size_t sent = 0;
  while (sent < length && client.connected()) {
    const size_t written = client.write(data + sent, length - sent);
    if (written) {
      sent += written;
    } else if (millis() - started >= kControlWriteTimeoutMs) {
      return false;
    } else {
      vTaskDelay(1);
    }
  }
  return sent == length;
}

void sendNumBuses() {
  const uint8_t reply[] = {0xF1, 0x0C, 0x01};
  writeControl(reply, sizeof(reply));
}

void sendBusInfo() {
  const uint8_t reply[] = {0xF1, 0x06, 0x01, 0x00, 0xF4, 0x01,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  writeControl(reply, sizeof(reply));
}

void sendExtendedBusInfo() {
  uint8_t reply[17] = {};
  reply[0] = 0xF1;
  reply[1] = 0x0D;
  writeControl(reply, sizeof(reply));
}

void sendDeviceInfo() {
  const uint8_t reply[] = {0xF1, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  writeControl(reply, sizeof(reply));
}

void sendTimeSync() {
  const uint32_t now = micros();
  const uint8_t reply[] = {0xF1, 0x01, static_cast<uint8_t>(now),
                           static_cast<uint8_t>(now >> 8),
                           static_cast<uint8_t>(now >> 16),
                           static_cast<uint8_t>(now >> 24)};
  writeControl(reply, sizeof(reply));
}

void transmitHostFrame() {
  if (payloadIndex < 7) return;
  const uint32_t rawId = static_cast<uint32_t>(payload[0]) |
                         (static_cast<uint32_t>(payload[1]) << 8) |
                         (static_cast<uint32_t>(payload[2]) << 16) |
                         (static_cast<uint32_t>(payload[3]) << 24);
  twai_message_t frame = {};
  frame.extd = (rawId & 0x80000000UL) != 0;
  frame.identifier = frame.extd ? (rawId & 0x1FFFFFFFUL) : (rawId & 0x7FF);
  frame.data_length_code = payload[5] > 8 ? 8 : payload[5];
  for (uint8_t index = 0; index < frame.data_length_code; ++index) {
    frame.data[index] = payload[6 + index];
  }
  twai_transmit(&frame, pdMS_TO_TICKS(5));
}

void processCommand() {
  switch (command) {
    case 0x00: transmitHostFrame(); break;
    case 0x01: sendTimeSync(); break;
    case 0x06: sendBusInfo(); break;
    case 0x07: sendDeviceInfo(); break;
    case 0x09: { const uint8_t reply[] = {0xF1, 0x09}; writeControl(reply, sizeof(reply)); break; }
    case 0x0C: sendNumBuses(); break;
    case 0x0D: sendExtendedBusInfo(); break;
    default: break;
  }
}

void processByte(uint8_t input) {
  if (enableSequence < 2) {
    enableSequence = input == 0xE7 ? enableSequence + 1 : 0;
    return;
  }
  switch (parserState) {
    case WAIT_FOR_COMMAND:
      if (input == 0xF1) parserState = READ_COMMAND;
      break;
    case READ_COMMAND:
      command = input;
      payloadIndex = 0;
      if (command == 0x00) {
        payloadExpected = 6;
        parserState = READ_PAYLOAD;
      } else if (command == 0x05) {
        payloadExpected = 9;
        parserState = READ_PAYLOAD;
      } else if (command == 0x0E) {
        payloadExpected = 13;
        parserState = READ_PAYLOAD;
      } else {
        processCommand();
        parserState = WAIT_FOR_COMMAND;
      }
      break;
    case READ_PAYLOAD:
      if (payloadIndex >= sizeof(payload)) {
        parserState = WAIT_FOR_COMMAND;
        break;
      }
      payload[payloadIndex++] = input;
      if (payloadIndex == 6) payloadExpected = 7 + payload[5];
      if (payloadIndex >= payloadExpected) {
        processCommand();
        parserState = WAIT_FOR_COMMAND;
      }
      break;
  }
}

void sendFrame(const QueuedFrame& entry) {
  uint8_t packet[19] = {};
  uint32_t id = entry.frame.identifier & 0x1FFFFFFF;
  if (entry.frame.extd) id |= 0x80000000UL;
  packet[0] = 0xF1;
  packet[1] = 0x00;
  packet[2] = static_cast<uint8_t>(entry.timestampUs);
  packet[3] = static_cast<uint8_t>(entry.timestampUs >> 8);
  packet[4] = static_cast<uint8_t>(entry.timestampUs >> 16);
  packet[5] = static_cast<uint8_t>(entry.timestampUs >> 24);
  packet[6] = static_cast<uint8_t>(id);
  packet[7] = static_cast<uint8_t>(id >> 8);
  packet[8] = static_cast<uint8_t>(id >> 16);
  packet[9] = static_cast<uint8_t>(id >> 24);
  packet[10] = entry.frame.data_length_code & 0x0F;
  for (uint8_t index = 0; index < entry.frame.data_length_code; ++index) {
    packet[11 + index] = entry.frame.data[index];
  }
  const size_t length = 11 + entry.frame.data_length_code;
  if (savvyCanSerialEnabled) {
    Serial.write(packet, length);
  } else if (client && client.connected()) {
    client.write(packet, length);
  }
}

void task(void*) {
  for (;;) {
    if (!savvyCanWifiEnabled && !savvyCanSerialEnabled) {
      if (client) client.stop();
      serverStarted = false;
      serialStarted = false;
      resetParser();
      vTaskDelay(1);
      continue;
    }

    if (savvyCanSerialEnabled) {
      if (!serialStarted) {
        Serial.begin(1000000);
        serialStarted = true;
        resetParser();
      }
      while (Serial.available()) processByte(static_cast<uint8_t>(Serial.read()));
    } else {
      serialStarted = false;
      if (WiFi.getMode() == WIFI_OFF) {
        if (client) client.stop();
        serverStarted = false;
        resetParser();
        vTaskDelay(1);
        continue;
      }
      if (!serverStarted) {
        server.begin();
        server.setNoDelay(true);
        serverStarted = true;
      }
      if ((!client || !client.connected()) && serverStarted) {
        if (client) client.stop();
        client = server.available();
        resetParser();
      }
      while (client && client.connected() && client.available()) {
        processByte(static_cast<uint8_t>(client.read()));
      }
    }

    QueuedFrame entry;
    while (xQueueReceive(queue, &entry, 0) == pdTRUE) sendFrame(entry);
    vTaskDelay(1);
  }
}

}  // namespace

void savvyCanInit() {
  if (!queue) queue = xQueueCreate(kQueueDepth, sizeof(QueuedFrame));
  xTaskCreate(task, "savvycan", 4096, nullptr, 13, nullptr);
  DEBUG_SAVVY("init (wifi=%d serial=%d)", (int)(bool)savvyCanWifiEnabled,
              (int)(bool)savvyCanSerialEnabled);
}

void savvyCanSetWifiEnabled(bool enabled) {
  savvyCanWifiEnabled = enabled;
  if (enabled) savvyCanSerialEnabled = false;
  if (queue) xQueueReset(queue);
  DEBUG_SAVVY("WiFi GVRET %s", enabled ? "ON (tcp:23)" : "OFF");
}

void savvyCanSetSerialEnabled(bool enabled) {
  savvyCanSerialEnabled = enabled;
  if (enabled) savvyCanWifiEnabled = false;
  if (queue) xQueueReset(queue);
  // NB: no DEBUG here — enabling serial GVRET takes over the USB UART at
  // 1 Mbaud; a debug line would land in the binary GVRET stream.
}

bool savvyCanRequiresCanTransmit() {
  return savvyCanWifiEnabled || savvyCanSerialEnabled;
}

void savvyCanQueueFrame(const twai_message_t& frame) {
  if ((!savvyCanWifiEnabled && !savvyCanSerialEnabled) || !queue) return;
  const QueuedFrame entry = {frame, micros()};
  if (xQueueSend(queue, &entry, 0) != pdTRUE) savvyCanFramesDropped++;
}