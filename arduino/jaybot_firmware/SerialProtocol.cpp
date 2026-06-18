#include "SerialProtocol.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

namespace {
constexpr long MIN_RPM_X10 = -3500;
constexpr long MAX_RPM_X10 = 3500;
constexpr long MIN_STEERING_X10 = 0;
constexpr long MAX_STEERING_X10 = 1800;
}

SerialProtocol::SerialProtocol(HardwareSerial &serialPort)
    : serial(serialPort),
      receiveIndex(0),
      receivingFrame(false),
      errorFlags(STATUS_OK) {
  receiveBuffer[0] = '\0';
}

uint8_t SerialProtocol::calculateCRC8(
    const char *data,
    size_t length) {
  // CRC-8/ATM: polynomial 0x07, initial value 0x00.
  uint8_t crc = 0x00;

  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint8_t>(data[index]);

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x80) != 0) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

int8_t SerialProtocol::hexadecimalValue(char character) {
  if (character >= '0' && character <= '9') {
    return character - '0';
  }
  if (character >= 'A' && character <= 'F') {
    return character - 'A' + 10;
  }
  if (character >= 'a' && character <= 'f') {
    return character - 'a' + 10;
  }
  return -1;
}

bool SerialProtocol::parseSignedLong(
    const char *text,
    long minimum,
    long maximum,
    long &result) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *endPointer = nullptr;
  const long parsed = strtol(text, &endPointer, 10);

  if (*endPointer != '\0') {
    return false;
  }

  if (parsed < minimum || parsed > maximum) {
    return false;
  }

  result = parsed;
  return true;
}

bool SerialProtocol::parseCommandFrame(
    char *frame,
    DriveCommand &command) {
  // frame contains everything after '@' and before '\n'.
  char *asterisk = strchr(frame, '*');
  if (asterisk == nullptr) {
    return false;
  }

  // Exactly two hexadecimal CRC characters must follow '*'.
  if (asterisk[1] == '\0' ||
      asterisk[2] == '\0' ||
      asterisk[3] != '\0') {
    return false;
  }

  const int8_t crcHigh = hexadecimalValue(asterisk[1]);
  const int8_t crcLow = hexadecimalValue(asterisk[2]);
  if (crcHigh < 0 || crcLow < 0) {
    return false;
  }

  const uint8_t receivedCRC =
      static_cast<uint8_t>((crcHigh << 4) | crcLow);

  *asterisk = '\0';

  const uint8_t calculatedCRC =
      calculateCRC8(frame, strlen(frame));

  if (receivedCRC != calculatedCRC) {
    return false;
  }

  // Five integer fields:
  // sequence,leftRPMx10,rightRPMx10,leftSteeringx10,rightSteeringx10
  char *savePointer = nullptr;
  char *tokens[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};

  size_t tokenCount = 0;
  char *token = strtok_r(frame, ",", &savePointer);

  while (token != nullptr && tokenCount < 5) {
    tokens[tokenCount++] = token;
    token = strtok_r(nullptr, ",", &savePointer);
  }

  // Reject missing fields and extra fields.
  if (tokenCount != 5 || token != nullptr) {
    return false;
  }

  long sequence = 0;
  long leftRPMX10 = 0;
  long rightRPMX10 = 0;
  long leftSteeringX10 = 0;
  long rightSteeringX10 = 0;

  if (!parseSignedLong(tokens[0], 0, 65535, sequence) ||
      !parseSignedLong(
          tokens[1], MIN_RPM_X10, MAX_RPM_X10, leftRPMX10) ||
      !parseSignedLong(
          tokens[2], MIN_RPM_X10, MAX_RPM_X10, rightRPMX10) ||
      !parseSignedLong(
          tokens[3],
          MIN_STEERING_X10,
          MAX_STEERING_X10,
          leftSteeringX10) ||
      !parseSignedLong(
          tokens[4],
          MIN_STEERING_X10,
          MAX_STEERING_X10,
          rightSteeringX10)) {
    return false;
  }

  // Apply atomically only after every field and the CRC are valid.
  command.sequence = static_cast<uint16_t>(sequence);
  command.leftRPMX10 = static_cast<int16_t>(leftRPMX10);
  command.rightRPMX10 = static_cast<int16_t>(rightRPMX10);
  command.leftSteeringX10 =
      static_cast<int16_t>(leftSteeringX10);
  command.rightSteeringX10 =
      static_cast<int16_t>(rightSteeringX10);

  return true;
}

bool SerialProtocol::poll(DriveCommand &latestCommand) {
  bool decodedValidCommand = false;

  while (serial.available() > 0) {
    const char incoming =
        static_cast<char>(serial.read());

    // A new '@' always starts/restarts synchronization.
    if (incoming == '@') {
      receivingFrame = true;
      receiveIndex = 0;
      receiveBuffer[0] = '\0';
      continue;
    }

    if (!receivingFrame) {
      continue;
    }

    // Ignore carriage return so both '\n' and "\r\n" senders work.
    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      receiveBuffer[receiveIndex] = '\0';

      DriveCommand candidate;
      if (parseCommandFrame(receiveBuffer, candidate)) {
        latestCommand = candidate;
        decodedValidCommand = true;
      } else {
        errorFlags |= STATUS_BAD_PACKET;
      }

      receivingFrame = false;
      receiveIndex = 0;
      continue;
    }

    if (receiveIndex < RX_BUFFER_SIZE - 1) {
      receiveBuffer[receiveIndex++] = incoming;
    } else {
      // Discard the oversized frame and wait for the next '@'.
      receivingFrame = false;
      receiveIndex = 0;
      errorFlags |= STATUS_RX_OVERFLOW;
    }
  }

  return decodedValidCommand;
}

bool SerialProtocol::sendTelemetry(
    const TelemetryData &telemetry) {
  char payload[TELEMETRY_PAYLOAD_SIZE];

  const int payloadLength = snprintf(
      payload,
      sizeof(payload),
      "%u,%d,%d,%d,%d,%ld,%ld,%u",
      static_cast<unsigned int>(telemetry.sequence),
      static_cast<int>(telemetry.leftMeasuredRPMX10),
      static_cast<int>(telemetry.rightMeasuredRPMX10),
      static_cast<int>(telemetry.leftSignedPWM),
      static_cast<int>(telemetry.rightSignedPWM),
      static_cast<long>(telemetry.leftTicks),
      static_cast<long>(telemetry.rightTicks),
      static_cast<unsigned int>(telemetry.status));

  if (payloadLength <= 0 ||
      static_cast<size_t>(payloadLength) >= sizeof(payload)) {
    return false;
  }

  const uint8_t crc =
      calculateCRC8(payload, static_cast<size_t>(payloadLength));

  char frame[TELEMETRY_FRAME_SIZE];
  const int frameLength = snprintf(
      frame,
      sizeof(frame),
      "!%s*%02X\n",
      payload,
      static_cast<unsigned int>(crc));

  if (frameLength <= 0 ||
      static_cast<size_t>(frameLength) >= sizeof(frame)) {
    return false;
  }

  if (serial.availableForWrite() < frameLength) {
    return false;
  }

  serial.write(
      reinterpret_cast<const uint8_t *>(frame),
      static_cast<size_t>(frameLength));

  return true;
}

uint8_t SerialProtocol::getErrorFlags() const {
  return errorFlags;
}

void SerialProtocol::clearErrorFlags() {
  errorFlags = STATUS_OK;
}
