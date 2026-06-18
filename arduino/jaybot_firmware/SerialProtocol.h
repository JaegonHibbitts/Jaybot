#ifndef JAYBOT_SERIAL_PROTOCOL_H
#define JAYBOT_SERIAL_PROTOCOL_H

#include <Arduino.h>

// Command frame, all integer fields:
// @sequence,leftRPMx10,rightRPMx10,leftSteeringx10,rightSteeringx10*CRC\n
//
// Example:
// @104,900,900,900,900*AB\n
//
// Telemetry frame, all integer fields:
// !sequence,leftMeasuredRPMx10,rightMeasuredRPMx10,leftSignedPWM,
//  rightSignedPWM,leftTicks,rightTicks,status*CRC\n

struct DriveCommand {
  uint16_t sequence;
  int16_t leftRPMX10;
  int16_t rightRPMX10;
  int16_t leftSteeringX10;
  int16_t rightSteeringX10;
};

struct TelemetryData {
  uint16_t sequence;
  int16_t leftMeasuredRPMX10;
  int16_t rightMeasuredRPMX10;
  int16_t leftSignedPWM;
  int16_t rightSignedPWM;
  int32_t leftTicks;
  int32_t rightTicks;
  uint8_t status;
};

// Telemetry status bit mask. These are not command modes.
enum TelemetryStatus : uint8_t {
  STATUS_OK             = 0,
  STATUS_COMMS_TIMEOUT  = 1 << 0,
  STATUS_BAD_PACKET     = 1 << 1,
  STATUS_RX_OVERFLOW    = 1 << 2
};

class SerialProtocol {
 public:
  explicit SerialProtocol(HardwareSerial &serialPort);

  // Nonblocking byte-by-byte receiver.
  // Returns true when at least one complete, valid command was decoded.
  bool poll(DriveCommand &latestCommand);

  // Returns false instead of blocking if the hardware TX buffer does not
  // currently have room for the entire telemetry frame.
  bool sendTelemetry(const TelemetryData &telemetry);

  uint8_t getErrorFlags() const;
  void clearErrorFlags();

  static uint8_t calculateCRC8(
      const char *data,
      size_t length);

 private:
  static constexpr size_t RX_BUFFER_SIZE = 64;
  static constexpr size_t TELEMETRY_PAYLOAD_SIZE = 80;
  static constexpr size_t TELEMETRY_FRAME_SIZE = 88;

  HardwareSerial &serial;

  char receiveBuffer[RX_BUFFER_SIZE];
  size_t receiveIndex;
  bool receivingFrame;
  uint8_t errorFlags;

  bool parseCommandFrame(
      char *frame,
      DriveCommand &command);

  static bool parseSignedLong(
      const char *text,
      long minimum,
      long maximum,
      long &result);

  static int8_t hexadecimalValue(char character);
};

#endif
