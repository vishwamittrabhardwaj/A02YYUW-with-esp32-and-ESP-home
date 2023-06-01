#pragma once

#include "esphome.h"

class A02YYUWSensor : public esphome::Component, public esphome::sensor::Sensor {
public:
  void setup() override;
  void loop() override;

private:
  HardwareSerial serial_ = HardwareSerial(2); // Use the second hardware serial port (GPIO16=RX, GPIO17=TX)
  uint8_t buffer_[4] = {};
  float distance_ = 0.0f;
};

void A02YYUWSensor::setup() {
  // Set up the serial port for the UART communication
  serial_.begin(9600, SERIAL_8N1, 16, 17); // baud rate, parity, RX pin, TX pin
}

void A02YYUWSensor::loop() {
  // Read the UART data from the sensor
  while (serial_.available()) {
    if (buffer_[0] == 0xFF && serial_.readBytes(buffer_ + 1, 3) == 3) {
      uint8_t sum = (buffer_[0] + buffer_[1] + buffer_[2]) & 0xFF;
      if (sum == buffer_[3]) {
        uint16_t distance_raw = (buffer_[1] << 8) | buffer_[2];
        if (distance_raw > 30) {
          distance_ = static_cast<float>(distance_raw) / 10.0f; // Convert to cm
        } else {
          distance_ = 0.0f; // Below the lower limit
        }
        break; // Successfully parsed a valid UART frame, exit the loop
      }
    } else {
      buffer_[0] = serial_.read(); // Invalid header, discard and try again
    }
  }

  // Update the sensor with the current distance value
  publish_state(distance_);

  // Your code for processing sensor data goes here
  // ...
}
