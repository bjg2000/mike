/*!
 * @File  main.cpp
 * @brief  Adapted from DFRobot_IraserSensor.ino to use Hardware Serial (pins 0 and 1)
 * for continuous measurement with the SEN0366 sensor.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence  The MIT License (MIT)
 * @author  [liunian](nian.liu@dfrobot.com) - Original DFRobot code
 * @version  V1.0 - Original DFRobot code
 * @date  2020-08-13 - Original DFRobot code
 * @note   Modified for PlatformIO and Hardware Serial by Gemini
 */

#include <Arduino.h>

// NOTE: This code uses the hardware serial pins (0 and 1) for communication
// with the sensor. These pins are also used for the Serial Monitor.
// When the sensor is connected, you will likely not see output in the Serial Monitor.
// Disconnect the sensor from pins 0 and 1 when uploading code or using the Serial Monitor
// for debugging other parts of your sketch.

// Command to start continuous measurement mode (80 06 03 77)
// ADDR (80) | GRP (06) | CMD (03) | CS (77)
unsigned char continuousModeCommand[4] = {0x80, 0x06, 0x03, 0x77};

// Buffer to store incoming data from the sensor
unsigned char serialData[11] = {0};

void setup() {
  // Initialize hardware serial communication with the sensor
  // Set the baud rate to 9600 as specified in the sensor's protocol
  Serial.begin(9600);

  // Give the sensor a moment to power up and initialize
  delay(1000);

  // Send the command to start continuous measurement mode
  // Use Serial.write() to send raw bytes
  Serial.write(continuousModeCommand, sizeof(continuousModeCommand));

  // Note: You will likely not see this message in the Serial Monitor
  // because the sensor is using the hardware serial pins.
  // If you disconnect the sensor and reset the Arduino, you might see it.
  // Serial.println("Sending continuous mode command...");
}

void loop() {
  // Check if data is available to read from the sensor
  if (Serial.available() > 0) {
    // Read the incoming bytes into the buffer
    // We expect 11 bytes for a complete message in continuous mode
    // This simple approach assumes we receive 11 bytes at once.
    // A more robust approach would buffer bytes and look for message start/end.
    delay(50); // Small delay to allow all bytes of a message to arrive (may need adjustment)
    int bytesRead = Serial.readBytes(serialData, 11);

    if (bytesRead == 11) {
      // Calculate checksum
      unsigned char calculatedChecksum = 0;
      for (int i = 0; i < 10; i++) { // Sum the first 10 bytes
        calculatedChecksum += serialData[i];
      }
      // Apply the two's complement (reverse and add 1) and ensure it's an 8-bit value
      calculatedChecksum = (~calculatedChecksum + 1);

      // Validate checksum
      if (serialData[10] == calculatedChecksum) {
        // Check for error message ('ERR' in ASCII)
        // The error message format is ADDR 06 83 E R R ... CS
        // 'E' is 0x45, 'R' is 0x52
        if (serialData[3] == 'E' && serialData[4] == 'R' && serialData[5] == 'R') {
          // Note: You will likely not see this message in the Serial Monitor
          // Serial.println("Out of range");
        } else {
          // Extract distance (ASCII bytes from index 3 to 9)
          // The format is typically XXX.YYY or XXX.YYYY
          // We need to convert the ASCII characters to a float
          float distance = 0.0;
          // Assuming format like ddd.ddd or ddd.dddd
          // Find the decimal point (ASCII 0x2E)
          int decimalPointIndex = -1;
          for(int i = 3; i < 10; i++) {
              if (serialData[i] == 0x2E) {
                  decimalPointIndex = i;
                  break;
              }
          }

          if (decimalPointIndex != -1) {
              // Extract integer part
              for (int i = 3; i < decimalPointIndex; i++) {
                  if (serialData[i] >= '0' && serialData[i] <= '9') {
                      distance = distance * 10.0 + (serialData[i] - '0');
                  }
              }
              // Extract fractional part
              float fractionalPart = 0.0;
              float decimalPlace = 0.1;
              for (int i = decimalPointIndex + 1; i < 10; i++) {
                   if (serialData[i] >= '0' && serialData[i] <= '9') {
                      fractionalPart += (serialData[i] - '0') * decimalPlace;
                      decimalPlace *= 0.1;
                   }
              }
              distance += fractionalPart;

              // Print the distance to the Serial Monitor
              // This output will only be visible if the sensor is disconnected
              // from pins 0 and 1 after the code starts running.
              Serial.print("Distance = ");
              Serial.print(distance, 3); // Print with 3 decimal places
              Serial.println(" M");

          } else {
              // Note: You will likely not see this message in the Serial Monitor
              // Serial.print("Invalid data format (no decimal point): ");
              // for(int i=0; i<11; i++) Serial.print(serialData[i], HEX);
              // Serial.println();
          }
        }
      } else {
        // Note: You will likely not see this message in the Serial Monitor
        // Serial.print("Checksum error! Calculated: ");
        // Serial.print(calculatedChecksum, HEX);
        // Serial.print(", Received: ");
        // Serial.print(serialData[10], HEX);
        // Serial.print(". Message: ");
        //  for(int i=0; i<11; i++) Serial.print(serialData[i], HEX);
        // Serial.println();
      }
    } else {
      // Note: You will likely not see this message in the Serial Monitor
      // Serial.print("Incomplete message received: ");
      // for(int i=0; i<bytesRead; i++) Serial.print(serialData[i], HEX);
      // Serial.println();
    }
  }

  // A small delay is still included, but the sensor will stream data independently
  // of this loop speed once in continuous mode.
  delay(1);
}

// Helper function to calculate checksum (same logic as in Python)
// Note: This function is not used for sending commands based on the
// documented examples, but is included for potential future use
// or verification if needed.
unsigned char calculateChecksum(const unsigned char* data, size_t len) {
    unsigned char checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    checksum = (~checksum + 1);
    return checksum;
}
