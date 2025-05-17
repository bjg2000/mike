#include <Wire.h>
#include <Arduino.h>

// MPU6050 I2C address
const int MPU6050_ADDR = 0x68;

// Registers
const int ACCEL_XOUT_H = 0x3B;
const int PWR_MGMT_1 = 0x6B;
const int GYRO_XOUT_H = 0x43;

// Sensitivity scale factors (based on default MPU6050 configuration: Accel +/-2g, Gyro +/-250 deg/s)
// These values might need adjustment if your MPU6050 is configured differently.
const float ACCEL_SCALE_FACTOR = 16384.0; // LSB/g for +/-2g range
const float GYRO_SCALE_FACTOR = 131.0;   // LSB/deg/s for +/-250 deg/s range

// Acceleration due to gravity
const float GRAVITY_MS2 = 9.80665; // m/s^2

void setup() {
  // Initialize serial communication
  Serial.begin(115200); // Increased baud rate for faster output

  // Initialize I2C communication
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // Set to 0 to wake up the sensor
  Wire.endTransmission();

  Serial.println("MPU6050 Data (Physical Units)");
  Serial.println("-------------------------------------------------------------------------------------------------------------------");
  Serial.println("AccelX (m/s^2)\tAccelY (m/s^2)\tAccelZ (m/s^2)\tGyroX (deg/s)\tGyroY (deg/s)\tGyroZ (deg/s)"); // Units added to header
  Serial.println("-------------------------------------------------------------------------------------------------------------------");
}

void loop() {
  // Request accelerometer and gyroscope data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H); // Starting register for accelerometer data
  Wire.endTransmission(false); // Keep connection open for reading
  Wire.requestFrom(MPU6050_ADDR, 14, true); // Request 14 bytes (Accel, Temp, Gyro)

  // Read raw accelerometer data (16-bit signed integers)
  int16_t rawAccelX = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelY = (Wire.read() << 8) | Wire.read();
  int16_t rawAccelZ = (Wire.read() << 8) | Wire.read();

  // Skip temperature data (2 bytes)
  Wire.read();
  Wire.read();

  // Read raw gyroscope data (16-bit signed integers)
  int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
  int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();

  // Convert raw accelerometer data to m/s^2
  float accelX_ms2 = (float)rawAccelX / ACCEL_SCALE_FACTOR * GRAVITY_MS2;
  float accelY_ms2 = (float)rawAccelY / ACCEL_SCALE_FACTOR * GRAVITY_MS2;
  float accelZ_ms2 = (float)rawAccelZ / ACCEL_SCALE_FACTOR * GRAVITY_MS2;

  // Convert raw gyroscope data to deg/s
  float gyroX_degs = (float)rawGyroX / GYRO_SCALE_FACTOR;
  float gyroY_degs = (float)rawGyroY / GYRO_SCALE_FACTOR;
  float gyroZ_degs = (float)rawGyroZ / GYRO_SCALE_FACTOR;

  // Print converted data
  Serial.print(accelX_ms2, 2); // Print with 2 decimal places
  Serial.print("\t");
  Serial.print(accelY_ms2, 2);
  Serial.print("\t");
  Serial.print(accelZ_ms2, 2);
  Serial.print("\t");
  Serial.print(gyroX_degs, 2);
  Serial.print("\t");
  Serial.print(gyroY_degs, 2);
  Serial.print("\t");
  Serial.println(gyroZ_degs, 2);

  // Reduced delay for faster data retrieval
  delay(10); // Reduced delay from 50ms to 10ms (adjust as needed)
}
