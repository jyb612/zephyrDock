#include <Wire.h>

// I2C addresses of the GY-US42v2 sensors
#define SENSOR_1_ADDRESS 0x32  // First sensor address
#define SENSOR_2_ADDRESS 0x34  // Second sensor address

// Define the I2C pins for ESP32 (you can change these if needed)
#define I2C_SDA 21  // Default SDA pin for ESP32
#define I2C_SCL 22  // Default SCL pin for ESP32

// Function to read distance from the sensor
int readDistance(byte address) {
  Wire.beginTransmission(address);
  Wire.write(0x51); // Command to start measurement
  Wire.endTransmission();

  delay(70); // Wait for the measurement to complete

  Wire.requestFrom(address, 2); // Request 2 bytes of data
  if (Wire.available() == 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    int distance = (highByte << 8) | lowByte; // Combine bytes to get distance in cm
    return distance;
  } else {
    return -1; // Return -1 if reading fails
  }
}

void setup() {
  Serial.begin(115200); // ESP32 uses 115200 baud rate by default
  Wire.begin(I2C_SDA, I2C_SCL); // Initialize I2C with custom pins

  Serial.println("GY-US42v2 Dual Ultrasonic Sensor Test");
  Serial.println("Reading distances from both sensors...");
}

void loop() {
  // Read distance from the first sensor (0x32)
  int distance1 = readDistance(SENSOR_1_ADDRESS);
  if (distance1 != -1) {
    Serial.print("Distance at 0x32: ");
    Serial.print(distance1);
    Serial.println(" cm");
  } else {
    Serial.println("Failed to read distance from sensor at 0x32!");
  }

  // Read distance from the second sensor (0x34)
  int distance2 = readDistance(SENSOR_2_ADDRESS);
  if (distance2 != -1) {
    Serial.print("Distance at 0x34: ");
    Serial.print(distance2);
    Serial.println(" cm");
  } else {
    Serial.println("Failed to read distance from sensor at 0x34!");
  }

  delay(1000); // Wait 1 second before the next reading
}