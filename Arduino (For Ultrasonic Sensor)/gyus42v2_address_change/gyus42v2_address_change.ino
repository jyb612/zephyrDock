#include <Wire.h>

#define OLD_ADDRESS 0x32   // Current I2C address
#define NEW_ADDRESS 0x68   // New I2C address

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C communication

  // Change the address of the sensor
  changeAddress(OLD_ADDRESS, NEW_ADDRESS);
}

void loop() {
  // Your sensor reading code here
}

void changeAddress(byte oldAddress, byte newAddress) {
  Wire.beginTransmission(oldAddress);
  Wire.write(0xAA); // Unlock command to change address
  Wire.write(0xA5); // Unlock command
  Wire.write(newAddress); // New I2C address
  Wire.endTransmission();
  delay(100); // Wait for the change to take effect
  Serial.println("Address Changed!");
}
