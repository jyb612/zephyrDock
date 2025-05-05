#include <Wire.h>

#define I2C_SDA 21  // Default SDA pin for ESP32
#define I2C_SCL 22  // Default SCL pin for ESP32

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("I2C Address Scan");
  Serial.println("Scanning...");
}

void loop() {
  byte error, address;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  delay(5000); // Wait 5 seconds before scanning again
}