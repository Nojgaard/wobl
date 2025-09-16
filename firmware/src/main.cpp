#include <Arduino.h>
#include <Wire.h>
#include "imu.hpp"

IMU imu;

#define S_SCL 22
#define S_SDA 21

/*void setup()
{
  // unsigned long baud = 1000000;
  unsigned long baud = 115200;
  Serial.begin(baud);

  Wire.begin(S_SDA, S_SCL);
  Wire.setClock(100000);

  Serial.println("Initializing IMU...");
  bool initialized = false;
  while (!initialized)
  {
    initialized = imu.initialize();

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(imu.icm_.statusString());
    if (imu.icm_.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(2000);
    }
    else
    {
      initialized = true;
    }
  }
}*/
#include <Arduino.h>
#include <Wire.h>
#include "imu.hpp"

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // I have also tried with Wire.begin(26, 27) according to the schematic
  Serial.println("Scanning I2C bus...");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("No I2C devices found!");
}

void loop(){}