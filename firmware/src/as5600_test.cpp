/**
 * AS5600 + SimpleFOC sanity check
 *
 * Wiring (matches wheel_task.cpp wire0 pins):
 *   SDA -> GPIO 21
 *   SCL -> GPIO 22
 *   VCC -> 3.3 V
 *   GND -> GND
 *   DIR -> GND  (counter-clockwise positive)
 *
 * Upload with:  pio run -e as5600_test -t upload
 * Monitor with: pio device monitor -b 115200
 */

#include <Arduino.h>
#include <SimpleFOC.h>

static constexpr int kSDA = 21;
static constexpr int kSCL = 22;
static constexpr long kI2CSpeed = 400000;

MagneticSensorI2C sensor(AS5600_I2C);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== AS5600 SimpleFOC test ===");

    // I2C bus scan before handing over to SimpleFOC
    Wire.begin(kSDA, kSCL, kI2CSpeed);
    Serial.print("Scanning I2C bus... ");
    bool found = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("device at 0x%02X", addr);
            if (addr == 0x36) {
                Serial.print(" <-- AS5600");
                found = true;
            }
            Serial.println();
        }
    }
    if (!found) {
        while (true) {
            Serial.println("AS5600 NOT found! Check wiring and power.");
            delay(1000);
        }
    }

    // SimpleFOC sensor init
    sensor.init(&Wire);
    Serial.println("Sensor init done.");
    Serial.println("angle_deg,velocity_rad_s");
}

void loop() {
    sensor.update();

    float angle_deg = sensor.getAngle() * (180.0f / PI);
    float velocity  = sensor.getVelocity(); // rad/s

    Serial.printf("%.2f,%.2f\n", angle_deg, velocity);
    delay(100); // 10 Hz
}
