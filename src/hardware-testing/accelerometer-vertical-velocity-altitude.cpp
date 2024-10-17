#include <Arduino.h>
#include <Wire.h>
#include "Accelerometer.hpp"
#include <cmath>

const float TIME_STEP = 0.1;  // s

Accelerometer accelerometer;
float inertialAccZ;  // m/s^2
float velocityZ;  // m/s
float altitude;  // m

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    accelerometer.init();
    inertialAccZ = 0.0;
    velocityZ = 0.0;
    altitude = 0.0;
}

void loop() {
    // Update altitude first using the inertial z acceleration and z velocity from the previous iteration
    altitude += velocityZ*TIME_STEP + 0.5*inertialAccZ*pow(TIME_STEP, 2);
    // Update z velocity using the inertial z acceleration from the previous iteration
    velocityZ += inertialAccZ*TIME_STEP;
    // Update accelerometer measurements and get the new inertial z acceleration
    accelerometer.update();
    inertialAccZ = accelerometer.calculateInertialAccZ();

    Serial.print("Velocity Z: ");
    Serial.print(velocityZ*100);
    Serial.println("cm/s");

    Serial.print("Altitude: ");
    Serial.print(altitude*100);
    Serial.println("cm");

    delay(TIME_STEP*1000);
}