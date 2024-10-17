#include <Arduino.h>
#include <Wire.h>
#include "Accelerometer.hpp"
#include <vector>
#include <numeric>
#include <cmath>

int n = 0;
bool finished = false;
const float TIME_STEP = 0.1;  // Seconds

Accelerometer accelerometer;
float intertialAccZ;
std::vector<float> accsVec;

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    accelerometer.init();
}

void loop() {
    // The accelerometer must be kept stationary until the mean and variance values are displayed in the serial monitor
    if (!finished) {
        accelerometer.update();
        intertialAccZ = accelerometer.calculateInertialAccZ();
        accsVec.push_back(intertialAccZ);

        Serial.print("Inertial z acceleration: ");
        Serial.print(intertialAccZ, 6);
        Serial.println("m/s^2");

        n++;

        if (n == 600) {
            // Sample mean of altitudes
            float accsSum = 0;
            for (auto& a : accsVec) {
                accsSum += a;
            }
            float accsMean = accsSum / n;

            // Sample variance of altitudes
            float sumSquareDiff = 0;
            for (auto& a : accsVec) {
                sumSquareDiff += pow((a - accsMean), 2);
            }
            float accsVariance = sumSquareDiff / (n - 1);

            Serial.println("");
            Serial.print("Inertial z acceleration mean: ");
            Serial.print(accsMean, 6);
            Serial.println("m/s^2");

            Serial.print("Inertial z acceleration variance: ");
            Serial.print(accsVariance, 6);
            Serial.println("m^2/s^4");

            finished = true;
        }

        delay(TIME_STEP*1000);
    }
}