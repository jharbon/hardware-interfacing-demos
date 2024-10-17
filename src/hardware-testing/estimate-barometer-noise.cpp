#include <Arduino.h>
#include <Wire.h>
#include "Barometer.hpp"
#include <vector>
#include <numeric>
#include <cmath>

int n = 0;
bool finished = false;
const float TIME_STEP = 0.1;  // Seconds

Barometer barometer;
double altitude;
std::vector<double> altitudesVec;

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    barometer.init();
}

void loop() {
    if (!finished) {
        barometer.update();
        altitude = barometer.getAltitude();
        altitudesVec.push_back(altitude);

        Serial.print("Altitude: ");
        Serial.print(altitude*100);
        Serial.println("cm");

        n++;

        if (n == 600) {
            // Sample mean of altitudes
            float altitudesSum = 0;
            for (auto& a : altitudesVec) {
                altitudesSum += a;
            }
            float altitudesMean = altitudesSum / n;

            // Sample variance of altitudes
            float sumSquareDiff = 0;
            for (auto& a : altitudesVec) {
                sumSquareDiff += pow((a - altitudesMean), 2);
            }
            float altitudesVariance = sumSquareDiff / (n - 1);

            Serial.println("");
            Serial.print("Altitude mean: ");
            Serial.print(altitudesMean*100);
            Serial.println("cm");

            Serial.print("Altitude variance: ");
            Serial.print(altitudesVariance*pow(100, 2));
            Serial.println("cm^2");

            finished = true;
        }

        delay(TIME_STEP*1000);
    }
}
