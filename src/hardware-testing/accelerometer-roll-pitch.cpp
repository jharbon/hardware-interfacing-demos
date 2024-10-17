#include <Arduino.h>
#include <Wire.h>
#include "Accelerometer.hpp"

Accelerometer accelerometer;
float accX, accY, accZ;
float roll, pitch;

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    accelerometer.init();
}

void loop() {
    accelerometer.update();
    accX = accelerometer.getAccX();
    accY = accelerometer.getAccY();
    accZ = accelerometer.getAccZ();
    roll = accelerometer.getRoll();
    pitch = accelerometer.getPitch();

    Serial.print("Acceleration X: ");
    Serial.print(accX, 4);
    Serial.println("g");

    Serial.print("Acceleration Y: ");
    Serial.print(accY, 4);
    Serial.println("g");

    Serial.print("Acceleration Z: ");
    Serial.print(accZ, 4);
    Serial.println("g");

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.println("deg");

    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.println("deg");

    delay(200);
}