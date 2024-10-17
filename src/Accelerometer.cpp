#include "Accelerometer.hpp"
#include <Wire.h>
#include <cmath>

const int MPU6050_ADDRESS = 0x68;
const int MPU6050_POWER_MANAGEMENT_REGISTER = 0x6B;
const int MPU6050_POWER_MODE = 0x00;
const int MPU6050_CONFIG_REGISTER = 0x1A;
const int MPU6050_CONFIG_MODE = 0x05;  // Low-pass filter selected with cutoff frequency of 10Hz to avoid interference from motor vibrations

const int ACCELEROMETER_CONFIG_REGISTER = 0x1C;
const int ACCELEROMETER_CONFIG_MODE = 0x10;  // Set full scale range to +/- 8g and sensitivity scale factor to 4096 LSB/g
const int SENSITIVITY_SCALE_FACTOR = 4096;
const int ACCELEROMETER_MEASUREMENTS_FIRST_REGISTER = 0x3B;

// Manually set calibration values here based on observations of the MPU6050 on a horizontal surface
const float ACC_X_CALIBRATION = 0.05;
const float ACC_Y_CALIBRATION = 0.0;
const float ACC_Z_CALIBRATION = 0.01;

const float GRAVITATIONAL_ACC = 9.81;  // m/s^2

void Accelerometer::init() {
    startPowerMode();
    configSignals();
    accX = accY = 0.0;
    accZ = 1.0;
    roll = pitch = 0.0;
}

void Accelerometer::startPowerMode() {
    // Start the MPU6050 in power mode
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_POWER_MANAGEMENT_REGISTER);
    Wire.write(MPU6050_POWER_MODE);
    Wire.endTransmission();
    delay(250);
}

void Accelerometer::configSignals() {
    // Turn on low-pass filter
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_CONFIG_REGISTER); 
    Wire.write(MPU6050_CONFIG_MODE); 
    Wire.endTransmission();

    // Set full scale range and sensitivity scale factor
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(ACCELEROMETER_CONFIG_REGISTER); 
    Wire.write(ACCELEROMETER_CONFIG_MODE);  
    Wire.endTransmission();
}

void Accelerometer::update() {
    // Select the first register we will use for pulling accelerometer measurements from the MPU6050
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(ACCELEROMETER_MEASUREMENTS_FIRST_REGISTER);
    Wire.endTransmission();

    // Measure the accelerations and calculate the new roll and pitch angles
    updateAccs();
    updateRollPitch();
}

void Accelerometer::updateAccs() {
    Wire.requestFrom(MPU6050_ADDRESS, 6); // Get data from the 6 registers corresponding to the accelerometer measurements along the x, y and z axes
    // Read data twice for each of x, y and z since each full measurement along a given axis is spread across two registers
    // For each measurement shift the first of two bytes 8 bits to the left and add the second byte to the right with | 
    int16_t lsbX = Wire.read() << 8 | Wire.read();
    int16_t lsbY = Wire.read() << 8 | Wire.read();
    int16_t lsbZ = Wire.read() << 8 | Wire.read();

    // Convert from LSB to g and calibrate
    accX = (float)lsbX/SENSITIVITY_SCALE_FACTOR - ACC_X_CALIBRATION;
    accY = (float)lsbY/SENSITIVITY_SCALE_FACTOR - ACC_Y_CALIBRATION;
    accZ = (float)lsbZ/SENSITIVITY_SCALE_FACTOR - ACC_Z_CALIBRATION;
}

void Accelerometer::updateRollPitch() {
    // Calculate roll and pitch angles and also convert from rads to degrees
    roll = atan(accY / sqrt(pow(accX,2)+pow(accZ,2))) * 180/M_PI;
    pitch = atan(-accX / sqrt(pow(accY,2)+pow(accZ,2))) * 180/M_PI;
}

float Accelerometer::calculateInertialAccZ() {
    // Sum the components of the x, y and z accelerations in the 'inertial z' direction which gravity acts along and subtract 1g to account for gravity
    float inertialAccZ = accX*sin(-pitch*DEG_TO_RAD) + accY*sin(roll*DEG_TO_RAD)*cos(pitch*DEG_TO_RAD) + accZ*cos(roll*DEG_TO_RAD)*cos(pitch*DEG_TO_RAD) - 1;
    inertialAccZ *= GRAVITATIONAL_ACC;  // Convert units: g -> m/s^2
    return inertialAccZ;
}

float Accelerometer::getAccX() {
    return accX;
}

float Accelerometer::getAccY() {
    return accY;
}

float Accelerometer::getAccZ() {
    return accZ;
}

float Accelerometer::getRoll() {
    return roll;
}

float Accelerometer::getPitch() {
    return pitch;
}
