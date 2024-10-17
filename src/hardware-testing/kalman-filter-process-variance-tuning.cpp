#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include "Accelerometer.hpp"
#include "Barometer.hpp"
#include "ArduinoEigen.h"
#include "KalmanFilter.hpp"
#include <cmath>

const float TIME_STEP = 0.02;  // Seconds
const std::vector<float> PROCESS_VARIANCE_VALUES = {0.00961, 0.01011, 0.01061, 0.01111, 0.01161};  // m^2/s^4
const int NUM_ITERATIONS_PER_VALUE = 1000;
const float ALTITUDE_MEASUREMENT_VARIANCE = 0.0130;  // m^2  

float altitudeAbsDiffNormSum;  // Sum of absolute((measured - actual)/actual)
float altitudeMeanAbsPercentageError;  // (100 * absDiffNormSum)/NUM_ITERATIONS_PER_VALUE
float altitudeActualValue = 0.15;  // MPU6050 + BMP280 need to remain stationary from measurements start until finish
float velocityAbsDiffNormSum;  
float velocityMeanAbsPercentageError; 
float velocityActualValue = 0.001;  // MPU6050 + BMP280 need to remain stationary from measurements start until finish
bool finished  = false;

Accelerometer accelerometer;
float inertialAccZ;
Barometer barometer;
double altitude;

VectorXd u(1);  // Control input 
VectorXd z(1);  // Measurement
VectorXd x(2);
MatrixXd P(2,2);
MatrixXd T(2,2);
MatrixXd G(2,1);
MatrixXd Q(2,2);  // Declare process variance matrix here and define it for each process variance value during the loop
MatrixXd H(1,2);
MatrixXd R(1,1);
KalmanFilter kalmanFilter;
MatrixXd K(2,1);

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    accelerometer.init();
    barometer.init();
    delay(250);

    // Initial state vector (z position, z velocity)
    x << 0.0, 0.0;

    // Initial covariance matrix
    P(0,0) = 0.01;
    P(0,1) = 0.0;
    P(1,0) = 0.0;
    P(1,1) = 0.01;

    // State transition matrix
    T(0,0) = 1.0;
    T(0,1) = TIME_STEP;
    T(1,0) = 0.0;
    T(1,1) = 1.0;

    // Control matrix
    G(0,0) = 0.5*pow(TIME_STEP, 2);
    G(1,0) = TIME_STEP;

    // Observation matrix
    H(0,0) = 1;
    H(0,1) = 0.0;

    // Measurement noise matrix
    R(0,0) = ALTITUDE_MEASUREMENT_VARIANCE; 

    altitudeMeanAbsPercentageError = 0.0;
    velocityMeanAbsPercentageError = 0.0;

    // Add an extra delay of 10s to provide enough time to move the MPU6050 + BMP280 to the specified altitude before we start recording differences
    delay(10000);
}

void loop() {
    if (!finished) {
        for (float proVar : PROCESS_VARIANCE_VALUES) {
            Q = G * G.transpose() * proVar;
            // Initialise kalman filter and kalman gain matrix
            kalmanFilter = KalmanFilter(x, P, T, G, Q, H, R);
            K = kalmanFilter.getKalmanGain();

            altitudeAbsDiffNormSum = 0;
            velocityAbsDiffNormSum = 0;

            for (int i = 0; i < NUM_ITERATIONS_PER_VALUE; i++) {
                accelerometer.update();
                inertialAccZ = accelerometer.calculateInertialAccZ();
                u(0) = inertialAccZ;

                barometer.update();
                altitude = barometer.getAltitude();
                z(0) = altitude;

                kalmanFilter.updateStateEstimate(u, z);
                x = kalmanFilter.getState();
                P = kalmanFilter.getCovariance();
                K = kalmanFilter.getKalmanGain();

                altitudeAbsDiffNormSum += abs((x(0) - altitudeActualValue)/altitudeActualValue);
                velocityAbsDiffNormSum += abs((x(1) - velocityActualValue)/velocityActualValue);

                delay(TIME_STEP*1000);  // Converted time step to milliseconds
            }

            altitudeMeanAbsPercentageError = (100 * altitudeAbsDiffNormSum)/NUM_ITERATIONS_PER_VALUE;
            velocityMeanAbsPercentageError = (100 * velocityAbsDiffNormSum)/NUM_ITERATIONS_PER_VALUE;

            Serial.print("Process variance = ");
            Serial.println(proVar, 6);
            Serial.print("Altitude mean absolute percentage error = ");
            Serial.print(altitudeMeanAbsPercentageError);
            Serial.print("%, Altitude variance = ");
            Serial.print(P(0,0)*pow(100, 2));
            Serial.print("cm^2, Altitude Kalman gain: ");
            Serial.println(K(0));
            Serial.print("Velocity mean absolute percentage error = ");
            Serial.print(velocityMeanAbsPercentageError);
            Serial.print("%, Velocity variance = ");
            Serial.print(P(1,1)*pow(100, 2));
            Serial.print("cm^2/s^2, Velocity Kalman gain = ");
            Serial.println(K(1));
        }

        finished = true;
    }
}