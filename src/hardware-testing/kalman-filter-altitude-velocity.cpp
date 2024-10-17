#include <Arduino.h>
#include <Wire.h>
#include "Accelerometer.hpp"
#include "Barometer.hpp"
#include "ArduinoEigen.h"
#include "KalmanFilter.hpp"
#include <cmath>

const float TIME_STEP = 0.02;  // Seconds
float timeElapsedSincePrint = 0.0;  // Seconds
const float PROCESS_VARIANCE = 0.00761;  // m^2/s^4
const float ALTITUDE_MEASUREMENT_VARIANCE = 0.0130;  // m^2  

Accelerometer accelerometer;
float inertialAccZ;
Barometer barometer;
double altitude;

VectorXd u(1);  // Control input 
VectorXd z(1);  // Measurement
VectorXd x(2);
MatrixXd P(2,2);
KalmanFilter kalmanFilter;
MatrixXd K(2,1);

void setup() {
    Serial.begin(38400);
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    Wire.begin();
    accelerometer.init();
    barometer.init();

    // Initial state vector (z position, z velocity)
    x << 0.0, 0.0;

    // Initial covariance matrix
    P(0,0) = 0.01;
    P(0,1) = 0.0;
    P(1,0) = 0.0;
    P(1,1) = 0.01;

    // State transition matrix
    MatrixXd T(2,2);
    T(0,0) = 1.0;
    T(0,1) = TIME_STEP;
    T(1,0) = 0.0;
    T(1,1) = 1.0;

    // Control matrix
    MatrixXd G(2,1);
    G(0,0) = 0.5*pow(TIME_STEP, 2);
    G(1,0) = TIME_STEP;

    // Process noise matrix 
    MatrixXd Q(2,2);
    Q = G * G.transpose() * PROCESS_VARIANCE;

    // Observation matrix
    MatrixXd H(1,2);
    H(0,0) = 1;
    H(0,1) = 0.0;

    // Measurement noise matrix
    MatrixXd R(1,1);
    R(0,0) = ALTITUDE_MEASUREMENT_VARIANCE; 
    
    // Initialise kalman filter and kalman gain matrix
    kalmanFilter = KalmanFilter(x, P, T, G, Q, H, R);
    K = kalmanFilter.getKalmanGain();
}

void loop() {
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

    // Print some output roughly every 0.25s
    if (timeElapsedSincePrint >= 0.25) {
        Serial.print("Inertial Z Acceleration: ");
        Serial.print(u(0)*100);
        Serial.println("cm/s^2");

        Serial.print("Altitude: ");
        Serial.print(z(0)*100);
        Serial.println("cm");

        Serial.print("Kalman altitude: ");
        Serial.print(x(0)*100);
        Serial.println("cm");

        Serial.print("Kalman altitude variance: ");
        Serial.print(P(0,0)*pow(100, 2));
        Serial.println("cm^2");

        Serial.print("Kalman gain altitude: ");
        Serial.println(K(0));

        Serial.print("Kalman velocity: ");
        Serial.print(x(1)*100);
        Serial.println("cm/s");

        Serial.print("Kalman velocity variance: ");
        Serial.print(P(1,1)*pow(100, 2));
        Serial.println("cm^2/s^2");

        Serial.print("Kalman gain velocity: ");
        Serial.println(K(1));

        timeElapsedSincePrint = 0.0;
    }

    delay(TIME_STEP*1000);  // Converted time step to milliseconds
    timeElapsedSincePrint += TIME_STEP;
}
