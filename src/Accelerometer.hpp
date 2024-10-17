#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <chrono>

class Accelerometer
{
  private: 
  float accX, accY, accZ;  // g's
  float roll, pitch;  // Degrees

  void startPowerMode();
  void configSignals();
  void updateAccs();
  void updateRollPitch();

  public:
  Accelerometer() {};
  void init();
  void update();
  float calculateInertialAccZ();
  float getAccX();
  float getAccY();
  float getAccZ();
  float getRoll();
  float getPitch();
};

#endif