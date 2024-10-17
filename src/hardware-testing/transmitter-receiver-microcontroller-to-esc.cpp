#include <PulsePosition.h>
#include <vector>
#include <cmath>
#include <Arduino.h>

const int RECEIVER_INPUT_PIN = 14;  // PPM input from receiver
const int THROTTLE_OUTPUT_PIN = 1;  // Output goes to ESC
const int PWM_OUTPUT_FREQUENCY = 400;  // Hz - the frequency of the pwm signal being sent to the ESC

PulsePositionInput receiver(RISING);
int numChannels = 0;
std::vector<float> channelPulseDelays = {0, 0, 0, 0, 0, 0, 0, 0};  // Microseconds
float throttlePulseDelay = 0;
int throttleOutput = 0;

void getPulseDelays();

void setup() {
    Serial.begin(38400);
    receiver.begin(RECEIVER_INPUT_PIN);
    // Set PWM output signal frequency
    analogWriteFrequency(THROTTLE_OUTPUT_PIN, PWM_OUTPUT_FREQUENCY);
    // Set resolution to 12 bits for a value range of 0-4095
    analogWriteResolution(12);
    delay(250);
    getPulseDelays();
    // Do not finish setup until the receiver starts sending data (throttle pulse delay will be in range 1000-2000 microseconds) and throttle initial position is 5 percent or less of max value (to avoid uncontrolled motor start)
    while (throttlePulseDelay == 0 || throttlePulseDelay > 1050) {
        delay(50);
        getPulseDelays();
    }
}

void loop() {
    getPulseDelays();
    // Value of input pulse delay ranges from 1000-2000 microseconds and we want to write a value with a PWM duty cycle such that the output pulse width is also in the range 1000-2000 microseconds 
    throttleOutput = throttlePulseDelay * pow(10, -6) * PWM_OUTPUT_FREQUENCY * 4095;  // pulse delay * 1/(PWM output period) * (max duty cycle) where pulse delay has been converted to seconds
    analogWrite(THROTTLE_OUTPUT_PIN, throttleOutput);
    delay((1/PWM_OUTPUT_FREQUENCY) * pow(10, 3));  // PWM output signal period in milliseconds
}

void getPulseDelays() {
    numChannels = receiver.available();
    // Check if a new frame of PPM data has arrived
    if (numChannels > 0) {
        for (int i = 0; i < numChannels; i++) {
            channelPulseDelays[i] = receiver.read(i + 1);
        }
        throttlePulseDelay = channelPulseDelays[2];  // Channel 3
    }
}
