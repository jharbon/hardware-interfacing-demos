#include <PulsePosition.h>
#include <vector>
#include <Arduino.h>

PulsePositionInput receiver(RISING);
const int RECEIVER_INPUT_PIN = 14;
int numChannels = 0;
std::vector<float> channelPulseDelays = {0, 0, 0, 0, 0, 0, 0, 0};  // Microseconds

void setup() {
    Serial.begin(38400);
    receiver.begin(RECEIVER_INPUT_PIN);
    delay(250);
}

void loop() {
    numChannels = receiver.available();
    // Check if a new frame of PPM data has arrived
    if (numChannels > 0) {
        for (int i = 0; i < numChannels; i++) {
            channelPulseDelays[i] = receiver.read(i + 1);
        }

        Serial.println("Pulse delays:");

        Serial.print("Roll: ");
        Serial.print(channelPulseDelays[0]/1000);  // Channel 1
        Serial.println("ms");

        Serial.print("Pitch: ");
        Serial.print(channelPulseDelays[1]/1000);  // Channel 2
        Serial.println("ms");

        Serial.print("Throttle: ");
        Serial.print(channelPulseDelays[2]/1000);  // Channel 3
        Serial.println("ms");

        Serial.print("Yaw: ");
        Serial.print(channelPulseDelays[3]/1000);  // Channel 4
        Serial.println("ms");
    }

    delay(200);  // To make the serial output easier to read
}
