#include <Arduino.h>
#include <cmath>

const int VOLTAGE_INPUT_PIN = 15;
const float VOLTAGE_REDUCTION_FACTOR = 220.0/(220.0 + 1000.0);
const int RED_LED_PIN = 5;
const int GREEN_LED_PIN = 6;

float inputVoltage = 0;
float actualVoltage = 0;

void setup() {
    Serial.begin(38400);
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(RED_LED_PIN, LOW);
    pinMode(GREEN_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, LOW);
}

void loop() {
    inputVoltage = (static_cast<float>(analogRead(VOLTAGE_INPUT_PIN)) / 1023) * 3.3;  // First convert from 10 bit read value to a voltage in the pin input range 0-3.3V 
    actualVoltage = inputVoltage / VOLTAGE_REDUCTION_FACTOR;  // Convert from the divided voltage (we are receiving the smaller of the two) to the actual undivided voltage 
    
    Serial.print("Input voltage: ");
    Serial.print(inputVoltage);
    Serial.println("V");
    Serial.print("Actual voltage: ");
    Serial.print(actualVoltage);
    Serial.println("V");

    if (actualVoltage >= 3.0 && actualVoltage <= 3.6) {
        // The red LED will turn ON and the green LED will turn OFF if the voltage source to the divider is the 3.3v microcontroller output 
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(GREEN_LED_PIN, LOW);
    }
    else if (actualVoltage >= 5.0 && actualVoltage <= 5.6) {
        // The red LED will turn OFF and the green LED will turn ON if the voltage source to the divider is the 5v microcontroller output 
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH);
    }

    delay(500);
}