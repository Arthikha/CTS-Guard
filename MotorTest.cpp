#include <Arduino.h>

#define MOTOR_PIN 4
// #define BUTTON_PIN 23

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  // pinMode(BUTTON_PIN, INPUT_PULLUP); // button pressed = LOW
}

void loop() {
  // if (digitalRead(BUTTON_PIN) == LOW) { // button pressed
    digitalWrite(MOTOR_PIN, HIGH);
    Serial.println("Motor ON");
    delay(2000);
  // } else {
    digitalWrite(MOTOR_PIN, LOW);
    Serial.println("Motor OFF");
    delay(2000);
  // }
}
