#include <Arduino.h>
#include "config.h"

void setup() {
    Serial.begin();
    Serial.println("Starting...");

    initServos();

    Serial.println("Moving to Home Position...");
    penUp();
    Homeposition();

    delay(3000);
}

void loop() {
    Serial.println("Drawing 'A,B'");
    drawChar('A');
    delay(1000);

    drawChar('B');
    delay(1000);

    Serial.println("Finished.");
    penUp();
    goHome();

    while (true) {
        delay(100);
    }
}