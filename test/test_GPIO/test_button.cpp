#include "Arduino.h"

#define btn_1 13
#define btn_2 12
#define btn_3 14

void setup () {
    Serial.begin(115200);  // We initialize serial connection so that we could print values from sensor.

    pinMode(btn_1, INPUT);
    pinMode(btn_2, INPUT);
    pinMode(btn_3, INPUT);
}

void loop () {
    Serial.print("Button 1: ");
    Serial.println(digitalRead(btn_1));
    Serial.print("Button 2: ");
    Serial.println(digitalRead(btn_2));
    Serial.print("Button 3: ");
    Serial.println(digitalRead(btn_3));
    delay(500);
}