#include "sensor_config.h"
#include "device_config.h"
#include "Arduino.h"

float bus_voltage = 0, current = 0, shunt_voltage = 0;
unsigned long printTime = 0, switchTime = 0;

void setup()
{
    Serial.begin(115200);
    init_current_sensor();

    // pinMode(25, OUTPUT);
    // pinMode(26, OUTPUT);
    // pinMode(27, OUTPUT);
    // pinMode(14, OUTPUT);
    // pinMode(13, OUTPUT);

    // digitalWrite(25, LOW);
    // digitalWrite(26, LOW);
    // digitalWrite(27, LOW);
    // digitalWrite(14, LOW);
    // digitalWrite(13, LOW);
}

void loop()
{
    if(millis() - printTime > 1000)
    {
        printTime = millis();
        bus_voltage = get_sourse_voltage_V();
        current = get_current_mA();

        Serial.print("Bus voltage:");
        Serial.print(bus_voltage);
        Serial.print(" V");
        Serial.print("Shunt voltage:");
        Serial.print(shunt_voltage);
        Serial.print(" V");
        Serial.print(" Current:");
        Serial.print(current);
        Serial.println(" mA");
    }

    // if(millis() - switchTime > 5000)
    // {
    //     switchTime = millis();
    //     digitalWrite(25, !digitalRead(25));
    //     digitalWrite(26, !digitalRead(26));
    //     digitalWrite(27, !digitalRead(27));
    //     digitalWrite(14, !digitalRead(14));
    //     digitalWrite(13, !digitalRead(13));
    // }



}

