#include <Wire.h>
#include <Arduino.h>
#include "sensor_config.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "wifi_ota.h"

// Access Point credentials
const char *ssid = "ESP32_OTA_AP";
const char *password = "12345678";
bool OTA = false;

const int buttonPin = 13;
const int ledPin = 2;

AsyncWebServer server(80);

// Interrupt service routine (ISR)
void IRAM_ATTR buttonISR()
{
    OTA = true;
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);

    // Initialize the button pin as an input with pull-up resistor
    pinMode(buttonPin, INPUT_PULLUP);
    // Attach the interrupt to the button pin
    attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

    Serial.println("SHT20 Test");
}

void loop()
{
    if (OTA)
    {
        // Start the ESP32 as an Access Point
        WiFi.softAP(ssid, password);

        Serial.println("Access Point Started");
        Serial.print("IP Address: ");
        Serial.println(WiFi.softAPIP());

        wifi_ota_begin(&server);
        server.begin();

        digitalWrite(ledPin, HIGH);
        OTA = false;
    }
    wifi_ota_loop();

    float temperature = get_temperature();
    float humidity = get_humidity();

    if (temperature != -999 && humidity != -999)
    {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");

        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
    }
    else
    {
        Serial.println("Failed to read from SHT20");
    }

    delay(2000);
}
