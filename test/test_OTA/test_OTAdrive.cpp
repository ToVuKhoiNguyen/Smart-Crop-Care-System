#include <Arduino.h>
#include <OTA_config.h>
#include <otadrive_esp.h>

#define APIKEY "c12e60bd-4c62-44d1-8d72-3f0f5ce2aaa1"

void setup(void) {
    Serial.begin(115200);
    OTADRIVE.setInfo(APIKEY, "V1.0.6");
    initSPIFFS();

    // Initialize WiFi
    if(initWiFi()) {
        // Request for firmware
        OTADRIVE.updateFirmware();
    }
    else {
        setWifiManager();
    }

    pinMode(25, OUTPUT);
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);
}

void loop(void) {
    // Request for firmware update every 10 seconds
    // if(WiFi.status() == WL_CONNECTED) {
    //     if(OTADRIVE.timeTick(10))
    //         OTADRIVE.updateFirmware();
    // }

    digitalWrite(25, HIGH);
    delay(1000);
    digitalWrite(32, HIGH);
    delay(1000);
    digitalWrite(33, HIGH);
    delay(1000);
    digitalWrite(25, LOW);
    delay(250);
    digitalWrite(32, LOW);
    delay(250);
    digitalWrite(33, LOW);
    delay(250);
}