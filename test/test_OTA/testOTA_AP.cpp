#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "wifi_ota.h"

// Access Point credentials
const char* ssid = "ESP32_OTA_AP";
const char* password = "12345678";

AsyncWebServer server(80);

void setup()
{
  Serial.begin(115200);
//   WiFi.begin(ssid, password);
  // Start the ESP32 as an Access Point
  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  wifi_ota_begin(&server);
  server.begin();
  pinMode(2, OUTPUT);
}
void loop() 
{
  wifi_ota_loop();
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
}