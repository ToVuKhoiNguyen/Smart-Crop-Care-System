#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "wifi_ota.h"

const char* ssid = "TTS_TMRD";
const char* password = "rdtts2024";

AsyncWebServer server(80);

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  wifi_ota_begin(&server);
  server.begin();
  pinMode(2, OUTPUT);
}
void loop() 
{
  wifi_ota_loop();
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  delay(1000);
}