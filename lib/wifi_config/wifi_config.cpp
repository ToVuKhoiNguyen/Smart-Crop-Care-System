#include "wifi_config.h"
#include "WiFi.h"

// Replace with your network credentials
const char *ssid = "512BB";     // Network SSID (name)
const char *password = "1123581321"; // Network password

void init_wifi(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Waiting for WiFi Connection ..............");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("");
    Serial.println("WiFi Connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void init_wifi(const char* ssid, const char* password)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Waiting for WiFi Connection ..............");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("");
    Serial.println("WiFi Connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void deinit_wifi()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi disconnected and turned off");
}

bool check_wifi_connect()
{
    return WiFi.status() == WL_CONNECTED;
}
