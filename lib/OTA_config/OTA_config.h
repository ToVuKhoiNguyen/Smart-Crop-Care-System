#include "SPIFFS.h"

/****************************************
  Define Constants
****************************************/

#define DEVICE_LABEL "ESP32"

void init_wifi(const char* ssid, const char* password);
void initSPIFFS(void);
String readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
bool initWiFi();
void setWifiManager(void);
