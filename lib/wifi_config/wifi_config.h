#ifndef WiFi_config_h
#define WiFi_config_h

void init_wifi(void);
void init_wifi(const char* ssid, const char* password);
void deinit_wifi();
bool check_wifi_connect();

#endif