#ifndef _WIFI_OTA_H_
#define _WIFI_OTA_H_

#include <ESPAsyncWebServer.h>
#include <Update.h>

typedef void (*wifiota_callback_t)(uint32_t curen, uint32_t totol);

void wifi_ota_set_begin_callback(wifiota_callback_t c);
void wifi_ota_set_proces_callback(wifiota_callback_t c);
void wifi_ota_set_end_callback(wifiota_callback_t c);
void ota_set_debug(bool mode);

void UpdateRun();

void init_access_point(void);

void wifi_ota_begin(AsyncWebServer *s);
void wifi_ota_loop();

#endif
