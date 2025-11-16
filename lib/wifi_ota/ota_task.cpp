#include "ota_task.h"
#include "wifi_ota.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server_ota(80);

void OTA_Init()
{
    wifi_ota_begin(&server_ota);
    server_ota.begin();
    // Serial.println("OTA_Init");
    xTaskCreatePinnedToCore(OTA_Task, "OTA_Task", 5 * 1024, NULL, 1, NULL, 1);
    // Serial.println("OTA_Init");
    // OK
}

void OTA_Task(void *parameter)
{
    wifi_ota_loop();
    // vTaskDelay(pdMS_TO_TICKS(10));
}