/* Define MQTT host */
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "0ae424b1-ebd6-497c-9680-d19428d57778"

#include "sensor_config.h"
#include "device_config.h"
#include <esp_task_wdt.h>
#include <ERa.hpp>
#include "ERa_config.cpp"
#include "BLE_config.cpp"

unsigned long main_dashboard_time = 0;
extern tft_screen_select_t tft_screen_select;
volatile bool BLE_finished = false;

// Khai báo các handle cho Task
TaskHandle_t HandleEraTask;
TaskHandle_t HandleLoRaReadTask;
TaskHandle_t HandleLoRaProcessTask;

// task cho ERa
void EraTask(void *pvParameters)
{
    // Serial.println(xPortGetCoreID());
    /* Setup Client for Modbus TCP/IP */
    ERa.setModbusClient(mbTcpClient);
    /* Set scan WiFi. If activated, the board will scan
    and connect to the best quality WiFi. */
    ERa.setScanWiFi(true);
    /* Initializing the ERa library. */
    ERa.begin(ssid, pass);
    Serial.println("ERa initialization done.");
    for (;;)
    {
        ERa.run();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// task cho LoRa
void LoRaReadTask(void *pvParameters)
{
    for(;;)
    {
        loraReadPackage();
        vTaskDelay(pdMS_TO_TICKS(100)); // Để nhả CPU cho các task khác
    }
}

void LoRaProcessTask(void *pvParameters)
{
    for(;;)
    {
        loraProcessPackage();
        vTaskDelay(pdMS_TO_TICKS(100)); // Để nhả CPU cho các task khác
    }
}

void BLEInitTask(void* pvParameters) {
    initBLE();     // gọi BLEDevice::init(...) tại đây
    vTaskDelete(NULL);
}

void setup()
{
    esp_task_wdt_init(5, true);
    Serial.begin(115200);

    // I2C device
    init_RTC_sensor();
    init_RTC_alarm();
    check_alarm();
    init_current_sensor();

    // SPI device
    set_SPI_CS_pin();
    init_TFT_screen();
    init_sd_card();
    loraInit();
    loraConfig(433E6, 14, 7, 125E3, 0xA5, 5); // Cấu hình LoRa: tần số 433MHz, công suất phát 17dBm, SF7, BW125kHz, sync word 0xA5, CR5 (4/5)
    // loraConfig(433E6, 17, 12, 62.5E3, 0xA5, 8);

    // TFT screen
    draw_main_dashboard();
    update_main_dashboard_communicate();
    update_main_dashboard_time();
    update_main_dashboard_sensor();

    // RTC - DS3231
    // check_alarm();
    calculatePumpTime();

    // /* Setup Client for Modbus TCP/IP */
    // ERa.setModbusClient(mbTcpClient);
    // /* Set scan WiFi. If activated, the board will scan
    // and connect to the best quality WiFi. */
    // ERa.setScanWiFi(true);
    // /* Initializing the ERa library. */
    // ERa.begin(ssid, pass);
    // Serial.println("ERa initialization done.");

    createQueue();

    initBLE();
    while(1)
    {
        if(BLE_finished)
        {
            Serial.print("BLE finished");
            break;
        }
    }
    // xTaskCreatePinnedToCore(
    //     BLEInitTask,
    //     "BLEInitTask",
    //     8192,
    //     NULL,
    //     1,
    //     NULL,
    //     1 // chạy trên Core 1
    // );

    xTaskCreatePinnedToCore(
        EraTask,        // Hàm xử lý
        "EraTask",      // Tên Task
        10000,          // Stack size
        NULL,           // Tham số truyền vào
        1,              // Mức ưu tiên (1 = thấp)
        &HandleEraTask, // Handle
        0               // Chạy trên core 1
    );

    xTaskCreatePinnedToCore(
        LoRaReadTask,        // Hàm xử lý
        "LoRaReadTask",      // Tên Task
        4096,          // Stack size
        NULL,           // Tham số truyền vào
        1,              // Mức ưu tiên (1 = thấp)
        &HandleLoRaReadTask, // Handle
        0               // Chạy trên core 0
    );

    xTaskCreatePinnedToCore(
        LoRaProcessTask,        // Hàm xử lý
        "LoRaProcessTask",      // Tên Task
        4096,          // Stack size
        NULL,           // Tham số truyền vào
        3,              // Mức ưu tiên (1 = thấp)
        &HandleLoRaProcessTask, // Handle
        0               // Chạy trên core 1
    );

    sendNodePacket(CONFIG_REQ, mac_sens_node[0]);
    sendNodePacket(CONFIG_REQ, mac_sens_node[1]);
}

void loop()
{   
    // ERa.run();
    check_touch_screen();
    check_alarm_flag();
    switch (tft_screen_select)
    {
    case MAIN_DASHBOARD:
        if (millis() - main_dashboard_time >= MAIN_DASHBOARD_TIME_UPDATE)
        {
            main_dashboard_time = millis();
            update_main_dashboard_time();
            update_main_dashboard_sensor();
        }
        break;

    default:
        break;
    }
}
