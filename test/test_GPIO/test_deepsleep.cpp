#include <Arduino.h>
#include <esp_sleep.h>
#include "sensor_config.h"

#define uS_TO_S_FACTOR 1000000  // Chuyển giây thành micro giây
#define TIME_TO_SLEEP 10        // Thời gian ngủ (giây)

float bus_voltage = 0, current = 0;

void setup() {
    Serial.begin(115200);
    init_current_sensor();

    bus_voltage = get_sourse_voltage_V();
    current = get_current_mA();

    Serial.print("Bus voltage:");
    Serial.print(bus_voltage);
    Serial.print(" V");
    Serial.print(" Current:");
    Serial.print(current);
    Serial.println(" mA");

    Serial.println("ESP32 đang vào Deep Sleep...");

    // Cấu hình thời gian ngủ
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    Serial.println("ESP32 sẽ thức dậy sau " + String(TIME_TO_SLEEP) + " giây...");
    Serial.flush();  // Đảm bảo dữ liệu được in hết trước khi ngủ

    esp_deep_sleep_start();  // Bắt đầu Deep Sleep
}

void loop() {
    // Không dùng trong Deep Sleep vì ESP sẽ reset khi thức dậy
}
