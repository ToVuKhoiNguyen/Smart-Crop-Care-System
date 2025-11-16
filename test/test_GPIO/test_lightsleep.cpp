#include <Arduino.h>
#include <esp_sleep.h>

#define SLEEP_DURATION 10  // Thời gian ngủ (giây)

void setup() {
    Serial.begin(115200);
    delay(1000);  

    Serial.println("ESP32 vào chế độ Light Sleep...");

    // Cấu hình RTC Timer để đánh thức ESP32
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000);  

    Serial.println("Bắt đầu Light Sleep...");
    esp_light_sleep_start();  // Bắt đầu Light Sleep

    Serial.println("ESP32 đã thức dậy!");
}

void loop() {
    Serial.println("Chạy tiếp sau Light Sleep...");
    delay(3000);
}
