#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include "time.h"

// ======= Cấu hình WiFi & Thời gian =======
const char* ssid = "P511A";
const char* password = "passcua319b";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600; // GMT+7
const int daylightOffset_sec = 0;

// ======= Cấu hình Relay & LoRa =======
#define ss 5
#define rst 4
#define dio0 2
#define RELAY_PIN 13

// ======= Định nghĩa thời gian =======
#define SECOND 1000
#define MINUTE (60 * SECOND)

// ======= Cấu hình Thời gian Bật/Tắt =======
const int startHour = 15;
const int startMinute = 0;
const int endHour = 22;
const int endMinute = 24;
const int intervalMinutes = 30;  // Khoảng cách giữa các lần bật (30 phút)
const unsigned long relayDuration = 10 * SECOND;  // Thời gian bật relay (10 giây)

bool relayState = false;
unsigned long relayOnTime = 0;
int lastTriggerMinute = -1;  // Lưu lại phút cuối cùng relay được bật

void sendRelayState() {
    LoRa.beginPacket();
    LoRa.print(relayState ? "Relay ON" : "Relay OFF");
    LoRa.endPacket();
    Serial.print("Sent Relay State: ");
    Serial.println(relayState ? "ON" : "OFF");
}

void setup() {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH); // Tắt relay lúc khởi động

    Serial.begin(115200);
    while (!Serial);
    Serial.println("Control node");

    // Kết nối WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");

    // Cấu hình NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Khởi tạo LoRa
    LoRa.setPins(ss, rst, dio0);
    while (!LoRa.begin(433E6)) {
        Serial.println("LoRa Initializing failed!");
        delay(500);
    }
    LoRa.setSyncWord(0xA5);
    Serial.println("LoRa Initializing OK!");

    // Gửi trạng thái ban đầu khi khởi động
    sendRelayState();
}

void loop() {
    // ====== Lấy thời gian hiện tại ======
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }

    int currentHour = timeinfo.tm_hour;
    int currentMinute = timeinfo.tm_min;
    int currentTotalMinutes = currentHour * 60 + currentMinute;
    int startTotalMinutes = startHour * 60 + startMinute;
    int endTotalMinutes = endHour * 60 + endMinute;

    // ====== Điều kiện bật relay theo mốc cố định ======
    if (currentTotalMinutes >= startTotalMinutes && currentTotalMinutes <= endTotalMinutes) {
        int minutesSinceStart = currentTotalMinutes - startTotalMinutes;
        if (minutesSinceStart % intervalMinutes == 0 && lastTriggerMinute != currentMinute && !relayState) {
            digitalWrite(RELAY_PIN, LOW);  // Bật relay
            relayState = true;
            relayOnTime = millis();
            lastTriggerMinute = currentMinute;
            Serial.printf("Relay ON at %02d:%02d (Auto)\n", currentHour, currentMinute);
            sendRelayState();
        }
    }

    // ====== Tắt relay sau khi hết thời gian ======
    if (relayState && millis() - relayOnTime >= relayDuration) {
        digitalWrite(RELAY_PIN, HIGH);  // Tắt relay
        relayState = false;
        Serial.printf("Relay OFF at %02d:%02d\n", currentHour, currentMinute);
        sendRelayState();
    }

    // ====== Kiểm tra lệnh từ LoRa ======
    uint8_t buffer[8];
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        int bytesRead = LoRa.readBytes(buffer, sizeof(buffer));
        if (buffer[5] == 1) {
            if (!relayState) {
                digitalWrite(RELAY_PIN, LOW);
                relayState = true;
                relayOnTime = millis();
                Serial.println("Relay ON (LoRa)");
                sendRelayState();
            }
        } else if (buffer[5] == 0) {
            if (relayState) {
                digitalWrite(RELAY_PIN, HIGH);
                relayState = false;
                Serial.println("Relay OFF (LoRa)");
                sendRelayState();
            }
        }
    }
}
