#define SENSOR_NODE

#include "device_config.h"
#include "sensor_config.h"
#include "Wire.h"
#include <LoRa.h>
#include <esp_sleep.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int pkgRSSI = 0;
float pkgSNR = 0.0;

#define uS_TO_S_FACTOR 1000000  // Chuyển giây thành micro giây
#define TIME_TO_SLEEP 1800        // Thời gian ngủ (giây)
#define SENSOR_NODE_ID 1
#define PACKET_TIMEOUT 1000     // thời gian chờ giữa các packet

QueueHandle_t loraPackage;
unsigned long packetTime = 0;    // thời gian hiện tại của packet
bool finishDecodePkg = false;     // xử lý hết packet rồi mới vào loop

TaskHandle_t HandleLoRaProcessTask;
TaskHandle_t HandleLoRaReceiveTask;

void LoRaProcessTask(void *pvParameters)
{
    for (;;)
    {
        int numPackets = uxQueueMessagesWaiting(loraPackage);

        if (numPackets > 0)
        {
            // Nếu đã đủ thời gian không nhận thêm packet, thì bắt đầu xử lý batch
            if (millis() - packetTime > PACKET_TIMEOUT)
            {
                Serial.printf("🧮 Start decoding batch (%d packets)\n", numPackets);
                
                while (uxQueueMessagesWaiting(loraPackage) > 0)
                {
                    Packet pkt;
                    if (xQueueReceive(loraPackage, &pkt, 0) == pdTRUE)
                    {
                        decodeSensorPacket(pkt.data, pkt.len, mac_sens_node[SENSOR_NODE_ID - 1]);
                    }
                }
                Serial.println("✅ Finished decoding batch");
                finishDecodePkg = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Giảm tần suất kiểm tra
    }
}

void LoRaReceiveTask(void *pvParameters)
{
    for (;;)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            pkgRSSI = LoRa.packetRssi();
            pkgSNR = LoRa.packetSnr();
            Packet pkt;
            int bytesRead = 0;
            while (LoRa.available() && bytesRead < packetSize)
            {
                pkt.data[bytesRead++] = LoRa.read();
            }
            pkt.len = bytesRead;

            Serial.print("📦 Received packet: ");
            for (int i = 0; i < pkt.len; i++)
            {
                Serial.printf("%02X ", pkt.data[i]);
            }
            Serial.println();

            packetTime = millis(); // cập nhật thời gian nhận mới nhất

            if (xQueueSend(loraPackage, &pkt, 0) != pdPASS)
            {
                Serial.println("⚠️ Queue full, packet dropped");
            }
            else
            {
                Serial.println("📥 Packet pushed to queue");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // để tránh chiếm CPU
    }
}

void deepSleepStart()
{
    Serial.println("ESP32 đang vào Deep Sleep...");
    display.clearDisplay();
    display.display();  // clear buffer

    // Tắt hiển thị và OLED controller
    display.ssd1306_command(SSD1306_DISPLAYOFF); 
    Wire.end(); // Ngắt I2C nếu không dùng nữa

    // Cấu hình thời gian ngủ
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    Serial.println("ESP32 sẽ thức dậy sau " + String(TIME_TO_SLEEP) + " giây...");
    Serial.flush();  // Đảm bảo dữ liệu được in hết trước khi ngủ
    esp_deep_sleep_start();  // Bắt đầu Deep Sleep
}

unsigned long lastSendTime = 0, lastReadTime = 0;;
static bool isConfiged = false;
const unsigned long sendInterval = 5000, readInterval = 200;
uint8_t total_send_count = 0, count = 0;
extern bool ackReceived;

void setup()
{
    setCpuFrequencyMhz(80);
    pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(13, OUTPUT);

    // digitalWrite(25, LOW);
    // digitalWrite(26, LOW);
    // digitalWrite(27, LOW);
    // digitalWrite(14, LOW);
    // digitalWrite(13, LOW);

    digitalWrite(25, HIGH);
    digitalWrite(26, HIGH);
    digitalWrite(27, HIGH);
    digitalWrite(14, HIGH);
    digitalWrite(13, HIGH);
    delay(3000);
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Initializing Sensor Node");
    printMacAddress(mac_sens_node[SENSOR_NODE_ID - 1]);
    init_current_sensor();

    loraPackage = xQueueCreate(10, sizeof(Packet));
    if (!loraPackage)
    {
        Serial.println("❌ Failed to create Queue");
        while (1)
            ;
    }

    loraInit();                               // Khởi tạo LoRa
    loraConfig(433E6, 14, 7, 125E3, 0xA5, 5); // Cấu hình LoRa: tần số 433MHz, công suất phát 17dBm, SF7, BW125kHz, sync word 0xA5, CR5 (4/5)

    delay(1000);
    LoRa.idle(); // Bắt đầu lắng nghe sau khi init
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }
    delay(1000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("LoRa Receiver");
    display.display();

    xTaskCreatePinnedToCore(
        LoRaProcessTask,        // Hàm xử lý
        "LoRaProcessTask",      // Tên Task
        4096,          // Stack size
        NULL,           // Tham số truyền vào
        3,              // Mức ưu tiên (1 = thấp)
        &HandleLoRaProcessTask, // Handle
        1               // Chạy trên core 1
    );

    xTaskCreatePinnedToCore(
        LoRaReceiveTask,
        "LoRaReceiveTask",
        4096,
        NULL,
        1,
        &HandleLoRaReceiveTask,
        1
    );
}

void loop()
{
    // đọc dữ liệu cảm biến mỗi 0.2s
    if(millis() - lastReadTime > readInterval)
    {    
        lastReadTime = millis();
        count = check_sensor_status();
        read_sensor_node_data();
    }
    // Gửi dữ liệu cảm biến định kỳ
    if (millis() - lastSendTime >= sendInterval)
    {
        lastSendTime = millis();
        if(sendSensorData(mac_sens_node[SENSOR_NODE_ID - 1]))
        {
            Serial.println("📤 Gửi dữ liệu cảm biến thành công!");
            total_send_count++;
        }
        else 
            Serial.println("❌ Chưa thể gửi dữ liệu cảm biến.");
    }

    if (ackReceived)
    {
        ackReceived = false;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(20, 0);
        display.println("LoRa Receiver");
    
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
        display.print("RSSI: ");
        display.println(pkgRSSI);
        display.print("SNR: ");
        display.println(pkgSNR);
        display.print("Sent: ");
        display.println(total_send_count);
        display.print("Active sensor: ");
        display.println(count);
        display.display();
        Serial.println("✅ Đã nhận ACK từ Gateway, đi ngủ thôi");
        deepSleepStart();
    }

    if(total_send_count == 10)
    {
        Serial.println("Gửi 10 lần không có phản hồi ACK, đi ngủ thôi");
        deepSleepStart();
    }
}