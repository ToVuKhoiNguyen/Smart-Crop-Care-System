#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <stdlib.h>
#include <Arduino.h>

#define STX1 0x4E
#define STX2 0x57
#define END_BYTE 0x68
#define NUMBER_OF_FIXED_BYTES 14 // số byte cố định trong gói tin
#define NUMBER_OF_SENS_NODES 2
#define NUMBER_OF_CTRL_NODES 2
#define NUMBER_OF_RELAYS 5
#define NUMBER_OF_MAC_BYTES 6

// cấu hình lệnh để giải mã
#define SENSOR_VAL 0x01
#define CONTROL_CMD 0x02
#define CONFIG_REQ 0x03
#define STATUS 0x04
#define ACK 0x05
#define CONFIG_SEN 0x06
#define CONFIG_CTRL 0x07

// ID cho các bơm
#define WATER_PUMP_ID 2 // Bơm nước
#define A_PUMP_ID 3 // Bơm dung dịch A
#define B_PUMP_ID 4 // Bơm dung dịch B
#define HNO3_PUMP_ID 5 // Bơm HNO₃

// Cấu hinh bật tắt relay
#define RELAY_ON 0x01
#define RELAY_OFF 0x00

// node define
#define GATEWAY_NODE
// #define SENSOR_NODE
// #define CONTROL_NODE

// ======= Định nghĩa MAC của các NODE ĐIỀU KHIỂN =========   //ví dụ
// mac_ctrlnode1: 5C:01:3B:33:E3:50 -> 0x5C, 0x01, 0x3B, 0x33, 0xE3, 0x50
const uint8_t mac_ctrl_node[NUMBER_OF_CTRL_NODES][NUMBER_OF_MAC_BYTES] = {
                    {0x5C, 0x01, 0x3B, 0x33, 0xE3, 0x50},
                    {0xF4, 0x65, 0x0B, 0x56, 0x48, 0x04}};

const uint8_t mac_sens_node[NUMBER_OF_SENS_NODES][NUMBER_OF_MAC_BYTES] = {
                    {0xF4, 0x65, 0x0B, 0x54, 0x9F, 0x34},
                    {0xF4, 0x65, 0x0B, 0x56, 0x30, 0xFC}};

void set_SPI_CS_pin();

/* SD Card*/
#define SD_CS 14  // chân CS cho module SD

void init_sd_card();
void logDataToSD(float temp, float humid, float ec, float ph, float volt, float curr, float light, int moisture);
void get_sd_card_type();

/* LoRa*/
#define LORA_CS 5
#define LORA_RST 4
#define LORA_DIO0 2

#define MAX_PACKET_SIZE 64
struct Packet
{
    uint8_t data[MAX_PACKET_SIZE];
    size_t len;
};

void loraInit(void);
void loraConfig(int fr, int pw, int sf, int bw, int sw, int cr);
void createQueue(void);
void loraReadPackage(void);

#if defined(GATEWAY_NODE)
void loraProcessPackage(void);
#endif

// sensor node
#if defined(SENSOR_NODE)
    bool sendSensorData(const uint8_t *mac_address);
    void sendSensorConfig(const uint8_t *mac_address);
    void decodeSensorPacket(uint8_t *data, uint8_t incomingLength, const uint8_t *macAdress);
#endif

// gateway node
#if defined(GATEWAY_NODE)
    // các hàm xử lý dữ liệu
    void decodeIncomingPacket(uint8_t *data, uint8_t incomingLength);
    void handleSensorData(uint8_t *data, uint8_t length);
    void handleSensorConfig(uint8_t *data, uint8_t length);
    void handleControlPacket(uint8_t *data, uint8_t length, uint8_t packetType);

    // các hàm gửi dữ liệu
    void sendControlCommand(const uint8_t *mac, uint8_t relay, uint8_t value, uint8_t duration);
    void sendControlConfig(const uint8_t *mac);
    void sendNodePacket(uint8_t packetType, const uint8_t *mac);
#endif

// control node
#if defined(CONTROL_NODE)
    const uint8_t relayPins[NUMBER_OF_RELAYS] = {25, 26, 27, 14, 13};
    const uint8_t buttonPins[NUMBER_OF_RELAYS] = {36, 39, 34, 35, 32};

    #define RELAY_1_HEADER 0x01
    #define RELAY_2_HEADER 0x02
    #define RELAY_3_HEADER 0x03
    #define RELAY_4_HEADER 0x04
    #define RELAY_5_HEADER 0x05

    void initRelay(void);
    void initButton(void);
    void setRelay(uint8_t id, bool state);
    void toggleRelay(uint8_t id);
    void sendControlPacket(uint8_t packetType, const uint8_t *macAdress);
    void decodeControlPacket(uint8_t *data, uint8_t incomingLength, const uint8_t *macAdress);
    void handleControlCommand(uint8_t *data, const uint8_t *macAdress);
    void handleControlConfig(uint8_t *data, uint8_t length);
    void checkAutoOffRelays();
#endif

// all node
void getMacAddress(uint8_t *mac);
void printMacAddress(const uint8_t *mac);
bool compareMacAddress(const uint8_t *mac1, const uint8_t *mac2);
bool waitForAck(uint32_t timeout, const uint8_t *expectedMac);
uint16_t CalculateCRC(uint8_t *data, uint16_t length);

/* TFT screen*/
#if defined(GATEWAY_NODE)
typedef enum
{
    MAIN_DASHBOARD,
    NODE_DASHBOARD,
    SENSOR_DASHBOARD,
    CONTROL_DASHBOARD,
} tft_screen_select_t;

#define MAIN_DASHBOARD_TIME_UPDATE 1000

#define RELAY_ON_COLOR TFT_GREEN
#define RELAY_OFF_COLOR TFT_LIGHTGREY

void init_TFT_screen();
void touch_calibrate();
void test_touch_screen();
void check_touch_screen();
void jpegRender(int xpos, int ypos);
void jpegInfo();
void showTime(uint32_t msTime);
void drawSdJpeg(const char *filename, int xpos, int ypos);

#if defined(TFT_BMP)
#include <FS.h>
void drawBmp(const char *filename, int16_t x, int16_t y);
uint16_t read16(File &f);
uint32_t read32(File &f);
#endif

void draw_main_dashboard();
void draw_node_dashboard();
void draw_sensor_dashboard(uint8_t ID);
void draw_control_dashboard(uint8_t ID);

// main dashboard
void update_main_dashboard_time();
void update_main_dashboard_sensor();
void update_main_dashboard_communicate();

// control dashboard
void update_control_dashboard(uint8_t order);
#endif

#endif // SENSOR_CONFIG_H