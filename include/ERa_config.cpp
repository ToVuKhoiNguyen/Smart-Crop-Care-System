#include "sensor_config.h"
#include "device_config.h"

// const char ssid[] = "P511A";
// const char pass[] = "passcua319b";
const char ssid[] = "Phong209";
const char pass[] = "pttkdtvt";
// const char ssid[] = "512B";
// const char pass[] = "1123581321";
// const char ssid[] = "Toan Store";
// const char pass[] = "66668888";
// const char ssid[] = "Nguyen Thi Thanh";
// const char pass[] = "66668888";

WiFiClient mbTcpClient;

/* This function will run every time ERa is connected */
ERA_CONNECTED()
{
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa connected!"));
}

/* This function will run every time ERa is disconnected */
ERA_DISCONNECTED()
{
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa disconnected!"));
}

// Nhận lệnh từ web điều khiển relay
ERA_WRITE(V101)
{
    int value = param.getInt();
    Serial.print("Received command for Relay 1: ");
    Serial.println(value);
    sendControlCommand(mac_ctrl_node[0], 1, uint8_t(value), 0);
    ERa.virtualWrite(V101, value);
}

ERA_WRITE(V102)
{
    int value = param.getInt();
    Serial.print("Received command for Relay 2: ");
    Serial.println(value);
    sendControlCommand(mac_ctrl_node[0], 2, uint8_t(value), 0);
    ERa.virtualWrite(V102, value);
}

ERA_WRITE(V103)
{
    int value = param.getInt();
    Serial.print("Received command for Relay 3: ");
    Serial.println(value);
    sendControlCommand(mac_ctrl_node[0], 3, uint8_t(value), 0);
    ERa.virtualWrite(V103, value);
}

ERA_WRITE(V104)
{
    int value = param.getInt();
    Serial.print("Received command for Relay 4: ");
    Serial.println(value);
    sendControlCommand(mac_ctrl_node[0], 4, uint8_t(value), 0);
    ERa.virtualWrite(V104, value);
}

ERA_WRITE(V105)
{
    int value = param.getInt();
    Serial.print("Received command for Relay 5: ");
    Serial.println(value);
    sendControlCommand(mac_ctrl_node[0], 5, uint8_t(value), 0);
    ERa.virtualWrite(V105, value);
}