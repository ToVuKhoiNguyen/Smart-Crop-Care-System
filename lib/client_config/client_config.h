// #define WIFISSID "512B" // Put your WifiSSID here
// #define PASSWORD "112358132134" // Put your wifi password here
// #define TOKEN "BBUS-Q0ojbU9aBNC7MNEZPqWgbOUvjt06gm" // Put your Ubidots' TOKEN
// #define MQTT_CLIENT_NAME "Solar_Monitoring" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string;
 
/****************************************
  Define Constants
****************************************/
#ifndef client_config_h
#define client_config_h

#include <Arduino.h>

#define VARIABLE_LABEL1 "Solar Voltage" // Assing the variable label
#define VARIABLE_LABEL2 "Current"
#define VARIABLE_LABEL3 "Power"
#define VARIABLE_LABEL4 "KWh"
#define VARIABLE_LABEL5 "Luminorsity"
 
#define DEVICE_LABEL "ESP32"

void client_reconnect(void);
void init_client(void);
bool check_client_connect();
void handle_message(String message, int device_index);
void client_subscribe(void);
void client_publish_status(void);
void client_loop(void);
void init_control_node(void);
void TurnOffAutoMode(void);
void TurnOnAutoMode(void);

#endif
