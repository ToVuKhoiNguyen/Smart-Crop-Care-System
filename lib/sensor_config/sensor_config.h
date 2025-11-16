#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <RTClib.h>
#include "device_config.h"

/* Light Sensor */
#define TSL2561_I2C_ADDRESS 0x39

void init_light_sensor(void);
void light_sensor_config(void);
float get_luminorsity();

/* INA226 - Current sensor */
#define INA226_I2C_ADDRESS 0x41
#define MAX_VOLTAGE 4.2
#define MIN_VOLTAGE 3.0

void init_current_sensor(void);
float get_sourse_voltage_V();
float get_shunt_voltage_V();
float get_current_mA();
float get_power_mW();

/* SHT20 - Temperature/Humidity sensor*/
#define SHT20_ADDRESS 0x40

float get_SHT20_temperature();
float get_SHT20_humidity();

/* DHT11 - Temperature/Humidity sensor*/
#define DHTPIN 26     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11     // DHT 11
// #define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

void init_DHT_sensor(void);
void get_DHT_data(float *temperature, float *humidity);

/* pH Sensor*/
#define PH_SENSOR_TO_ADS1115_PIN 0
#define ADS_PH_ADDRESS 0x4A
#define PH_SLOPE            7.09
#define PH_OFFSET           -0.077// deviation compensate
#define SAMPLING_INTERVAL   20
#define PRINT_INTERVAL      800
#define ARRAY_LENGTH        40      // times of collection

void init_ph_sensor(void);
void cal_ph_value(void);
float get_ph_value();
double avergearray(int *arr, int number);
float ph_model_real(float t);
float getDurationForPH(float targetPH, float t_water);

/* EC Sensor*/
#define EC_SENSOR_TO_ADS1115_PIN 0
#define ADS_EC_ADDRESS 0x48

void init_ec_sensor(void);
float get_ec_value();
float ec_model_linear(float t);
float getDurationForEC(float targetEC);

/* Soil moisture Sensor*/
#define SOIL_MOISTURE_PIN 34 // Chân analog cho cảm biến độ ẩm đất
void init_soil_moisture_sensor(void);

/* RTC sensor - DS3231*/
// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 35

// Thời gian bật và tắt relay
#define START_HOUR 8
#define START_MINUTE 0
#define END_HOUR 18
#define END_MINUTE 59
#define INTERVAL_MINUTES 15 // Khoảng cách giữa các lần bật (x phút)
#define INTERVAL_HOURS 6    // Khoảng cách giữa các lần bật (y giờ)
#define RELAY_DURATION 10 // Thời gian bật relay (10 giây)

void init_RTC_sensor();
void init_RTC_alarm();

#if defined(GATEWAY_NODE)
void calculatePumpTime();
void check_alarm();
bool checkTime(DateTime time);
void check_alarm_flag();
void sendCommandAutoMode();
#endif

void get_current_time();
void onAlarm();
void RTC_set_time(DateTime setTime);
void RTC_set_alarm1(DateTime alarmTime);
void get_RTC_temperature();

/* All Sensors*/

#define VOLTAGE_SENSOR 0x01
#define CURRENT_SENSOR 0x02
#define LIGHT_SENSOR 0x03
#define TEMPERATURE_SENSOR 0x04
#define HUMIDITY_SENSOR 0x05
#define SOIL_MOISTURE_SENSOR 0x06
#define EC_SENSOR 0x07
#define PH_SENSOR 0x08

typedef struct sensor_data
{
    float temperature;
    float humidity;
    int soil_moisture;
    float voltage;
    float current;
    float ec_value;
    float ph_value;
    float lumiorsity_value;
} sensor_data; // Declare a global variable of type sensor_data

typedef struct sensor_state
{
    bool temperature_sensor;
    bool humidity_sensor;
    bool soil_moisture_sensor;
    bool voltage_sensor;
    bool current_sensor;
    bool ph_sensor;
    bool ec_sensor;
    bool light_sensor;
} sensor_state; // Declare a global variable of type sensor_state

void init_sensor(void);
void read_gateway_sensor_data(float *voltage, float *current, float *temperature);
float filter_kalman(float value, float& last_est, float& err_est);
float filter_median(float value, float* buffer, int& count, const int size);
void read_sensor_node_data(void);
uint8_t check_sensor_status();
bool checkI2CConnection(uint8_t address);
void checkAnalogConnection(uint8_t *count);

#endif // SENSOR_CONFIG_H