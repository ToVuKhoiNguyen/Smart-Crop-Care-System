#include <Adafruit_TSL2561_U.h>
#include <Adafruit_Sensor.h>
#include "INA226_WE.h"
#include "Wire.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_I2CDevice.h>
#include <RTClib.h>
#include <DHT.h>
#include <DHT_U.h>
#include "sensor_config.h"
#include "device_config.h"

/* All sensors*/

sensor_data sensor;         // Declare a global variable of type sensor_data
sensor_state sensor_status; // Declare a global variable of type sensor_state
bool is_SHT20 = false;      // Chỉ dùng SHT20 hoặc DHT11, không dùng cả 2 một lúc
bool is_DHT11 = false;
/* Light Sensor*/
Adafruit_TSL2561_Unified TSL = Adafruit_TSL2561_Unified(TSL2561_I2C_ADDRESS, 12345);

void init_light_sensor(void)
{
    Serial.println("Init Light Sensor");
    if (!TSL.begin())
    {
        /* There was a problem detecting the TSL2561 ... check your connections */
        Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
    light_sensor_config();
    Serial.println("Success");
}

void light_sensor_config(void)
{
    /* You can also manually set the gain or enable auto-gain support */
    // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
    // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
    TSL.enableAutoRange(true); /* Auto-gain ... switches automatically between 1x and 16x */

    /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
    //   TSL.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
    // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
    TSL.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS); /* 16-bit data but slowest conversions */

    /* Update these values depending on what you've set above! */
    // Serial.println("------------------------------------");
    // Serial.print("Gain:         ");
    // Serial.println("Auto");
    // Serial.print("Timing:       ");
    // Serial.println("402 ms");
    // Serial.println("------------------------------------");
}

float get_luminorsity()
{
    /* Get a new sensor event */
    sensors_event_t event;
    TSL.getEvent(&event);

    /* Display the results (light is measured in lux) */
    //   if (event.light)
    //   {
    //     Serial.print(event.light); Serial.println(" lux");
    //   }
    //   else
    //   {
    //     /* If event.light = 0 lux the sensor is probably saturated
    //        and no reliable data could be generated! */
    //     Serial.println("Sensor overload");
    //   }
    return event.light;
}

/* INA226 - Current sensor */
INA226_WE ina226 = INA226_WE(INA226_I2C_ADDRESS);

void init_current_sensor(void)
{
    Wire.begin();
    Serial.println("Init Current Sensor");
    if (!ina226.init())
    {
        Serial.println("could not connect. Fix and Reboot");
        while (1)
            ;
    }

    /* Set measure mode
        POWER_DOWN - INA226 switched off
        TRIGGERED  - measurement on demand
        CONTINUOUS  - continuous measurements (default)*/

    // ina226.setMeasureMode(CONTINUOUS);

    ina226.setResistorRange(0.1, 0.8); // 0.1 Ohm - 1.3 A
    ina226.setAverage(AVERAGE_64);
    ina226.setConversionTime(CONV_TIME_8244); //  0 reads fast  ...  7 stair-casing, slower reads)
    ina226.setCorrectionFactor(1.1);
    ina226.waitUntilConversionCompleted();
    Serial.println("Success");
}

float get_sourse_voltage_V()
{
    return ina226.getBusVoltage_V();
}

float get_shunt_voltage_V()
{
    return ina226.getShuntVoltage_V();
}

float get_current_mA()
{
    return ina226.getCurrent_mA();
}

float get_power_mW()
{
    return ina226.getBusPower();
}

/* SHT20 - Temperature/Humidity sensor*/
float get_SHT20_temperature()
{
    Wire.beginTransmission(SHT20_ADDRESS);
    Wire.write(0xF3); // Command to measure temperature
    if (Wire.endTransmission() != 0)
        return -999; // Error

    delay(100); // Wait for measurement

    Wire.requestFrom(SHT20_ADDRESS, 2);
    if (Wire.available() != 2)
        return -999; // Error

    uint16_t rawData = (Wire.read() << 8) | Wire.read();
    return -46.85 + 175.72 * (rawData / 65536.0);
}

float get_SHT20_humidity()
{
    Wire.beginTransmission(SHT20_ADDRESS);
    Wire.write(0xF5); // Command to measure humidity
    if (Wire.endTransmission() != 0)
        return -999; // Error

    delay(100); // Wait for measurement

    Wire.requestFrom(SHT20_ADDRESS, 2);
    if (Wire.available() != 2)
        return -999; // Error

    uint16_t rawData = (Wire.read() << 8) | Wire.read();
    return -6.0 + 125.0 * (rawData / 65536.0);
}

/* DHT11 - Temperature/Humidity sensor*/
DHT_Unified dht(DHTPIN, DHTTYPE);

void init_DHT_sensor(void)
{
    Serial.println("Init DHT Sensor");
    dht.begin();
}

void get_DHT_data(float *temperature, float *humidity)
{
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    *temperature = event.temperature;
    dht.humidity().getEvent(&event);
    *humidity = event.relative_humidity;
}

/* pH Sensor*/
Adafruit_ADS1115 adsPH;
int pHArray[ARRAY_LENGTH]; // Store the average value of the sensor feedback
int pHArrayIndex = 0;
static unsigned long samplingTime = 0;
static float pHValue, pHvoltage;

// --- Thêm cho kiểm tra ổn định ---
float lastPH = 0;
int stableCount = 0;
const float STABLE_THRESHOLD = 0.02; // chênh lệch nhỏ hơn 0.02 pH được coi là ổn định
const int REQUIRED_STABLE_COUNT = 10;
bool isPHStable = false;

void init_ph_sensor(void)
{
    if (!adsPH.begin(ADS_PH_ADDRESS))
    {
        Serial.println("Failed to initialize ADS - PH sensor");
        while (1)
            ;
    }
}

void cal_ph_value(void)
{
    if (millis() - samplingTime > SAMPLING_INTERVAL)
    {
        pHArray[pHArrayIndex++] = adsPH.readADC_SingleEnded(PH_SENSOR_TO_ADS1115_PIN);
        if (pHArrayIndex == ARRAY_LENGTH)
            pHArrayIndex = 0;
        pHvoltage = avergearray(pHArray, ARRAY_LENGTH) * 0.125 / 1000.0;
        pHValue = PH_SLOPE * pHvoltage + PH_OFFSET;
        samplingTime = millis();
    }
    // Kiểm tra độ ổn định của pH
    if (fabs(pHValue - lastPH) < STABLE_THRESHOLD)
    {
        stableCount++;
        if (stableCount >= REQUIRED_STABLE_COUNT)
        {
            isPHStable = true;
        }
    }
    else
    {
        stableCount = 0;
        isPHStable = false;
    }
    lastPH = pHValue;
}

float get_ph_value()
{
    cal_ph_value();
    return pHValue;
}

double avergearray(int *arr, int number)
{
    int i, max, min;
    long amount = 0;
    if (number < 5)
    {
        for (i = 0; i < number; i++)
            amount += arr[i];
        return (double)amount / number;
    }
    else
    {
        max = min = arr[0];
        for (i = 1; i < number; i++)
        {
            if (arr[i] > max)
                max = arr[i];
            else if (arr[i] < min)
                min = arr[i];
            amount += arr[i];
        }
        amount = amount - max - min;
        return (double)amount / (number - 2);
    }
}

// === MÔ HÌNH pH bậc 2 đã fit thực tế ===
// pH(t) = 0.0321 * t^2 - 0.3460 * t + 7.7548
float ph_model_real(float t)
{
    return 0.0321 * t * t - 0.3460 * t + 7.2; // cho nước về 7 thay vì 7.7548
}

// pH → Thời gian bơm (float phút)
float getDurationForPH(float targetPH, float t_water)
{
    for (float t = 0.0; t <= t_water; t += 0.01)
    {
        if (ph_model_real(t) <= targetPH)
            return t;
    }
    return t_water;
}

/* EC Sensor*/
Adafruit_ADS1115 adsEC;

void init_ec_sensor(void)
{
    if (!adsEC.begin(ADS_EC_ADDRESS))
    {
        Serial.println("Failed to initialize ADS - EC sensor");
        while (1)
            ;
    }
}

float get_ec_value()
{
    return (float)adsEC.readADC_SingleEnded(EC_SENSOR_TO_ADS1115_PIN);
}

// === MÔ HÌNH EC (sau khi pha đậm 1.33) ===
// EC(t) = 637.09 * t + 383.57
float ec_model_linear(float t)
{
    return 637.09 * t + 383.57;
}

// EC → Thời gian bơm (trả về float phút)
float getDurationForEC(float targetEC)
{
    return (targetEC - 383.57) / 637.09;
}

/* Soil moisture Sensor*/
void init_soil_moisture_sensor(void)
{
    Serial.println("Init Soil Moisture Sensor");
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    sensor_status.soil_moisture_sensor = true;
}

/* RTC sensor - DS3231*/
RTC_DS3231 rtc;
char daysOfTheWeek[7][5] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
int lastTriggeredMinute = -1; // -1 có nghĩa là chưa có lần bật nào
bool alarmFlag = false;
int startTotalMinutes = START_HOUR * 60 + START_MINUTE;
int endTotalMinutes = END_HOUR * 60 + END_MINUTE;
// set the alarms
// (year, month, day, hour, minutes, seconds)
// DateTime alarm1Time = DateTime(2024, 12, 18, 12, 49, 0);
// DateTime alarm2Time = DateTime(2024, 12, 18, 11, 10, 0);
// DateTime setTime = DateTime(2025, 5, 5, 22, 18, 0);
DateTime nextAlarm;

void init_RTC_sensor()
{
    // initializing the rtc
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        while (1)
            delay(10);
    }
    // we don't need the 32K Pin, so disable it
    rtc.disable32K();
    // RTC_set_time(setTime);
}

void init_RTC_alarm()
{
    // Trigger an interrupt when the alarm happens
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);

    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);

    // stop oscillating signals at SQW Pin, otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);

    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);
}

#if defined(GATEWAY_NODE)
// chỗ này là mấy cái thông số cho chế độ tự động
// --- Tham số mục tiêu ---
uint8_t relayDuration[NUMBER_OF_RELAYS] = {0, 0, 0, 0, 0};
float EC_target = 2000.0;
float pH_target = 6.5;
float totalVolume_ml = 4000.0;
int flowRateWater_ml_per_min = 2000;

void calculatePumpTime()
{
    // Tính thời gian bơm nước (phút)
    float t_water = (float)totalVolume_ml / flowRateWater_ml_per_min;
    Serial.printf("Thời gian bơm nước: %.2f phút\n", t_water);

    // Tính thời gian bơm dung dịch
    float t_ab = min(getDurationForEC(EC_target), t_water);
    float t_hno3 = min(getDurationForPH(pH_target, t_water), t_water);

    Serial.printf("Thời gian bơm A + B: %.2f phút\n", t_ab);
    Serial.printf("Thời gian bơm HNO₃ (clamp): %.2f phút\n", t_hno3);

    // cập nhật cho mảng thời gian - số phút sang số giây
    relayDuration[WATER_PUMP_ID - 1] = (uint8_t)(t_water * 60UL);
    relayDuration[A_PUMP_ID - 1] = (uint8_t)(t_ab * 60UL);
    relayDuration[B_PUMP_ID - 1] = (uint8_t)(t_ab * 60UL);
    relayDuration[HNO3_PUMP_ID - 1] = (uint8_t)(t_hno3 * 60UL);
    // activateRelay(0, (unsigned long)(t_water * 60000UL)); // Nước
    // activateRelay(1, (unsigned long)(t_ab * 60000UL));    // A
    // activateRelay(2, (unsigned long)(t_ab * 60000UL));    // B
    // activateRelay(3, (unsigned long)(t_hno3 * 60000UL));  // HNO₃
}

void check_alarm()
{
    DateTime now = rtc.now();
    // Nếu đang trong khoảng thời gian hoạt động thì tính khoảng thời gian đã trôi qua, set alarm đầu tiên theo chu kỳ x phút và đặt cho nextAlarm
    int elapsedMinutes = (now.hour() * 60 + now.minute()) - startTotalMinutes; // Tính số phút đã trôi qua từ thời gian bắt đầu

    if (now.hour() >= START_HOUR && now.hour() < END_HOUR) // vẫn trong khoảng tgian hoạt động
    {
        if (elapsedMinutes >= 0 && now.minute() != lastTriggeredMinute)
        {
            if (now.second() == 0 && elapsedMinutes % INTERVAL_MINUTES == 0)
            {
                sendCommandAutoMode();
            }
            int offset = elapsedMinutes % INTERVAL_MINUTES;
            nextAlarm = now + TimeSpan(0, 0, INTERVAL_MINUTES - offset, -now.second()); // Tính thời gian alarm kế tiếp
            // DateTime(firstAlarm.year(), firstAlarm.month(), firstAlarm.day(), firstAlarm.hour(), firstAlarm.minute() + intervalMinutes, 0); // Tính thời gian alarm kế tiếp
            RTC_set_alarm1(nextAlarm);
        }
        else
        {
            Serial.println("Không cần đặt alarm, đã có alarm trước đó.");
        }
    }
    else
    {
        Serial.println("Hết khung giờ hoạt động, alarm đặt về giờ bắt đầu: ");
        now = now + TimeSpan(1, 0, 0, 0); // thêm 1 ngày
        DateTime firstAlarm = DateTime(now.year(), now.month(), now.day(), START_HOUR, 0, 0);
        RTC_set_alarm1(firstAlarm);
    }
}

bool checkTime(DateTime time)
{
    // Kiểm tra nếu vẫn trong khoảng 1h đến trước 23h thì đặt tiếp
    int currentTotalMinutes = 60 * time.hour() + time.minute();
    Serial.printf("currentTotalMinutes: %d\n", currentTotalMinutes);
    if (currentTotalMinutes >= startTotalMinutes && currentTotalMinutes <= endTotalMinutes)
    {
        return true; // Trong khoảng thời gian hoạt động
    }
    else
    {
        return false; // Ngoài khoảng thời gian hoạt động
    }
}

void check_alarm_flag()
{
    if (alarmFlag)
    {
        rtc.clearAlarm(1); // 🛠️ Xóa để tránh ngắt giữ trạng thái
        DateTime now = rtc.now();

        // ====== Điều kiện bật relay theo mốc cố định ======
        if (checkTime(now))
        {
            sendCommandAutoMode();
            // Tính thời gian alarm kế tiếp
            nextAlarm = now + TimeSpan(0, 0, INTERVAL_MINUTES, -now.second()); // +7 phút
            RTC_set_alarm1(nextAlarm);
        }
        else
        {
            Serial.println("Hết khung giờ hoạt động, alarm đặt về giờ bắt đầu: ");

            now = now + TimeSpan(1, 0, 0, 0); // thêm 1 ngày
            nextAlarm = DateTime(now.year(), now.month(), now.day(), START_HOUR, 0, 0);
            RTC_set_alarm1(nextAlarm);
        }
        alarmFlag = false;
    }
}

void sendCommandAutoMode()
{
    Serial.println("Time to turn on relay water and macronutrient.");
    sendControlCommand(mac_ctrl_node[0], WATER_PUMP_ID, RELAY_ON, relayDuration[WATER_PUMP_ID - 1]); // Gửi lệnh bật relay nước
    Serial.printf("Duration relay %d: %d\n", WATER_PUMP_ID, relayDuration[WATER_PUMP_ID - 1]);
    vTaskDelay(pdMS_TO_TICKS(150));
    sendControlCommand(mac_ctrl_node[0], A_PUMP_ID, RELAY_ON, relayDuration[A_PUMP_ID - 1]); // Gửi lệnh bật relay đa lượng
    Serial.printf("Duration relay %d: %d\n", A_PUMP_ID, relayDuration[A_PUMP_ID - 1]);
    vTaskDelay(pdMS_TO_TICKS(150));
    sendControlCommand(mac_ctrl_node[0], B_PUMP_ID, RELAY_ON, relayDuration[B_PUMP_ID - 1]); // Gửi lệnh bật relay đa lượng
    Serial.printf("Duration relay %d: %d\n", B_PUMP_ID, relayDuration[B_PUMP_ID - 1]);
    vTaskDelay(pdMS_TO_TICKS(150));
    if (nextAlarm.hour() >= START_HOUR && nextAlarm.hour() <= END_HOUR - INTERVAL_HOURS)
    {
        Serial.println("Time to turn on relay micronutrient.");
        sendControlCommand(mac_ctrl_node[0], HNO3_PUMP_ID, RELAY_ON, relayDuration[HNO3_PUMP_ID - 1]); // Gửi lệnh bật relay vi lượng
        Serial.printf("Duration relay %d: %d\n", HNO3_PUMP_ID, relayDuration[HNO3_PUMP_ID - 1]);
    }
}

#endif

void get_current_time()
{
    // Get the current time from the RTC
    DateTime now = rtc.now();

    // Getting each time field in individual variables
    // And adding a leading zero when needed;
    String yearStr = String(now.year(), DEC);
    String monthStr = (now.month() < 10 ? "0" : "") + String(now.month(), DEC);
    String dayStr = (now.day() < 10 ? "0" : "") + String(now.day(), DEC);
    String hourStr = (now.hour() < 10 ? "0" : "") + String(now.hour(), DEC);
    String minuteStr = (now.minute() < 10 ? "0" : "") + String(now.minute(), DEC);
    String secondStr = (now.second() < 10 ? "0" : "") + String(now.second(), DEC);
    String dayOfWeek = daysOfTheWeek[now.dayOfTheWeek()];

    // Complete time string
    String formattedTime = dayOfWeek + ", " + yearStr + "-" + monthStr + "-" + dayStr + " " + hourStr + ":" + minuteStr + ":" + secondStr;

    // Print the complete formatted time
    Serial.println(formattedTime);
}

void onAlarm()
{
    Serial.println("Alarm occurred!");
    // do sth when alarm occurs
    // int state = digitalRead(LED_BUILTIN);
    // digitalWrite(LED_BUILTIN, !state);
    alarmFlag = true;
}

void RTC_set_time(DateTime setTime)
{
    // if (rtc.lostPower())
    // {
    //     // this will adjust to the date and time at compilation
    //     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // }
    // Uncomment if you need to adjust the time
    rtc.adjust(setTime);
}

void RTC_set_alarm1(DateTime alarmTime)
{
    // Schedule Alarm1 to fire when the minutes match
    if (!rtc.setAlarm1(alarmTime, DS3231_A1_Hour))
    { // this mode triggers the alarm when the minutes match
        Serial.println("Error, alarm wasn't set!");
    }
    else
    {
        Serial.print("Alarm tiếp theo lúc: ");
        Serial.println(alarmTime.timestamp());
    }
}

void get_RTC_temperature()
{
    // Get the temperature from the RTC
    float temperature = rtc.getTemperature();
    Serial.print("RTC Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
}

/* All Sensors*/
void init_sensor(void)
{
    init_light_sensor();
    init_current_sensor();
    init_ph_sensor();
    init_ec_sensor();
}

void read_gateway_sensor_data(float *voltage, float *current, float *temperature)
{
    // Read the sensor data
    *voltage = get_sourse_voltage_V();
    *current = get_current_mA();
    *temperature = rtc.getTemperature();
}

// biến này để đo thời gian giữa các lần đo công suất
unsigned long lastTimePower = 0;
// Cho dòng điện
float current_kalman_est = 0, current_kalman_err = 1;

// Cho ánh sáng
float light_kalman_est = 0, light_kalman_err = 1;

// Cho độ ẩm
float humid_median_buffer[5] = {0};
int humid_median_count = 0;

float filter_kalman(float value, float &last_est, float &err_est)
{
    const float err_measure = 0.5, q = 0.01;
    float gain = err_est / (err_est + err_measure);
    float estimate = last_est + gain * (value - last_est);
    err_est = (1.0 - gain) * err_est + fabs(last_est - estimate) * q;
    last_est = estimate;
    return estimate;
}

float filter_median(float value, float *buffer, int &count, const int size)
{
    for (int i = size - 1; i > 0; i--)
        buffer[i] = buffer[i - 1];
    buffer[0] = value;
    if (count < size)
        count++;

    float sorted[5];
    memcpy(sorted, buffer, sizeof(float) * size);
    std::sort(sorted, sorted + count);
    return sorted[count / 2];
}

void read_sensor_node_data(void)
{
    if (sensor_status.temperature_sensor && sensor_status.humidity_sensor)
    {
        if (is_SHT20)
        {
            sensor.temperature = get_SHT20_temperature();
            sensor.humidity = get_SHT20_humidity();
        }
        else if (is_DHT11)
        {
            float temperature, humidity;
            get_DHT_data(&temperature, &humidity);
            sensor.temperature = temperature;
            sensor.humidity = filter_median(humidity, humid_median_buffer, humid_median_count, 5);
        }
    }

    if (sensor_status.soil_moisture_sensor)
    {
        sensor.soil_moisture = analogRead(SOIL_MOISTURE_PIN);
    }

    if (sensor_status.voltage_sensor && sensor_status.current_sensor)
    {
        // chỗ này thay vì dòng điện thì tính công suất tiêu thụ
        // chuyển voltage thành phần trăm pin
        float battery_voltage = get_sourse_voltage_V();
        float batteryCap = (battery_voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100;
        sensor.voltage = batteryCap;
        // Serial.printf("Battery cap: %f\n", batteryCap);

        float current_raw = get_current_mA();
        float current_filtered = filter_kalman(current_raw, current_kalman_est, current_kalman_err);
        
        // float powermW = get_power_mW();
        float powermW = battery_voltage * current_filtered;
        // Tính năng lượng tiêu thụ:
        float energyIncrement = powermW * (millis() - lastTimePower) / 1000.0 / 3600.0; // mWh
        lastTimePower = millis();
        // sensor.current = get_current_mA();
        sensor.current += energyIncrement;
        // Serial.printf("Energy: %f\n", sensor.current);
    }

    if (sensor_status.ph_sensor)
    {
        sensor.ph_value = get_ph_value();
    }

    if (sensor_status.ec_sensor)
    {
        sensor.ec_value = get_ec_value();
    }

    if (sensor_status.light_sensor)
    {
        float lumiorsity_raw = get_luminorsity();
        sensor.lumiorsity_value = filter_kalman(lumiorsity_raw, light_kalman_est, light_kalman_err);;
    }
}

// hàm này đếm số lượng sensor đang hoạt động
uint8_t check_sensor_status()
{
    uint8_t count = 0;
    // Kiểm tra SHT20 qua I2C
    if (checkI2CConnection(SHT20_ADDRESS))
    {
        // Serial.println("SHT20 sensor is connected and responding.");
        sensor_status.temperature_sensor = true;
        sensor_status.humidity_sensor = true;
        count += 2;
        is_SHT20 = true;
    }
    else
    {
        // Serial.println("Failed to communicate with SHT20 sensor.");
        sensor_status.temperature_sensor = false;
        sensor_status.humidity_sensor = false;
        is_SHT20 = false;
    }

    // nếu SHT20 không hoạt động thì kiểm tra DHT11
    if (!sensor_status.temperature_sensor || !sensor_status.humidity_sensor)
    {
        float temperature, humidity;
        get_DHT_data(&temperature, &humidity);
        if (isnan(temperature))
        {
            // Serial.println(F("Error reading temperature!"));
            sensor_status.temperature_sensor = false;
            is_DHT11 = false;
        }
        else
        {
            sensor_status.temperature_sensor = true;
            count += 1;
            is_DHT11 = true;
        }
        if (isnan(humidity))
        {
            // Serial.println(F("Error reading humidity!"));
            sensor_status.humidity_sensor = false;
        }
        else
        {
            sensor_status.humidity_sensor = true;
            count += 1;
        }
    }

    // Kiểm tra INA226 qua I2C
    if (checkI2CConnection(INA226_I2C_ADDRESS))
    {
        // Serial.println("INA226 sensor is connected and working.");
        sensor_status.current_sensor = true;
        sensor_status.voltage_sensor = true;
        count += 2;
    }
    else
    {
        // Serial.println("Failed to initialize INA226.");
        sensor_status.current_sensor = false;
        sensor_status.voltage_sensor = false;
    }

    // Kiểm tra light sensor qua I2C
    if (checkI2CConnection(TSL2561_I2C_ADDRESS))
    {
        // Serial.println("Light sensor is connected and working.");
        sensor_status.light_sensor = true;
        count += 1;
    }
    else
    {
        // Serial.println("Failed to initialize light sensor.");
        sensor_status.light_sensor = false;
    }

    // Kiểm tra ADS1115 - EC sensor qua I2C
    if (adsEC.begin(ADS_EC_ADDRESS))
    {
        adsEC.setGain(GAIN_ONE); // ±4.096V
        // Serial.println("ADS1115 - EC is connected and working.");
        sensor_status.ec_sensor = true;
        count += 1;
    }
    else
    {
        // Serial.println("Failed to initialize ADS1115 - EC.");
        sensor_status.ec_sensor = false;
    }

    // Kiểm tra ADS1115 - PH sensor qua I2C
    if (adsPH.begin(ADS_PH_ADDRESS))
    {
        adsPH.setGain(GAIN_ONE); // ±4.096V
        // Serial.println("ADS1115 - PH is connected and working.");
        sensor_status.ph_sensor = true;
        count += 1;
    }
    else
    {
        // Serial.println("Failed to initialize ADS1115 - PH.");
        sensor_status.ph_sensor = false;
    }

    if (sensor_status.soil_moisture_sensor)
        count += 1;
    // checkAnalogConnection(&count); // Kiểm tra cảm biến pH qua chân analog

    return count;
}

bool checkI2CConnection(uint8_t address)
{
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0)
    {
        return true; // I2C device is connected
    }
    else
    {
        return false; // I2C device is not connected
    }
}

void checkAnalogConnection(uint8_t *count)
{
    // cách này đang hơi lỏ
    // Kiểm tra độ ẩm đất qua chân analog
    int soilMoistureValue = analogRead(SOIL_MOISTURE_PIN);
    Serial.printf("Soil Moisture value: %d\n", soilMoistureValue);
    if (soilMoistureValue >= 0 && soilMoistureValue <= 4095)
    {
        // Serial.print("Soil Moisture value: ");
        // Serial.println(soilMoistureValue);
        sensor_status.soil_moisture_sensor = true;
        *count += 1;
    }
    else
    {
        // Serial.println("Failed to read soil moisture value.");
        sensor_status.soil_moisture_sensor = false;
    }
}
