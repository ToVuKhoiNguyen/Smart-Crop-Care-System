#define TINY_GSM_MODEM_SIM7600
#include <WiFi.h>
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <ERa.hpp>
#include <Automation/ERaSmart.hpp>
#include <Time/ERaBaseTime.hpp>

#define simSerial Serial2
#define MCU_SIM_BAUDRATE 115200
#define MCU_SIM_TX_PIN 17
#define MCU_SIM_RX_PIN 16
#define MCU_SIM_EN_PIN 13

const char apn[] = "v-internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

HardwareSerial SerialGsm(1); // UART1 cho modem
TinyGsm modem(SerialGsm);
ERaBaseTime syncTime;
ERaSmart smart(ERa, syncTime);

// Trạng thái hệ thống
volatile bool wifiConnected = false;
volatile bool gprsConnected = false;
volatile bool forceReconnect = false;

void sim_at_wait()
{
    delay(100);
    while (simSerial.available())
    {
        Serial.write(simSerial.read());
    }
}

bool sim_at_cmd(String cmd)
{
    simSerial.println(cmd);
    sim_at_wait();
    return true;
}

void enable4GData()
{
    Serial.println("[4G] Restarting modem...");
    modem.restart();
    delay(5000);

    modem.sendAT("+CNMP=2"); // Auto GSM/3G/4G
    delay(500);
    modem.sendAT("+CMNB=2"); // All mode
    delay(500);
    modem.sendAT("+COPS=0"); // Auto operator
    delay(500);

    if (modem.waitForNetwork(60000))
    {
        Serial.println("[4G] Network found!");
        if (modem.gprsConnect(apn, gprsUser, gprsPass))
        {
            Serial.println("[4G] GPRS Connected!");
        }
        else
        {
            Serial.println("[4G] GPRS Connect Failed!");
        }
    }
    else
    {
        Serial.println("[4G] Network Registration Failed!");
    }
}

void init_module_sim()
{
    SerialGsm.begin(MCU_SIM_BAUDRATE);
    pinMode(MCU_SIM_EN_PIN, OUTPUT);
    digitalWrite(MCU_SIM_EN_PIN, LOW);
    delay(1500);
    digitalWrite(MCU_SIM_EN_PIN, HIGH);
}
