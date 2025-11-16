#include <strings.h>
#include <EEPROM.h>
#include <Adafruit_ADS1X15.h>

unsigned long lastMsg = 0;

float voltagePH, voltageEC, phValue, ecValue;
Adafruit_ADS1115 ads;

void setup() {

  Serial.begin(115200);

  Serial.println(" CONNECTED");
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

void loop() {
  //voltagePH = analogRead(PH_PIN)/1024.0*5000;          // read the ph voltage
  // phValue    = (float)analogRead(PH_PIN)/4096*14;
  //voltageEC = analogRead(EC_PIN)/1024.0*5000;
  ecValue    = (float)ads.readADC_SingleEnded(0);

  unsigned long now = millis(); 
  if (now - lastMsg > 1000) {
    lastMsg = now;
    Serial.print("EC value: ");
    Serial.println(ecValue);
  }
}
