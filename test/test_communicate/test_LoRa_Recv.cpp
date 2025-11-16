#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

 
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
#define ss 5
#define rst 4
#define dio0 2
String LoRaData;

uint8_t buffer[8];
 
void setup() 
{
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay(); 
  
  while (!Serial);
  Serial.println("LoRa Receiver");
 
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
 
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  
  display.println("LoRa Receiver");
  display.display();
}
 
void loop() 
{
  int packetSize = LoRa.parsePacket();    // try to parse packet
  if (packetSize) 
  {
    
    Serial.println("Received packet");
 
    while (LoRa.available())              // read packet
    {
      // LoRaData = LoRa.readString();
      LoRa.readBytes(buffer, sizeof(buffer));
      // Serial.print(LoRaData); 
      
    }
    for (int i = 0; i < 8; i++) {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.print("RSSI: ");         // print RSSI of packet
    Serial.println(LoRa.packetRssi());
    Serial.println("");
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 0);
    display.println("LoRa Receiver");
  
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.println(LoRaData);
    display.print("RSSI: ");
    display.println(LoRa.packetRssi());
    display.display();
  }
}