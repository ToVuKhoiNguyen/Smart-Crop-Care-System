#include <WiFi.h>

// Replace with your network credentials
const char* ssid = "512BB";
const char* password = "1123581321";

uint16_t count = 0;

// Set web server port number to 80
WiFiServer server(80);

// Current time
unsigned long currentTime = millis();

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    Serial.println("New Client.");          // print a message out in the serial port
    while (client.connected()) {            // loop while the client's connected
      count = 0;
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        // Serial.println(c);               // print it out the serial monitor
        count++;                  
      }
    }
    Serial.print("Total count: ");
    Serial.println(count);
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
  if(millis() - currentTime > 1000){
    currentTime = millis();
    digitalWrite(2, !digitalRead(2));
  }
}