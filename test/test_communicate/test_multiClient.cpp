#include <WiFi.h>
#include <vector>

const char *ssid = "512B";
const char *password = "112358132134";

WiFiServer server(80); // Server trên cổng 80
std::vector<WiFiClient> clientQueue; // Hàng đợi lưu các client

void handleClient(WiFiClient& client);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Đang kết nối WiFi...");
  }
  Serial.println("WiFi đã kết nối.");
  server.begin();
  Serial.println("Server đã bắt đầu hoạt động.");
  Serial.print("Địa chỉ IP của server: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Kiểm tra xem có client mới không
  WiFiClient newClient = server.available();
  if (newClient) {
    Serial.println("Client mới kết nối.");
    clientQueue.push_back(newClient); // Đưa client mới vào hàng đợi
    IPAddress clientIP = newClient.remoteIP();
    Serial.print("Địa chỉ IP của client mới: ");
    Serial.println(clientIP);
  }

  // Xử lý client đầu tiên trong hàng đợi
  if (!clientQueue.empty()) {
    WiFiClient currentClient = clientQueue.front(); // Lấy client đầu tiên
    if (currentClient.connected()) {
      Serial.println("Đang xử lý client.");
      handleClient(currentClient); // Hàm xử lý dữ liệu từ client
    } else {
      Serial.println("Client không còn kết nối, xóa khỏi hàng đợi.");
    }

    // Xóa client đã xử lý xong khỏi hàng đợi
    clientQueue.erase(clientQueue.begin());

    // Đưa client trở lại cuối hàng đợi nếu vẫn còn kết nối
    if (currentClient.connected()) {
      Serial.println("Đưa client trở lại cuối hàng đợi.");
      clientQueue.push_back(currentClient);
    } else {
      Serial.println("Client đã ngắt kết nối hoàn toàn.");
    }
  }
}

void handleClient(WiFiClient& client) {
  unsigned long startTime = millis();

  while (client.connected() && (millis() - startTime < 5000)) { // Xử lý tối đa 5 giây
    if (client.available()) {
      String message = client.readStringUntil('\n'); // Đọc dữ liệu từ client
      Serial.println("Dữ liệu nhận được: " + message);

      // Trả lời client
      client.println("Server đã nhận: " + message);
    }
  }

  if (millis() - startTime >= 5000) {
    Serial.println("Hết thời gian xử lý cho client này.");
  }
}
