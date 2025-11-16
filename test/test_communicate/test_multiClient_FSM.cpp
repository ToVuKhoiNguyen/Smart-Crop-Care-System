#include <WiFi.h>
#include <sys/socket.h>
#include <lwip/sockets.h>
#include <vector>

// Replace with your network credentials
const char *ssid = "TTS_TMRD";
const char *password = "rdtts2024";

// Set web server port number to 80
WiFiServer server(80);
WiFiClient currentClient, newClient;
std::vector<WiFiClient> clientQueue; // Hàng đợi lưu các client
String message = "";                 // Dữ liệu nhận được từ client

enum ServerState
{
  INIT,
  WAIT_FOR_CLIENT,
  TAKE_CLIENT,
  CLIENT_CONNECTED,
  READ_DATA,
  PREPROCESS_DATA,
  PROCESS_DATA,
  PROCESS_DATA_QUEUE,
  SEND_RESPONSE,
  SEND_RESPONSE_QUEUE,
  CHECK_CLIENTQUEUE
};

ServerState state = INIT;

// Current time
unsigned long startTime = millis();

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
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

void loop()
{
  switch (state)
  {
  case INIT:
    Serial.println("INIT state");
    if (server.hasClient())
    {
      server.end();
    }
    server.begin();
    Serial.println("Server started on port 80");

    state = WAIT_FOR_CLIENT;
    break;

  case WAIT_FOR_CLIENT:
    // Serial.println("WAIT_FOR_CLIENT state");
    // Kiểm tra xem có client mới không
    newClient = server.available();
    if (newClient)
    {
      // Sau khi chấp nhận kết nối từ client
      int socket = newClient.fd();

      int keepAlive = 1;    // Thiết lập tùy chọn TCP Keep-Alive
      int keepIdle = 5;     // Thời gian idle trước khi gửi gói Keep-Alive (giây)
      int keepInterval = 3; // Khoảng thời gian giữa các gói Keep-Alive (giây)
      int keepCount = 3;    // Số lần gửi Keep-Alive trước khi kết nối bị coi là mất

      setsockopt(socket, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(keepAlive));
      setsockopt(socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(keepIdle));
      setsockopt(socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(keepInterval));
      setsockopt(socket, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(keepCount));

      Serial.println("New client connected");
      clientQueue.push_back(newClient); // Đưa client mới vào hàng đợi
      Serial.print("Địa chỉ IP của client mới: ");
      Serial.println(newClient.remoteIP());

      state = TAKE_CLIENT;
    }
    if(!clientQueue.empty())
    {
      state = TAKE_CLIENT;
    }
    break;

  case TAKE_CLIENT:
    // Serial.println("TAKE_CLIENT state");
    // Lấy client đầu tiên trong hàng đợi
    while (!clientQueue.empty())
    {
      currentClient = clientQueue.front(); // Lấy client đầu tiên
      if (currentClient.connected())
      {
        state = CLIENT_CONNECTED;
        break;
      }
      else
      {
        clientQueue.erase(clientQueue.begin()); // Xóa client khỏi hàng đợi
        state = TAKE_CLIENT;
      }
    }
    if (clientQueue.empty())
    {
      state = WAIT_FOR_CLIENT;
    }

    break;

  case CLIENT_CONNECTED:
    // Serial.println("CLIENT_CONNECTED state");
    if (currentClient.connected())
    {
      state = READ_DATA;
    }
    else
    {

#ifdef DEBUG
      Serial.print("CLOSE_CONNECTION\n");
#endif
      state = CHECK_CLIENTQUEUE;
    }
    break;

  case READ_DATA:
    // Serial.println("READ_DATA state");
    startTime = millis();
    // while (currentClient.connected() && (millis() - startTime < 2000))
    // { // Xử lý tối đa 2 giây
      if (currentClient.available())
      {
        while (currentClient.available())
        {
          message = currentClient.readStringUntil('\n'); // Đọc dữ liệu từ client
          Serial.println("Dữ liệu nhận được: " + message);
        }
        state = PREPROCESS_DATA;
      }
      else
      {
        state = CHECK_CLIENTQUEUE;
      }
    // }
    // if (millis() - startTime >= 2000)
    // {
    //   Serial.println("Hết thời gian xử lý cho client này.");
    // }

    break;

  case PREPROCESS_DATA:
#ifdef DEBUG
    Serial.println(sBufferData.clientRecvData);
#endif

    state = PROCESS_DATA;

    break;

  case PROCESS_DATA:

    state = SEND_RESPONSE;
    break;

  case SEND_RESPONSE:
    // Serial.println("SEND_RESPONSE state");
    if (currentClient.connected())
    {
      currentClient.println("Recevived: " + message); // Gửi dữ liệu cho client
      currentClient.flush();
      state = CHECK_CLIENTQUEUE;
    }

    break;

  case CHECK_CLIENTQUEUE:
    // Serial.println("CHECK_CLIENTQUEUE state");
    // Xóa client đã xử lý xong khỏi hàng đợi
    clientQueue.erase(clientQueue.begin());
    if (currentClient.connected())
    {

      // Đưa client vào cuối hàng đợi nếu client đó vẫn còn kết nối
      clientQueue.push_back(currentClient);
    }
    else
    {
      // ngắt kết nối với client
      currentClient.stop();
      Serial.println("Client disconnected");
    }

    state = WAIT_FOR_CLIENT;

    break;

  default:
    if (server.hasClient())
    {
      server.end();
    }
    state = INIT;
    break;
  }
}