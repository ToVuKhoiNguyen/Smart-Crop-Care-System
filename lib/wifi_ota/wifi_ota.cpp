#include "wifi_ota.h"
#include "ota_index_page.h"

// Access Point credentials
const char *ssid = "ESP32_OTA_AP";
const char *password = "12345678";

// Tài khoản đăng nhập
const char *http_username = "admin";
const char *http_password = "1234";

// Biến trạng thái đăng nhập
bool isLoggedIn = false;

// Biến để quản lý session
String sessionID = "";

// Hàm tạo session ID ngẫu nhiên
String generateSessionID()
{
  String id = "";
  for (int i = 0; i < 16; i++)
  {
    id += String(random(0, 16), HEX);
  }
  return id;
}

void init_access_point(void)
{
  // Start the ESP32 as an Access Point
  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

static char flag = 0;
static uint32_t totol_size;
static uint32_t curren_download;
static char show_debug = true;

wifiota_callback_t wbegin_callback;
wifiota_callback_t wproces_callback;
wifiota_callback_t wend_callback;

void wifi_ota_set_begin_callback(wifiota_callback_t c)
{
  wbegin_callback = c;
}
void wifi_ota_set_proces_callback(wifiota_callback_t c)
{
  wproces_callback = c;
}
void wifi_ota_set_end_callback(wifiota_callback_t c)
{
  wend_callback = c;
}

void ota_set_debug(bool mode)
{
  show_debug = mode;
}

void UpdateRun()
{
  if (show_debug)
    Serial.println("OTA finished!");
  if (show_debug)
    Serial.printf("Hash in chip: %s\n", Update.md5String().c_str());
  if (Update.isFinished())
  {
    if (show_debug)
      Serial.println("Restart device!");
    ESP.restart();
  }
  else
  {
    if (show_debug)
      Serial.println("OTA not finished");
  }
}

static void handleUploadOTA(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  // Serial.println("len: " + String(len) + ", index: " + String(index) + ", final: " + String(final));
  if (!index)
  {
    if (request->hasHeader("Size"))
    {
      AsyncWebHeader *h = request->getHeader("Size");
      totol_size = h->value().toInt();
#if defined(ESP8266)
      Update.runAsync(true);
#endif
      Update.begin(totol_size);
      if (show_debug)
        Serial.println("UploadStart: " + String(totol_size));
      curren_download = 0;
      if (wbegin_callback)
        wbegin_callback(curren_download, totol_size);
    }
    else
    {
      if (show_debug)
        Serial.println(F("No Size, update fail"));
      request->send(400);
    }
  }
  // if(request->hasHeader("Extension"))
  // {
  //   AsyncWebHeader* h = request->getHeader("Extension");
  //   // if(show_debug)Serial.printf("Extension: %s\n", h->value().c_str());
  //   if(h->value() == "TMRbin")
  //   {
  //     uint8_t mask = (1 << 8) - 1;
  //     for(int i = 0; i < len; i++)
  //     {
  //       data[i] = (data[i] << 4) | (data[i] >> 4) & mask;
  //     }
  //   }
  // }

  Update.write(data, len);
  curren_download += len;
  // if(show_debug)Serial.println("Upload: " + String(curren_download) + "/" + String(totol_size));
  if (wproces_callback)
    wproces_callback(curren_download, totol_size);
  if (final)
  {
    if (totol_size == (index + len))
    {
      if (request->hasHeader("Hash"))
      {
        AsyncWebHeader *h = request->getHeader("Hash");
        Update.setMD5((const char *)h->value().c_str());
        if (show_debug)
          Serial.printf("Hash: %s\n", h->value().c_str());
        if (Update.end(true))
        {
          if (show_debug)
            Serial.printf("\r\nUploadEnd: %s, %u B\n", filename.c_str(), totol_size);
          if (wend_callback)
            wend_callback(curren_download, totol_size);
          flag = 1;
        }
        else
        {
          if (show_debug)
            Serial.println(F("MD5 hash fail !"));
          if (show_debug)
            Serial.println("Error occured #: " + String(Update.getError()));
          request->send(400, "MD5 error");
        }
      }
      else
      {
        if (show_debug)
          Serial.println(F("No Hash, update fail"));
        request->send(400);
      }
    }
    else
    {
      if (show_debug)
        Serial.println(F("Size firmware error"));
      request->send(400);
    }
  }
}
void wifi_ota_begin(AsyncWebServer *s)
{
  s->on("/uploadfw", HTTP_POST, [](AsyncWebServerRequest *request)
        { request->send(200); }, handleUploadOTA);

  // Xử lý đăng nhập
  s->on("/login", HTTP_GET, [](AsyncWebServerRequest *request)
        { request->send(200, "text/html", loginPage); });

  s->on("/login", HTTP_POST, [](AsyncWebServerRequest *request)
        {
    if (request->hasParam("username", true) && request->hasParam("password", true)) {
      String username = request->getParam("username", true)->value();
      String password = request->getParam("password", true)->value();
      if (username == http_username && password == http_password) {
        isLoggedIn = true;
        sessionID = generateSessionID();
        AsyncWebServerResponse *response = request->beginResponse(200, "text/html", R"rawliteral(
          <!DOCTYPE html>
          <html>
          <head>
            <title>Login successful</title>
            <script>
              setTimeout(() => {
                window.location.href = '/update';
              }, 1000);
            </script>
          </head>
          <body>
            <h1>Login successful!</h1>
            <p>Redirecting...</p>
          </body>
          </html>
        )rawliteral");
        response->addHeader("Set-Cookie", "session=" + sessionID + "; HttpOnly");
        request->send(response);
        request->redirect("/update");
      } else {
        request->send(200, "text/plain", "Wrong account or password!");
      }
    } else {
      request->send(400, "text/plain", "Invalid request!");
    } });

  // Xử lý cập nhật firmware
  s->on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
        {
    if(request->hasHeader("Cookie") && isLoggedIn) {
      String cookie = request->getHeader("Cookie")->value();
      if (cookie.indexOf("session=" + sessionID) != -1) {
        request->send(200, "text/html", uploadPage);
        return;
      }
    } else {
      request->redirect("/login");
    } });
}
void wifi_ota_loop()
{
  if (flag == 1)
  {
    flag = 0;
    UpdateRun();
  }
}
