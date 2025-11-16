# wifi_ota
Thư viện upload firmware ota cho esp32/esp8266 espidf qua webserver
Người dùng kết nối tới wifi do esp32 phát ra rồi truy cập vào địa chỉ 192.168.4.1/update để update firmware

# Sử dụng
Thư viện này yêu cầu cài ESPAsyncWebServer.h

# Add thư viện
```
#include "iot47_wifi_ota.h"
```
# Đặt hàm này vào setup:
```
WiFi.softAP(ssid, password); # ESP32 ở chế độ Access Point
wifi_ota_begin(&server);
server.begin();
```
# Đặt hàm này vào loop:
```
wifi_ota_loop();
```
Để vào trang upload firmware, vào trình duyệt gõ 192.168.4.1/update (192.168.4.1 là ip tùy chỉnh )


