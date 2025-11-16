#include <SPI.h>
#include <SD.h>

#define SD_CS 5  // chân CS cho module SD

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  // Khởi tạo SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("Lỗi khi khởi tạo thẻ SD!");
    return;
  }

  Serial.println("Thẻ SD đã sẵn sàng.");

  // Mở file để đọc
  File file = SD.open("/data.txt"); // file ở gốc thẻ SD

  if (!file) {
    Serial.println("Không thể mở file!");
    return;
  }

  Serial.println("Nội dung file:");

  // Đọc từng dòng
  while (file.available()) {
    String line = file.readStringUntil('\n');
    Serial.println(line);
  }

  file.close();
}

void loop() {
  // không cần làm gì thêm trong loop
}
