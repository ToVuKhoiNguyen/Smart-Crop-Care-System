#include "Arduino.h"

// Khai báo chân nút
#define btn_1 36
#define btn_2 39
#define btn_3 34
#define btn_4 35
#define btn_5 32

// Khai báo chân relay
#define relay_1 25
#define relay_2 26
#define relay_3 27
#define relay_4 14
#define relay_5 13

// Biến trạng thái relay
volatile bool relayState[5] = {false, false, false, false, false};

// Hàm xử lý ngắt
void IRAM_ATTR handleBtn1() {
  relayState[0] = !relayState[0];
  digitalWrite(relay_1, relayState[0]);
}

void IRAM_ATTR handleBtn2() {
  relayState[1] = !relayState[1];
  digitalWrite(relay_2, relayState[1]);
}

void IRAM_ATTR handleBtn3() {
  relayState[2] = !relayState[2];
  digitalWrite(relay_3, relayState[2]);
}

void IRAM_ATTR handleBtn4() {
  relayState[3] = !relayState[3];
  digitalWrite(relay_4, relayState[3]);
}

void IRAM_ATTR handleBtn5() {
  relayState[4] = !relayState[4];
  digitalWrite(relay_5, relayState[4]);
}

void setup() {
  Serial.begin(115200);

  // Khai báo output
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  pinMode(relay_5, OUTPUT);

  // Khai báo input (không cần PULLUP vì đã dùng schmitt bên ngoài)
  pinMode(btn_1, INPUT);
  pinMode(btn_2, INPUT);
  pinMode(btn_3, INPUT);
  pinMode(btn_4, INPUT);
  pinMode(btn_5, INPUT);

  // Gán ngắt
  attachInterrupt(digitalPinToInterrupt(btn_1), handleBtn1, FALLING);
  attachInterrupt(digitalPinToInterrupt(btn_2), handleBtn2, FALLING);
  attachInterrupt(digitalPinToInterrupt(btn_3), handleBtn3, FALLING);
  attachInterrupt(digitalPinToInterrupt(btn_4), handleBtn4, FALLING);
  attachInterrupt(digitalPinToInterrupt(btn_5), handleBtn5, FALLING);
}

void loop() {
  // Không cần làm gì trong loop
}
