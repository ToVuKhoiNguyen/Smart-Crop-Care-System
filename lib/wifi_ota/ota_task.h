#ifndef OTA_TASK_H
#define OTA_TASK_H

#include <Arduino.h>
#include <WiFi.h>

#include <iostream>
#include <cstring>

// #include "definition.h"

void OTA_Init();
void OTA_Task(void *parameter);

#endif // OTA_TASK_H