#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include "stdint.h"

#define LORA_CS 5
#define LORA_RST 4
#define LORA_DIO0 2

void init_LoRa();
void Transmit_Cmd(uint8_t relay, uint8_t value);
void run_LoRa();

#endif