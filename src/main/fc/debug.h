#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

void addDebugData(char* str);
const char* getDebugData(void);
uint8_t debugMag(uint8_t reg);
uint8_t debugMag2(uint8_t reg);