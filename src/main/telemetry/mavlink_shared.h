#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "common/time.h"
#include "telemetry/mavlink.h"

#define MAVLINK_FRAME_MAX_SIZE 280
#define CRSF_MAVLINK_BUFFER_SIZE MAVLINK_FRAME_MAX_SIZE
#define CRSF_FRAME_TX_MAVLINK_FRAME_SIZE = 32
#define CRSF_FRAME_RX_MAVLINK_FRAME_SIZE = 16
#define MAVLINK_BUFFER_LENGTH_OFFSET 1

typedef struct mavlinkBuffer_s {
    uint8_t bytes[CRSF_MAVLINK_BUFFER_SIZE];
    int len;
} mavlinkBuffer_s;

bool bufferCrsfMavlinkFrame(uint8_t *frameStart, int frameLength);
bool handleMavlinkFrame(uint8_t *frameStart, int frameLength);
bool sendMavlinkReply(uint8_t payloadSize, mspResponseFnPtr responseFn);