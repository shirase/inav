#include "mavlink_shared.h"

static mavlinkBuffer_s mavlinkRxBuffer;
static uint8_t mavlinkOutputData[CRSF_MAVLINK_BUFFER_SIZE];

bool bufferCrsfMavlinkFrame(uint8_t *frameStart, int frameLength)
{
    if (mavlinkRxBuffer.len + MAVLINK_BUFFER_LENGTH_OFFSET + frameLength > CRSF_MAVLINK_BUFFER_SIZE) {
        return false;
    } else {
        uint8_t *p = mavlinkRxBuffer.bytes + mavlinkRxBuffer.len;
        *p++ = frameLength;
        memcpy(p, frameStart, frameLength);
        mavlinkRxBuffer.len += MAVLINK_BUFFER_LENGTH_OFFSET + frameLength;
        return true;
    }
}

bool handleMavlinkFrame(uint8_t *frameStart, int frameLength)
{
    sbuf_t mavlinkOutputBuffer;
    sbuf_t *mavlinkOutputBuf = sbufInit(&mavlinkOutputBuffer, mavlinkOutputData, mavlinkOutputData + sizeof mavlinkOutputData);

    uint8_t *frameStart = (uint8_t *)&crsfFrame.frame.payload;
    foreach (uint8_t i = 0; i < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE - CRSF_FRAME_LENGTH_CRC; i++) {
        if (parseCharMAVLinkIncomingTelemetry((char)frameStart[i])) {
            setMavlinkSendMessageOutput(mavlinkOutputBuf);
            handleMAVLinkIncomingTelemetry();
            setMavlinkSendMessageOutput(NULL);
            sbufSwitchToReader(mavlinkOutputBuf, mavlinkOutputData);
            return true;
        }
    }

    return false;
}

bool sendMavlinkReply(uint8_t payloadSize, mspResponseFnPtr responseFn)
{
    uint8_t payloadOut[payloadSize];

    responseFn(payloadOut);
    return false;
}