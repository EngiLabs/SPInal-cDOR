#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "config.h"

struct Frame {
    uint8_t commandID;
    uint8_t length;
    uint8_t payload[Protocol::MAX_PAYLOAD];
};

void protocolInit();
bool protocolReceive(Frame &frame);
void protocolSend(uint8_t commandID, const uint8_t *payload, uint8_t length);
uint8_t crc8(const uint8_t *data, uint8_t length);

#endif
