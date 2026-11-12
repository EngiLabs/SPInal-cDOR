#include "protocol.h"
#include "config.h"
#include <Arduino.h>

namespace {
    enum class RxState : uint8_t {
        WAIT_HEADER_0,
        WAIT_HEADER_1,
        WAIT_LENGTH,
        WAIT_COMMAND,
        WAIT_PAYLOAD,
        WAIT_CRC
    };
}

static auto    rxState = RxState::WAIT_HEADER_0;
static uint8_t rxBuffer[Protocol::MAX_PAYLOAD];
static uint8_t rxLength  = 0,
               rxCommand = 0,
               rxIndex   = 0;

uint8_t crc8(const uint8_t *data, const uint8_t length) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void protocolInit() {
    Serial.begin(Protocol::BAUD_RATE);
}

bool protocolReceive(Frame &frame) {
    while (Serial.available()) {
        const uint8_t byte = Serial.read();

        switch (rxState) {
            case RxState::WAIT_HEADER_0:
                if (byte == Protocol::HEADER_0) rxState = RxState::WAIT_HEADER_1;
                break;

            case RxState::WAIT_HEADER_1:
                rxState = byte == Protocol::HEADER_1 ? RxState::WAIT_LENGTH : RxState::WAIT_HEADER_0;
                break;

            case RxState::WAIT_LENGTH:
                rxLength = byte;
                rxState = RxState::WAIT_COMMAND;
                break;

            case RxState::WAIT_COMMAND:
                rxCommand = byte;
                rxIndex = 0;
                rxState = rxLength > 1 ? RxState::WAIT_PAYLOAD : RxState::WAIT_CRC;
                break;

            case RxState::WAIT_PAYLOAD:
                if (rxIndex < Protocol::MAX_PAYLOAD) rxBuffer[rxIndex++] = byte;
                if (rxIndex >= rxLength - 1) rxState = RxState::WAIT_CRC;
                break;

            case RxState::WAIT_CRC: {
                uint8_t checkBuf[Protocol::MAX_PAYLOAD + 2];
                checkBuf[0] = rxLength;
                checkBuf[1] = rxCommand;
                memcpy(checkBuf + 2, rxBuffer, rxIndex);
                const uint8_t expected = crc8(checkBuf, rxIndex + 2);

                if (byte == expected) {
                    frame.commandID = rxCommand;
                    frame.length = rxIndex;
                    memcpy(frame.payload, rxBuffer, rxIndex);
                    rxState = RxState::WAIT_HEADER_0;
                    return true;
                }
                rxState = RxState::WAIT_HEADER_0;
                break;
            }
        }
    }
    return false;
}

void protocolSend(const uint8_t commandID, const uint8_t *payload, const uint8_t length) {
    const uint8_t totalLength = length + 1;

    Serial.write(Protocol::HEADER_0);
    Serial.write(Protocol::HEADER_1);
    Serial.write(totalLength);
    Serial.write(commandID);
    Serial.write(payload, length);

    uint8_t checkBuf[Protocol::MAX_PAYLOAD + 2];
    checkBuf[0] = totalLength;
    checkBuf[1] = commandID;
    memcpy(checkBuf + 2, payload, length);
    Serial.write(crc8(checkBuf, length + 2));
}
