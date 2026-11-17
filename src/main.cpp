#include "config.h"
#include "protocol.h"
#include "servo_driver.h"
#include "imu_reader.h"
#include "ir_sensors.h"
#include "gait_controller.h"
// #include "led_controller.h"

#include <Arduino.h>


static unsigned long last_suboperation_tick = 0;
static bool telemetryRequested = false;

static ServoDriver::Driver servoDriver;
static GaitController::GaitController gaitController(servoDriver);

static void handleCommand(const Frame &frame) {
    switch (frame.commandID) {
        case Protocol::MOVE: {
            const auto direction = static_cast<int8_t>(frame.payload[0]);
            const uint8_t speed = frame.payload[1];
            gaitController.setDirection(direction, speed);
            break;
        }

        default:
        case Protocol::STOP:
            gaitController.stop();
            break;

        case Protocol::SET_GAIT:
            gaitController.setType(frame.payload[0]);
            break;

        case Protocol::SET_POSE: {
            float angles[Servos::COUNT];
            for (uint8_t i = 0; i < Servos::COUNT && i * 2 + 1 < frame.length; i++)
                angles[i] = static_cast<float>(
                    (frame.payload[i * 2] << 8) | frame.payload[i * 2 + 1]
                ) / 100.0f;
            servoDriver.setAllAngles(angles);
            break;
        }

        case Protocol::REQUEST_TELEMETRY:
            telemetryRequested = true;
            break;

        case Protocol::SET_LED:
            break;
        /*{
            const auto mode = static_cast<LedMode>(frame.payload[0]);
            const uint8_t r = frame.payload[1];
            const uint8_t g = frame.payload[2];
            const uint8_t b = frame.payload[3];
            ledSetMode(mode, r, g, b);
            break;
        }*/

        case Protocol::HOME:
            gaitController.stop();
            servoDriver.home();
            break;

        case Protocol::RELAX:
            gaitController.stop();
            servoDriver.relaxAll();
            break;

        case Protocol::WAKE:
            servoDriver.wakeAll();
            break;
    }
}

static void sendTelemetry() {
    ImuData imu{};
    IrReadings ir{};
    imuRead(imu);
    irSensorsRead(ir);

    const uint16_t batteryRaw = analogRead(Pin::BATTERY_ADC);

    uint8_t payload[27];
    memcpy(payload + 0,  &imu.accelX, 4);
    memcpy(payload + 4,  &imu.accelY, 4);
    memcpy(payload + 8,  &imu.accelZ, 4);
    memcpy(payload + 12, &imu.gyroX, 4);
    memcpy(payload + 16, &imu.gyroY, 4);
    memcpy(payload + 20, &imu.gyroZ, 4);
    payload[24] = (ir.LF >> 8) & 0xFF;
    payload[25] = ir.LF & 0xFF;

    payload[26] = (batteryRaw >> 8) & 0xFF;

    protocolSend(Protocol::REQUEST_TELEMETRY, payload, 27);
}


void setup() {
    protocolInit();
    imuInit();
    irSensorsInit();
    // ledInit();

    servoDriver.home();
}


void loop() {
    const unsigned long now = millis();

    Frame frame{};
    if (protocolReceive(frame))
        handleCommand(frame);

    if (now - last_suboperation_tick >= Timing::SUBOPERATION_TICK) {
        last_suboperation_tick = now;
        gaitController.tick();
        // ledTick();
    }

    if (telemetryRequested) {
        telemetryRequested = false;
        sendTelemetry();
    }
}
