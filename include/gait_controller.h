#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include "config.h"
#include "servo_driver.h"
#include <stdint.h>

namespace GaitController {
    class GaitController {
    public:
        explicit GaitController(ServoDriver::Driver &driver);

        void setType(uint8_t type);
        void setDirection(int8_t direction, uint8_t speed);

        void stop();
        void tick();

        bool isActive() const;
    private:
        ServoDriver::Driver &servoDriver;

        int8_t currentDirection = 0;
        uint8_t currentSpeed = 0;
        bool active = false;
        uint32_t tickCounter = 0;

        float swingRatio = 0.5f;
        float legPhaseOffsets[Servos::LEGS] = {0.25f, 0.75f, 0.0f, 0.5f};
    };
}

#endif
