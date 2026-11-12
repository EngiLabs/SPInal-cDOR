#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>
#include "config.h"
#include "kinematics.h"
#include "relaxable_servo.h"

namespace ServoDriver {
    class Driver {
    public:
        Driver();

        bool setLegAngles(uint8_t legIndex, const LegAngles &angles);
        bool setLegPosition(uint8_t legIndex, float x, float y, float z);
        bool setLegPlanar(uint8_t legIndex, float R, float y);

        void setAllAngles(const float angles[12]);

        void relax(uint8_t legIndex);
        void relaxAll();

        void wake(uint8_t legIndex);
        void wakeAll();

        void home();
    protected:
        RelaxableServo servos[Servo::LEGS][Servo::DOF_PER_LEG]{};
    };
}

#endif
