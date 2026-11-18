#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>
#include "config.h"
#include "kinematics.h"
#include "relaxable_servo.h"

namespace ServoDriver {
    struct LegServos {
        RelaxableShieldServo phi, beta;
        RelaxableShieldServo alpha;

        Servos::Calibration calibPhi{},
                            calibAlpha{},
                            calibBeta{};

        bool writeAngles(float phiRad, float alphaRad, float betaRad);
        void relax();
        void wake();
    };

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

        LegServos* getLeg(uint8_t legIndex);

        LegServos RF;
        LegServos LF;
        LegServos RB;
        LegServos LB;

        RelaxableShieldServo &RF_alpha;
        RelaxableShieldServo   &RF_beta;
        RelaxableShieldServo   &RF_phi;
        RelaxableShieldServo &LF_alpha;
        RelaxableShieldServo   &LF_beta;
        RelaxableShieldServo   &LF_phi;
        RelaxableShieldServo &RB_alpha;
        RelaxableShieldServo   &RB_beta;
        RelaxableShieldServo   &RB_phi;
        RelaxableShieldServo &LB_alpha;
        RelaxableShieldServo   &LB_beta;
        RelaxableShieldServo   &LB_phi;
    };
}

#endif
