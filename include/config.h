#ifndef CONFIG_H
#define CONFIG_H

#include <pins_arduino.h>
#include <stdint.h>


namespace Pin {
    constexpr uint8_t IR_LF = A0,
                      IR_RF = A1,
                      IR_LB = A2,
                      IR_RB = A3;

    constexpr uint8_t MAX7219_DATA = 11,
                      MAX7219_CLK  = 13,
                      MAX7219_CS   = 10;

    constexpr uint8_t BATTERY_ADC  = A6;

    // Multiservo Shield pin mapping
    // Leg order: 0=LF, 1=RF, 2=LB, 3=RB
    constexpr uint8_t SERVO_PINS[4][3] = {
        { 0,  1,  2 },   // Front-Left:  Phi=0, Alpha=1, Beta=2
        { 3,  4,  5 },   // Front-Right: Phi=3, Alpha=4, Beta=5
        { 6,  7,  8 },   // Rear-Left:   Phi=6, Alpha=7, Beta=8
        { 9, 10, 11 }    // Rear-Right:  Phi=9, Alpha=10, Beta=11
    };
}

namespace Kinematics {
    constexpr float L1 = 60.0f,   // длина бедра (мм)
                    L2 = 90.0f;   // длина голени (мм)

    // Пределы из аналитического вывода
    constexpr float C_MIN     = -0.70710678f,               // -1/sqrt2
                    ALPHA_MIN = -3.14159265f / 2.0f,        // -pi/2
                    ALPHA_MAX =  3.14159265f / 2.0f,        //  pi/2
                    BETA_MIN  =  0.0f,
                    BETA_MAX  =  3.0f * 3.14159265f / 4.0f, // 3pi/4
                    PHI_MIN   = -3.14159265f / 3.0f,        // -pi/3
                    PHI_MAX   =  3.14159265f / 3.0f,        //  pi/3
                    ANG_EPS   = 1e-4f;

    // Опорная точка стойки (мм)
    constexpr float HOME_R = 85.0f;
    constexpr float HOME_Y = -55.0f;
}

namespace Servo {
    constexpr uint8_t COUNT       = 12,
                      LEGS        = 4,
                      DOF_PER_LEG = 3;

    enum Joint : uint8_t {
        JOINT_PHI   = 0,
        JOINT_ALPHA = 1,
        JOINT_BETA  = 2
    };

    enum Leg : uint8_t {
        LEG_LF = 0,
        LEG_RF = 1,
        LEG_LB = 2,
        LEG_RB = 3
    };

    constexpr int ALPHA_ZERO = 96,
                  ALPHA_DIR  = 1,

                  BETA_ZERO  = 138,
                  BETA_DIR   = -1,

                  PHI_ZERO   = 90,
                  PHI_DIR    = 1;

    constexpr int ALPHA_SERVO_MIN = 36,
                  ALPHA_SERVO_MAX = 156,
                  BETA_SERVO_MIN  = 18,
                  BETA_SERVO_MAX  = 168,
                  PHI_SERVO_MIN   = 0,
                  PHI_SERVO_MAX   = 180;
}

namespace Protocol {
    constexpr uint8_t  HEADER_0    = 0xAA;
    constexpr uint8_t  HEADER_1    = 0x55;
    constexpr uint32_t BAUD_RATE   = 115200;
    constexpr uint8_t  MAX_PAYLOAD = 64;

    enum CommandID : uint8_t {
        MOVE              = 0x01,
        STOP              = 0x02,
        SET_GAIT          = 0x03,
        SET_POSE          = 0x04,
        REQUEST_TELEMETRY = 0x05,
        SET_LED           = 0x06,
        HOME              = 0x07,
        RELAX             = 0x08,
        WAKE              = 0x09
    };
}

namespace Gait {
    enum Type : uint8_t {
        WALK  = 0,
        TROT  = 1,
        CREEP = 2
    };
}

namespace Imu {
    constexpr uint8_t MPU6050_ADDR = 0x68;
}

namespace Timing {
    constexpr uint16_t TELEMETRY_INTERVAL_MS = 100;
    constexpr uint16_t GAIT_TICK_MS          = 20;
}

#endif
