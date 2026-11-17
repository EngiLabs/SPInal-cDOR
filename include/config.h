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

    constexpr uint8_t BATTERY_ADC  = A0;

    constexpr uint8_t SERVO_PIN_RF_ALPHA = 2,
                      SERVO_PIN_LF_ALPHA = 3,
                      SERVO_PIN_RB_ALPHA = 4,
                      SERVO_PIN_LB_ALPHA = 5;

    constexpr uint8_t SERVO_SHIELD_RF_PHI  = 0,
                      SERVO_SHIELD_RF_BETA = 1,
                      SERVO_SHIELD_LF_PHI  = 2,
                      SERVO_SHIELD_LF_BETA = 3,
                      SERVO_SHIELD_RB_PHI  = 4,
                      SERVO_SHIELD_RB_BETA = 5,
                      SERVO_SHIELD_LB_PHI  = 6,
                      SERVO_SHIELD_LB_BETA = 7;
}

namespace Kinematics {
    constexpr float L1 = 70.0f, // #hip_length = 70 mm
                    L2 = 70.0f; // #shin_length = 70 mm

    // Угловые пределы
    constexpr float C_MIN     = -0.70710678f,                           // -1/sqrt2 (cos 135 deg)
                    ALPHA_MIN = -45.0f * 3.14159265f / 180.0f,          // -45 deg (#hip_lower_limit = 45 deg)
                    ALPHA_MAX =  45.0f * 3.14159265f / 180.0f,          // +45 deg (расширено с 30 для отработки подъема LIFT_HEIGHT=40мм)
                    BETA_MIN  =  0.0f,
                    BETA_MAX  =  3.0f * 3.14159265f / 4.0f,             // 3pi/4 (135 deg)
                    PHI_MIN   = -3.14159265f / 3.0f,                    // -pi/3 (-60 deg)
                    PHI_MAX   =  3.14159265f / 3.0f,                    //  pi/3 (+60 deg)
                    EPS       = 1e-4f;

    // опорная точка стойки (мм)
    constexpr float HOME_R = 85.0f;
    constexpr float HOME_Y = -55.0f;
}

namespace Servos {
    constexpr uint8_t COUNT       = 12,
                      LEGS        = 4,
                      DOF_PER_LEG = 3;

    enum Joint : uint8_t {
        JOINT_PHI   = 0,
        JOINT_ALPHA = 1,
        JOINT_BETA  = 2
    };

    enum Leg : uint8_t {
        LEG_RF = 0,
        LEG_LF = 1,
        LEG_RB = 2,
        LEG_LB = 3
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

    struct Calibration {
        int zero;
        int dir;
        int minAngle;
        int maxAngle;
    };

    constexpr Calibration
    DEFAULT_ALPHA = {
        .zero     = ALPHA_ZERO,
        .dir      = ALPHA_DIR,
        .minAngle = ALPHA_SERVO_MIN,
        .maxAngle = ALPHA_SERVO_MAX
    },
    DEFAULT_BETA = {
        .zero     = BETA_ZERO,
        .dir      = BETA_DIR,
        .minAngle = BETA_SERVO_MIN,
        .maxAngle = BETA_SERVO_MAX
    },
    DEFAULT_PHI = {
        .zero     = PHI_ZERO,
        .dir      = PHI_DIR,
        .minAngle = PHI_SERVO_MIN,
        .maxAngle = PHI_SERVO_MAX
    };

    constexpr Calibration CALIB_RF_ALPHA = DEFAULT_ALPHA;
    constexpr Calibration CALIB_RF_BETA  = DEFAULT_BETA;
    constexpr Calibration CALIB_RF_PHI   = DEFAULT_PHI;

    constexpr Calibration CALIB_LF_ALPHA = DEFAULT_ALPHA;
    constexpr Calibration CALIB_LF_BETA  = DEFAULT_BETA;
    constexpr Calibration CALIB_LF_PHI   = {
        .zero     = PHI_ZERO,
        .dir      = -PHI_DIR,
        .minAngle = PHI_SERVO_MIN,
        .maxAngle = PHI_SERVO_MAX
    };

    constexpr Calibration CALIB_RB_ALPHA = DEFAULT_ALPHA;
    constexpr Calibration CALIB_RB_BETA  = DEFAULT_BETA;
    constexpr Calibration CALIB_RB_PHI   = DEFAULT_PHI;

    constexpr Calibration CALIB_LB_ALPHA = DEFAULT_ALPHA;
    constexpr Calibration CALIB_LB_BETA  = DEFAULT_BETA;
    constexpr Calibration CALIB_LB_PHI   = {
        .zero     = PHI_ZERO,
        .dir      = -PHI_DIR,
        .minAngle = PHI_SERVO_MIN,
        .maxAngle = PHI_SERVO_MAX
    };
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
    constexpr float CYCLE_LENGTH_TICK = 50.0f,
                    LIFT_HEIGHT       = 40.f,  // мм
                    MAX_STRIDE        = 80.f;  // мм
}

namespace Imu {
    constexpr uint8_t MPU6050_ADDR = 0x68;
}

namespace Timing {
    constexpr uint16_t TELEMETRY_INTERVAL_MS = 100,
                       SUBOPERATION_TICK     = 20;
}

#endif
