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

    constexpr uint8_t SERVO_RF_ALPHA = 7,
                      SERVO_RF_PHI   = 9,
                      SERVO_RF_BETA  = 11;
    constexpr uint8_t SERVO_LF_ALPHA = 6,
                      SERVO_LF_PHI   = 8,
                      SERVO_LF_BETA  = 10;
    constexpr uint8_t SERVO_RB_ALPHA = 17,
                      SERVO_RB_PHI   = 15,
                      SERVO_RB_BETA  = 13;
    constexpr uint8_t SERVO_LB_ALPHA = 0,
                      SERVO_LB_PHI   = 2,
                      SERVO_LB_BETA  = 4;
}

namespace Kinematics {
    constexpr float L1 = 75.0f,  // hip (mm)
                    L2 = 75.0f;  // shin (mm)

    // angle limits
    constexpr float C_MIN     = -0.70710678f,  // -1/sqrt2 (cos 135 deg)
                    PI        =  3.14159265f,  // raspberry PI
                    ALPHA_MIN = -.25f * PI,    // -45 deg
                    ALPHA_MAX =  .25f * PI,    // +45 deg
                    BETA_MIN  = -.25f * PI,    // -45 deg
                    BETA_MAX  =  .50f * PI,    // +90 deg
                    PHI_MIN   = -.25f * PI,    // -45 deg
                    PHI_MAX   =  .25f * PI,    // +45 deg
                    EPS       =  1e-4f;

    // servo base position
    constexpr float HOME_R =  90.f;  // mm
    constexpr float HOME_Y = -85.f;  // mm
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

    constexpr int ALPHA_SERVO_MIN = 36,
                  ALPHA_SERVO_MAX = 156,
                  BETA_SERVO_MIN  = 18,
                  BETA_SERVO_MAX  = 168,
                  PHI_SERVO_MIN   = 0,
                  PHI_SERVO_MAX   = 180;

    struct Calibration {
        int zero;       // servo's initial position
        int dir;        // direction of rotating
        int minAngle;   // lower limit
        int maxAngle;   // upper limit
    };

    constexpr Calibration CALIB_RB_ALPHA = {
                              .zero     = 90,
                              .dir      = 1,
                              .minAngle = ALPHA_SERVO_MIN,
                              .maxAngle = ALPHA_SERVO_MAX
                          },
                          CALIB_RB_BETA  = {
                              .zero     = 91,
                              .dir      = -1,
                              .minAngle = BETA_SERVO_MIN,
                              .maxAngle = BETA_SERVO_MAX
                          },
                          CALIB_RB_PHI   = {
                              .zero     = 45,
                              .dir      = -1,
                              .minAngle = PHI_SERVO_MIN,
                              .maxAngle = PHI_SERVO_MAX
                          };
    constexpr Calibration CALIB_LB_ALPHA = {
                              .zero     = 96,
                              .dir      = 1,
                              .minAngle = ALPHA_SERVO_MIN,
                              .maxAngle = ALPHA_SERVO_MAX
                          },
                          CALIB_LB_BETA  = {
                              .zero     = 66,
                              .dir      = 1,
                              .minAngle = BETA_SERVO_MIN,
                              .maxAngle = BETA_SERVO_MAX
                          },
                          CALIB_LB_PHI   = {
                              .zero     = 45,
                              .dir      = 1,
                              .minAngle = PHI_SERVO_MIN,
                              .maxAngle = PHI_SERVO_MAX
                          };
    constexpr Calibration CALIB_LF_ALPHA = {
                              .zero     = 69,
                              .dir      = 1,
                              .minAngle = ALPHA_SERVO_MIN,
                              .maxAngle = ALPHA_SERVO_MAX
                          },
                          CALIB_LF_BETA  = {
                              .zero     = 98,
                              .dir      = 1,
                              .minAngle = BETA_SERVO_MIN,
                              .maxAngle = BETA_SERVO_MAX
                          },
                          CALIB_LF_PHI   = {
                              .zero     = 45,
                              .dir      = 1,
                              .minAngle = PHI_SERVO_MIN,
                              .maxAngle = PHI_SERVO_MAX
                          };
    constexpr Calibration CALIB_RF_ALPHA = {
                              .zero     = 96,
                              .dir      = 1,
                              .minAngle = ALPHA_SERVO_MIN,
                              .maxAngle = ALPHA_SERVO_MAX
                          },
                          CALIB_RF_BETA  = {
                              .zero     = 104,
                              .dir      = -1,
                              .minAngle = BETA_SERVO_MIN,
                              .maxAngle = BETA_SERVO_MAX
                          },
                          CALIB_RF_PHI   = {
                              .zero     = 45,
                              .dir      = -1,
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
    constexpr float CYCLE_LENGTH_TICK = 50.0f, // ms
                    LIFT_HEIGHT       = 40.f,  // mm
                    MAX_STRIDE        = 80.f;  // mm
}

namespace Imu {
    constexpr uint8_t MPU6050_ADDR = 0x68;
}

namespace Timing {
    constexpr uint16_t TELEMETRY_INTERVAL = 100,  // ms
                       SUBOPERATION_TICK  = 20;   // ms
}

#endif
