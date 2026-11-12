#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

struct LegAngles { // radians
    float phi, alpha, beta;
};

struct FootPosition { // mm
    float x, y, z;
};

enum class IkStatus : uint8_t {
    OK = 0,
    OUT_OF_REACH,
    BETA_LIMIT,
    ALPHA_LIMIT,
    PHI_LIMIT
};

const char* ikStatusText(IkStatus st);

bool forwardKinematics(const LegAngles &angles, FootPosition &pos);
IkStatus inverseKinematics(const FootPosition &pos, LegAngles &angles);
IkStatus solvePlanar(float R, float y, float &alpha, float &beta);
void kinematicsGetHomePosition(FootPosition &pos);

#endif
