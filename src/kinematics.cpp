#include "kinematics.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>


const char* ikStatusText(const IkStatus st) {
    switch (st) {
        case IkStatus::OK:           return "OK";
        case IkStatus::OUT_OF_REACH: return "out of reach (|C| > 1)";
        case IkStatus::BETA_LIMIT:   return "beta > 3pi/4";
        case IkStatus::ALPHA_LIMIT:  return "alpha out of [-pi/2; pi/2]";
        case IkStatus::PHI_LIMIT:    return "phi out of [-pi/3; pi/3]";
    }
    return "?";
}

bool forwardKinematics(const LegAngles &angles, FootPosition &pos) {
    const float x_prime = Kinematics::L1 * cos(angles.alpha) +
                          Kinematics::L2 * cos(angles.alpha - angles.beta);

    pos.y = Kinematics::L1 * sin(angles.alpha) +
            Kinematics::L2 * sin(angles.alpha - angles.beta);
    pos.x = x_prime * sin(angles.phi);
    pos.z = x_prime * cos(angles.phi);

    return true;
}

IkStatus solvePlanar(const float R, const float y, float &alpha, float &beta) {
    float C = (R * R + y * y - Kinematics::L1 * Kinematics::L1 - Kinematics::L2 * Kinematics::L2) /
              (2.0f * Kinematics::L1 * Kinematics::L2);

    if (C > 1.0f + Kinematics::EPS || C < -1.0f - Kinematics::EPS)
        return IkStatus::OUT_OF_REACH;
    C = constrain(C, -1.0f, 1.0f);

    if (C < Kinematics::C_MIN - Kinematics::EPS) {
        return IkStatus::BETA_LIMIT;
    }

    beta = acos(C);
    alpha = atan2(y, R) + atan2(Kinematics::L2 * sin(beta), Kinematics::L1 + Kinematics::L2 * cos(beta));

    if (alpha < Kinematics::ALPHA_MIN - Kinematics::EPS ||
        alpha > Kinematics::ALPHA_MAX + Kinematics::EPS)
        return IkStatus::ALPHA_LIMIT;

    return IkStatus::OK;
}

static int lastS = 1;
static float lastPhi = 0.0f;

IkStatus inverseKinematics(const FootPosition &pos, LegAngles &angles) {
    constexpr float EPS = 1e-6f;
    const float rho = sqrt(pos.x * pos.x + pos.z * pos.z);

    int s;
    if (pos.z > EPS)       s = 1;
    else if (pos.z < -EPS) s = -1;
    else                   s = lastS;

    float R, phi;
    if (rho < EPS) {
        R = 0.0f;
        phi = lastPhi;
    }
    else {
        R = s * rho;
        phi = s > 0 ? atan2(pos.x, pos.z) : atan2(-pos.x, -pos.z);
    }

    if (phi < Kinematics::PHI_MIN - Kinematics::EPS ||
        phi > Kinematics::PHI_MAX + Kinematics::EPS)
        return IkStatus::PHI_LIMIT;

    float alpha, beta;
    const IkStatus st = solvePlanar(R, pos.y, alpha, beta);
    if (st != IkStatus::OK) return st;

    angles.phi = phi;
    angles.alpha = alpha;
    angles.beta = beta;
    lastS = s;
    lastPhi = phi;

    return IkStatus::OK;
}

void kinematicsGetHomePosition(FootPosition &pos) {
    pos.x = 0.0f;
    pos.y = Kinematics::HOME_Y;
    pos.z = Kinematics::HOME_R;
}
