#include "servo_driver.h"

#include <math.h>


static int to_servo_angle(const float rad, const int zero, const int direction) {
    return zero + direction * static_cast<int>(lround(rad * 180.0f / 3.14159265f));
}

static bool in_range(const int val, const int lo, const int hi) {
    return val >= lo && val <= hi;
}


bool ServoDriver::LegServos::writeAngles(const float phiRad, const float alphaRad, const float betaRad) {
    const int phiAngle   = to_servo_angle(phiRad,   calibPhi.zero,   calibPhi.dir);
    const int alphaAngle = to_servo_angle(alphaRad, calibAlpha.zero, calibAlpha.dir);
    const int betaAngle  = to_servo_angle(betaRad,  calibBeta.zero,  calibBeta.dir);

    if (!in_range(phiAngle,   calibPhi.minAngle,   calibPhi.maxAngle) ||
        !in_range(alphaAngle, calibAlpha.minAngle, calibAlpha.maxAngle) ||
        !in_range(betaAngle,  calibBeta.minAngle,  calibBeta.maxAngle)) {
        return false;
    }

    phi.write(phiAngle);
    alpha.write(alphaAngle);
    beta.write(betaAngle);
    return true;
}

void ServoDriver::LegServos::relax() {
    phi.relax();
    alpha.relax();
    beta.relax();
}

void ServoDriver::LegServos::wake() {
    phi.wake();
    alpha.wake();
    beta.wake();
}


ServoDriver::Driver::Driver()
    : RF_alpha(RF.alpha), RF_beta(RF.beta), RF_phi(RF.phi),
      LF_alpha(LF.alpha), LF_beta(LF.beta), LF_phi(LF.phi),
      RB_alpha(RB.alpha), RB_beta(RB.beta), RB_phi(RB.phi),
      LB_alpha(LB.alpha), LB_beta(LB.beta), LB_phi(LB.phi)
{
    RF.calibAlpha = Servos::CALIB_RF_ALPHA;
    RF.calibBeta  = Servos::CALIB_RF_BETA;
    RF.calibPhi   = Servos::CALIB_RF_PHI;
    LF.calibAlpha = Servos::CALIB_LF_ALPHA;
    LF.calibBeta  = Servos::CALIB_LF_BETA;
    LF.calibPhi   = Servos::CALIB_LF_PHI;
    RB.calibAlpha = Servos::CALIB_RB_ALPHA;
    RB.calibBeta  = Servos::CALIB_RB_BETA;
    RB.calibPhi   = Servos::CALIB_RB_PHI;
    LB.calibAlpha = Servos::CALIB_LB_ALPHA;
    LB.calibBeta  = Servos::CALIB_LB_BETA;
    LB.calibPhi   = Servos::CALIB_LB_PHI;

    RF_alpha.attach(Pin::SERVO_PIN_RF_ALPHA);
    LF_alpha.attach(Pin::SERVO_PIN_LF_ALPHA);
    RB_alpha.attach(Pin::SERVO_PIN_RB_ALPHA);
    LB_alpha.attach(Pin::SERVO_PIN_LB_ALPHA);

    RF_phi.attach(Pin::SERVO_SHIELD_RF_PHI);
    RF_beta.attach(Pin::SERVO_SHIELD_RF_BETA);
    LF_phi.attach(Pin::SERVO_SHIELD_LF_PHI);
    LF_beta.attach(Pin::SERVO_SHIELD_LF_BETA);
    RB_phi.attach(Pin::SERVO_SHIELD_RB_PHI);
    RB_beta.attach(Pin::SERVO_SHIELD_RB_BETA);
    LB_phi.attach(Pin::SERVO_SHIELD_LB_PHI);
    LB_beta.attach(Pin::SERVO_SHIELD_LB_BETA);
}

ServoDriver::LegServos* ServoDriver::Driver::getLeg(const uint8_t legIndex) {
    switch (legIndex) {
        case Servos::LEG_RF: return &RF;
        case Servos::LEG_LF: return &LF;
        case Servos::LEG_RB: return &RB;
        case Servos::LEG_LB: return &LB;
        default: return nullptr;
    }
}

bool ServoDriver::Driver::setLegAngles(const uint8_t legIndex, const LegAngles &angles) {
    LegServos* leg = getLeg(legIndex);
    if (!leg) return false;
    return leg->writeAngles(angles.phi, angles.alpha, angles.beta);
}

bool ServoDriver::Driver::setLegPosition(const uint8_t legIndex, const float x, const float y, const float z) {
    const FootPosition pos = { .x = x, .y = y, .z = z };
    LegAngles angles{};
    if (inverseKinematics(pos, angles) != IkStatus::OK) return false;
    return setLegAngles(legIndex, angles);
}

bool ServoDriver::Driver::setLegPlanar(const uint8_t legIndex, const float R, const float y) {
    float alpha, beta;
    if (solvePlanar(R, y, alpha, beta) != IkStatus::OK) return false;
    return setLegAngles(legIndex, {
        .phi = 0.0f,
        .alpha = alpha,
        .beta = beta
    });
}

void ServoDriver::Driver::setAllAngles(const float angles[12]) {
    // RF = 0, LF = 1, RB = 2, LB = 3; phi = 0, alpha = 1, beta = 2
    for (uint8_t leg = 0; leg < Servos::LEGS; leg++)
        setLegAngles(leg, {
            .phi   = angles[leg * 3 + 0] * 3.14159265f / 180.0f,
            .alpha = angles[leg * 3 + 1] * 3.14159265f / 180.0f,
            .beta  = angles[leg * 3 + 2] * 3.14159265f / 180.0f
        });
}

void ServoDriver::Driver::relax(const uint8_t legIndex) {
    LegServos* leg = getLeg(legIndex);
    if (leg) leg->relax();
}

void ServoDriver::Driver::relaxAll() {
    RF.relax(); LF.relax(); RB.relax(); LB.relax();
}

void ServoDriver::Driver::wake(const uint8_t legIndex) {
    LegServos* leg = getLeg(legIndex);
    if (leg) leg->wake();
}

void ServoDriver::Driver::wakeAll() {
    RF.wake(); LF.wake(); RB.wake(); LB.wake();
}

void ServoDriver::Driver::home() {
    for (uint8_t leg = 0; leg < Servos::LEGS; leg++)
        setLegPlanar(leg, Kinematics::HOME_R, Kinematics::HOME_Y);
}
