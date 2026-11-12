#include "servo_driver.h"

#include <math.h>


static int to_servo_angle(const float rad, const int zero, const int direction) {
    return zero + direction * static_cast<int>(lround(rad * 180.0f / 3.14159265f));
}

static bool in_range(const int val, const int lo, const int hi) {
    return val >= lo && val <= hi;
}


ServoDriver::Driver::Driver() {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        for (uint8_t dof = 0; dof < Servo::DOF_PER_LEG; dof++)
            servos[leg][dof].attach(Pin::SERVO_PINS[leg][dof]);
}

bool ServoDriver::Driver::setLegAngles(const uint8_t legIndex, const LegAngles &angles) {
    if (legIndex >= Servo::LEGS) return false;

    const int phi   = to_servo_angle(angles.phi,   Servo::PHI_ZERO,   Servo::PHI_DIR),
              alpha = to_servo_angle(angles.alpha, Servo::ALPHA_ZERO, Servo::ALPHA_DIR),
              beta  = to_servo_angle(angles.beta,  Servo::BETA_ZERO,  Servo::BETA_DIR);

    if (!in_range(phi, Servo::PHI_SERVO_MIN, Servo::PHI_SERVO_MAX) ||
        !in_range(alpha, Servo::ALPHA_SERVO_MIN, Servo::ALPHA_SERVO_MAX) ||
        !in_range(beta, Servo::BETA_SERVO_MIN, Servo::BETA_SERVO_MAX))
        return false;

    servos[legIndex][Servo::JOINT_PHI].write(phi);
    servos[legIndex][Servo::JOINT_ALPHA].write(alpha);
    servos[legIndex][Servo::JOINT_BETA].write(beta);

    return true;
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
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        setLegAngles(leg, {
            .phi   = angles[leg * 3 + 0] * 3.14159265f / 180.0f,
            .alpha = angles[leg * 3 + 1] * 3.14159265f / 180.0f,
            .beta  = angles[leg * 3 + 2] * 3.14159265f / 180.0f
        });
}

void ServoDriver::Driver::relax(const uint8_t legIndex) {
    if (legIndex >= Servo::LEGS) return;
    for (uint8_t dof = 0; dof < Servo::DOF_PER_LEG; dof++)
        servos[legIndex][dof].relax();
}

void ServoDriver::Driver::relaxAll() {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        relax(leg);
}

void ServoDriver::Driver::wake(const uint8_t legIndex) {
    if (legIndex >= Servo::LEGS) return;
    for (uint8_t dof = 0; dof < Servo::DOF_PER_LEG; dof++)
        servos[legIndex][dof].wake();
}

void ServoDriver::Driver::wakeAll() {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        wake(leg);
}

void ServoDriver::Driver::home() {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        setLegPlanar(leg, Kinematics::HOME_R, Kinematics::HOME_Y);
}
