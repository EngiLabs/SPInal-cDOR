#include "servo_driver.h"
#include "config.h"
#include <math.h>


RelaxableServo::RelaxableServo() = default;
RelaxableServo::RelaxableServo(const uint8_t twiAddress) : Multiservo(twiAddress) {}

Multiservo::Error RelaxableServo::attach(const uint16_t pin) {
    m_savedPin = pin;
    return Multiservo::attach(pin);
}
Multiservo::Error RelaxableServo::attach(const uint16_t pin, const uint16_t minPulse, const uint16_t maxPulse) {
    m_savedPin = pin;
    return Multiservo::attach(pin, minPulse, maxPulse);
}

Multiservo::Error RelaxableServo::relax() {
    if (!attached()) return OK;
    m_pulseWidth = 0;
    return detach();
}

Multiservo::Error RelaxableServo::wake() {
    if (!attached() && m_savedPin >= 0)
        return Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
    return OK;
}

Multiservo::Error RelaxableServo::write(const int value) {
    if (!attached() && m_savedPin >= 0)
        Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
    return Multiservo::write(value);
}

Multiservo::Error RelaxableServo::writeMicroseconds(const int pulseWidth) {
    if (!attached() && m_savedPin >= 0)
        Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
    return Multiservo::writeMicroseconds(pulseWidth);
}


static RelaxableServo servos[Servo::LEGS][Servo::DOF_PER_LEG];

static int toServoAngle(const float rad, const int zero, const int dir) {
    return zero + dir * static_cast<int>(lround(rad * 180.0f / 3.14159265f));
}

static bool inRange(const int val, const int lo, const int hi) {
    return val >= lo && val <= hi;
}

void servoDriverInit() {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        for (uint8_t dof = 0; dof < Servo::DOF_PER_LEG; dof++)
            servos[leg][dof].attach(Pin::SERVO_PINS[leg][dof]);
}

bool servoSetLegAngles(const uint8_t legIndex, const LegAngles &angles) {
    if (legIndex >= Servo::LEGS) return false;

    const int phi   = toServoAngle(angles.phi,   Servo::PHI_ZERO,   Servo::PHI_DIR),
              alpha = toServoAngle(angles.alpha, Servo::ALPHA_ZERO, Servo::ALPHA_DIR),
              beta  = toServoAngle(angles.beta,  Servo::BETA_ZERO,  Servo::BETA_DIR);

    if (!inRange(phi, Servo::PHI_SERVO_MIN, Servo::PHI_SERVO_MAX) ||
        !inRange(alpha, Servo::ALPHA_SERVO_MIN, Servo::ALPHA_SERVO_MAX) ||
        !inRange(beta, Servo::BETA_SERVO_MIN, Servo::BETA_SERVO_MAX))
        return false;

    servos[legIndex][Servo::JOINT_PHI].write(phi);
    servos[legIndex][Servo::JOINT_ALPHA].write(alpha);
    servos[legIndex][Servo::JOINT_BETA].write(beta);

    return true;
}

bool servoSetLegPosition(const uint8_t legIndex, const float x, const float y, const float z) {
    const FootPosition pos = { .x = x, .y = y, .z = z };
    LegAngles angles{};
    const IkStatus status = inverseKinematics(pos, angles);

    if (status != IkStatus::OK) return false;
    return servoSetLegAngles(legIndex, angles);
}

bool servoSetLegPlanar(const uint8_t legIndex, const float R, const float y) {
    float alpha, beta;
    const IkStatus status = solvePlanar(R, y, alpha, beta);

    if (status != IkStatus::OK) return false;

    return servoSetLegAngles(legIndex, {
        .phi = 0.0f,
        .alpha = alpha,
        .beta = beta
    });
}

void servoSetAllAngles(const float angles[12]) {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        servoSetLegAngles(leg, {
            .phi   = angles[leg * 3 + 0] * 3.14159265f / 180.0f,
            .alpha = angles[leg * 3 + 1] * 3.14159265f / 180.0f,
            .beta  = angles[leg * 3 + 2] * 3.14159265f / 180.0f
        });
}

void servoRelax(const uint8_t legIndex) {
    if (legIndex >= Servo::LEGS) return;
    for (uint8_t dof = 0; dof < Servo::DOF_PER_LEG; dof++)
        servos[legIndex][dof].relax();
}

void servoRelaxAll() {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        servoRelax(leg);
}

void servoWake(const uint8_t legIndex) {
    if (legIndex >= Servo::LEGS) return;
    for (uint8_t dof = 0; dof < Servo::DOF_PER_LEG; dof++)
        servos[legIndex][dof].wake();
}

void servoWakeAll() {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        servoWake(leg);
}

void servoHome() {
    for (uint8_t leg = 0; leg < Servo::LEGS; leg++)
        servoSetLegPlanar(leg, Kinematics::HOME_R, Kinematics::HOME_Y);
}
