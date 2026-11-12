#include "relaxable_servo.h"


ServoDriver::RelaxableServo::RelaxableServo() = default;
ServoDriver::RelaxableServo::RelaxableServo(const uint8_t twiAddress) : Multiservo(twiAddress) {}

Multiservo::Error ServoDriver::RelaxableServo::attach(const uint16_t pin) {
    m_savedPin = pin;
    return Multiservo::attach(pin);
}
Multiservo::Error ServoDriver::RelaxableServo::attach(const uint16_t pin, const uint16_t minPulse, const uint16_t maxPulse) {
    m_savedPin = pin;
    return Multiservo::attach(pin, minPulse, maxPulse);
}

Multiservo::Error ServoDriver::RelaxableServo::relax() {
    if (!attached()) return OK;
    m_pulseWidth = 0;
    return detach();
}

Multiservo::Error ServoDriver::RelaxableServo::wake() {
    if (!attached() && m_savedPin >= 0)
        return Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
    return OK;
}

Multiservo::Error ServoDriver::RelaxableServo::write(const int value) {
    if (!attached() && m_savedPin >= 0)
        Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
    return Multiservo::write(value);
}

Multiservo::Error ServoDriver::RelaxableServo::writeMicroseconds(const int pulseWidth) {
    if (!attached() && m_savedPin >= 0)
        Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
    return Multiservo::writeMicroseconds(pulseWidth);
}
