#include "relaxable_servo.h"


ServoDriver::RelaxableShieldServo::RelaxableShieldServo(const uint8_t twiAddress)
: Multiservo(twiAddress) {}

Multiservo::Error ServoDriver::RelaxableShieldServo::attach(const uint16_t pin) {
    m_savedPin = pin;
    return Multiservo::attach(pin);
}

Multiservo::Error ServoDriver::RelaxableShieldServo::attach(const uint16_t pin, const uint16_t minPulse, const uint16_t maxPulse) {
    m_savedPin = pin;
    return Multiservo::attach(pin, minPulse, maxPulse);
}

Multiservo::Error ServoDriver::RelaxableShieldServo::relax() {
    if (!attached()) return OK;
    m_pulseWidth = 0;
    return detach();
}

Multiservo::Error ServoDriver::RelaxableShieldServo::wake() {
    if (!attached() && m_savedPin >= 0) {
        Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
        if (m_lastAngle >= 0)
            return Multiservo::write(m_lastAngle);
    }
    return OK;
}

Multiservo::Error ServoDriver::RelaxableShieldServo::write(const int value) {
    m_lastAngle = value;
    if (!attached() && m_savedPin >= 0)
        Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
    return Multiservo::write(value);
}

Multiservo::Error ServoDriver::RelaxableShieldServo::writeMicroseconds(const int pulseWidth) {
    if (!attached() && m_savedPin >= 0)
        Multiservo::attach(m_savedPin, m_minPulse, m_maxPulse);
    return Multiservo::writeMicroseconds(pulseWidth);
}


uint8_t ServoDriver::RelaxableStandardServo::attach(const int pin) {
    m_savedPin = pin;
    return m_servo.attach(pin);
}

uint8_t ServoDriver::RelaxableStandardServo::attach(const int pin, const int minPulse, const int maxPulse) {
    m_savedPin = pin;
    m_minPulse = minPulse;
    m_maxPulse = maxPulse;
    return m_servo.attach(pin, minPulse, maxPulse);
}

void ServoDriver::RelaxableStandardServo::detach() {
    m_servo.detach();
}

void ServoDriver::RelaxableStandardServo::relax() {
    if (!m_servo.attached()) return;
    m_servo.detach();
}

void ServoDriver::RelaxableStandardServo::wake() {
    if (!m_servo.attached() && m_savedPin >= 0) {
        m_servo.attach(m_savedPin, m_minPulse, m_maxPulse);
        if (m_lastAngle >= 0)
            m_servo.write(m_lastAngle);
    }
}

void ServoDriver::RelaxableStandardServo::write(const int value) {
    m_lastAngle = value;
    if (!m_servo.attached() && m_savedPin >= 0)
        m_servo.attach(m_savedPin, m_minPulse, m_maxPulse);
    m_servo.write(value);
}

void ServoDriver::RelaxableStandardServo::writeMicroseconds(const int pulseWidth) {
    if (!m_servo.attached() && m_savedPin >= 0)
        m_servo.attach(m_savedPin, m_minPulse, m_maxPulse);
    m_servo.writeMicroseconds(pulseWidth);
}

bool ServoDriver::RelaxableStandardServo::attached() {
    return m_servo.attached();
}

bool ServoDriver::RelaxableStandardServo::attached() const {
    return const_cast<::Servo&>(m_servo).attached();
}
