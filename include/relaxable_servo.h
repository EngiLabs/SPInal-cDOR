#ifndef RELAXABLE_SERVO_H
#define RELAXABLE_SERVO_H

#include "Multiservo.h"
#include "Servo.h"
#include <stdint.h>

namespace ServoDriver {
    class RelaxableShieldServo : public Multiservo {
    public:
        RelaxableShieldServo() = default;
        explicit RelaxableShieldServo(uint8_t twiAddress);

        Error attach(uint16_t pin);
        Error attach(uint16_t pin, uint16_t minPulse, uint16_t maxPulse);

        Error relax();
        Error wake();

        Error write(int value);
        Error writeMicroseconds(int pulseWidth);

        int getPin() const { return m_savedPin; }
    private:
        int m_savedPin = -1;
        int m_lastAngle = -1;
    };

    class RelaxableStandardServo {
    public:
        RelaxableStandardServo() = default;

        uint8_t attach(int pin);
        uint8_t attach(int pin, int minPulse, int maxPulse);
        void detach();

        void relax();
        void wake();

        void write(int value);
        void writeMicroseconds(int pulseWidth);

        bool attached();
        bool attached() const;
        int getPin() const { return m_savedPin; }

    private:
        ::Servo m_servo{};
        int m_savedPin = -1;
        int m_minPulse = 544,
            m_maxPulse = 2400;
        int m_lastAngle = -1;
    };
}

#endif
