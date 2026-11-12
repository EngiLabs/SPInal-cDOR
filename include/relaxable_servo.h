#ifndef RELAXABLE_SERVO_H
#define RELAXABLE_SERVO_H

#include "Multiservo.h"

namespace ServoDriver {
    class RelaxableServo : public Multiservo {
    public:
        RelaxableServo();
        explicit RelaxableServo(uint8_t twiAddress);

        Error attach(uint16_t pin);
        Error attach(uint16_t pin, uint16_t minPulse, uint16_t maxPulse);

        Error relax();
        Error wake();

        Error write(int value);

        Error writeMicroseconds(int pulseWidth);
    private:
        int m_savedPin = -1;
    };
}

#endif
