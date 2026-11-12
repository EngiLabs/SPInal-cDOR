#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>
#include "kinematics.h"
#include "Multiservo.h"

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

void servoDriverInit();

bool servoSetLegAngles(uint8_t legIndex, const LegAngles &angles);
bool servoSetLegPosition(uint8_t legIndex, float x, float y, float z);
bool servoSetLegPlanar(uint8_t legIndex, float R, float y);

void servoSetAllAngles(const float angles[12]);

void servoRelax(uint8_t legIndex);
void servoRelaxAll();

void servoWake(uint8_t legIndex);
void servoWakeAll();

void servoHome();

#endif
