#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdint.h>

struct IrReadings {
    uint16_t LF, RF, LB, RB;
};

void irSensorsInit();
void irSensorsRead(IrReadings &readings);

#endif
