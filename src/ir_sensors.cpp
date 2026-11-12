#include "ir_sensors.h"
#include "config.h"
#include <Arduino.h>

void irSensorsInit() {
    pinMode(Pin::IR_LF, INPUT);
    pinMode(Pin::IR_RF, INPUT);
    pinMode(Pin::IR_LB, INPUT);
    pinMode(Pin::IR_RB, INPUT);
}

void irSensorsRead(IrReadings &readings) {
    readings.LF = analogRead(Pin::IR_LF);
    readings.RF = analogRead(Pin::IR_RF);
    readings.LB = analogRead(Pin::IR_LB);
    readings.RB = analogRead(Pin::IR_RB);
}
