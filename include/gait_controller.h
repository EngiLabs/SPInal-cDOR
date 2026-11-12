#ifndef GAIT_CONTROLLER_H
#define GAIT_CONTROLLER_H

#include <stdint.h>

void gaitInit();
void gaitSetType(uint8_t gaitType);
void gaitSetDirection(int8_t direction, uint8_t speed);
void gaitStop();
void gaitTick();
bool gaitIsActive();

#endif
