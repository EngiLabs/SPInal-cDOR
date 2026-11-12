#ifndef IMU_READER_H
#define IMU_READER_H

#include <stdint.h>

struct ImuData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temperature;
};

void imuInit();
bool imuRead(ImuData &data);

#endif
