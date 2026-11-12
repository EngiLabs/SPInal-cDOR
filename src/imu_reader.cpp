#include "imu_reader.h"
#include "config.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static Adafruit_MPU6050 mpu;
static bool initialized = false;

void real_imuInit() {
    initialized = mpu.begin(Imu::MPU6050_ADDR);
    if (initialized) {
        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
}

void imuInit() {}

bool real_imuRead(ImuData &data) {
    if (!initialized) {
        memset(&data, 0, sizeof(ImuData));
        return false;
    }

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    data.accelX = accel.acceleration.x;
    data.accelY = accel.acceleration.y;
    data.accelZ = accel.acceleration.z;
    data.gyroX = gyro.gyro.x;
    data.gyroY = gyro.gyro.y;
    data.gyroZ = gyro.gyro.z;
    data.temperature = temp.temperature;

    return true;
}

bool imuRead(ImuData &data) {
    data.accelX = 0;
    data.accelY = 0;
    data.accelZ = 0;
    data.gyroX = 0;
    data.gyroY = 0;
    data.gyroZ = 0;
    data.temperature = 0;
    return true;
}
