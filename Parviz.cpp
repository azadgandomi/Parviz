#include "pico/stdlib.h"
#include "MPU6050Sensor.h"
#include "hardware/i2c.h"
#include "cstdio"


int main() {
    stdio_init_all();
    MPU6050Sensor mpu6050(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
    while (true) {
        mpu6050.update();
        auto quat = mpu6050.quaternion();
        printf("%ld %ld %ld %ld\n", quat.x, quat.y, quat.z, quat.w);
    }
}
