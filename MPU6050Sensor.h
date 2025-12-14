#ifndef PARVIZ_MPU6050Sensor_H
#define PARVIZ_MPU6050Sensor_H

#include <cstring>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"
#include "eMPL/inv_mpu.h"

namespace {
    constexpr unsigned short SAMPLE_RATE = 200;
}

class MPU6050Sensor {
public:

    struct Acceleration {
        float x{};
        float y{};
        float z{};
    };

    struct Gyro {
        float x{};
        float y{};
        float z{};
    };

    struct Quaternion {
        long w{};
        long x{};
        long y{};
        long z{};
    };

    MPU6050Sensor(uint sda_pin, uint scl_pin) {
        i2c_init(i2c_default, 400 * 1000);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);

        int result = mpu_init(nullptr);
        assert(result == 0);
        result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        assert(result == 0);
        result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        assert(result == 0);
        result = mpu_set_sample_rate(SAMPLE_RATE);
        assert(result == 0);
        result = dmp_load_motion_driver_firmware();
        assert(result == 0);
        result = dmp_enable_feature(
                DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_GYRO_CAL);
        assert(result == 0);
        result = dmp_set_fifo_rate(SAMPLE_RATE);
        assert(result == 0);
        result = mpu_set_dmp_state(1);
        assert(result == 0);

        unsigned short accel_sensitivity{};
        result = mpu_get_accel_sens(&accel_sensitivity);
        assert(result == 0);
        m_accel_sensitivity = static_cast<float>(accel_sensitivity);
        result = mpu_get_gyro_sens(&m_gyro_sensitivity);
        assert(result == 0);
    }

    [[nodiscard]] Acceleration acceleration() const {
        return m_acceleration;
    }

    [[nodiscard]] Gyro gyro() const {
        return m_gyro;
    }

    [[nodiscard]] Quaternion quaternion() const {
        return m_quat;
    }

    void update() {
        short gyro[3], accel[3], sensors;
        unsigned long sensor_timestamp;
        unsigned char more{};
        long quat[4];
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                      &more);
        if (sensors & INV_XYZ_GYRO) {
            m_gyro = {gyro[0] / m_gyro_sensitivity, gyro[1] / m_gyro_sensitivity, gyro[2] / m_gyro_sensitivity};
        }
        if (sensors & INV_XYZ_ACCEL) {
            m_acceleration = {accel[0] / m_accel_sensitivity, accel[1] / m_accel_sensitivity,
                              accel[2] / m_accel_sensitivity};
        }
        if (sensors & INV_WXYZ_QUAT) {
            m_quat = {quat[0], quat[1], quat[2], quat[3]};
        }
    }

private:

    const uint8_t addr = 0x68;
    Acceleration m_acceleration{};
    Gyro m_gyro{};
    Quaternion m_quat{};
    float m_accel_sensitivity{};
    float m_gyro_sensitivity{};
};

#endif