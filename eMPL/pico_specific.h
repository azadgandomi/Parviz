//
// Created by azad on 2021-06-02.
//

#ifndef PARVIZ_PICO_SPECIFIC_H
#define PARVIZ_PICO_SPECIFIC_H

#ifdef __cplusplus
extern "C" {
#endif

static int i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data) {
    uint8_t *buf = (uint8_t *) malloc(length + 1);
    buf[0] = reg_addr;
    memcpy(buf + 1, data, length);
    int result = i2c_write_blocking(i2c_default, slave_addr, buf, length + 1, false);
    free(buf);
    return result == length + 1 ? 0 : -1;
}

static int i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data) {

    if (i2c_write_blocking(i2c_default, slave_addr, &reg_addr, 1, true) != 1) {
        return -1;
    }
    return i2c_read_blocking(i2c_default, slave_addr, data, length, false) == length ? 0 : -1;
}

static int delay_ms(unsigned long num_ms) {
    sleep_ms(num_ms);
    return 0;
}

static int get_ms(unsigned long *count) {
    *count = to_ms_since_boot(get_absolute_time());
    return 0;
}

static int reg_int_cb(struct int_param_s * params)
{
    if (params) {
        //gpio_set_irq_enabled_with_callback(params->gpio, GPIO_IRQ_LEVEL_LOW, true, params->callback);
        return 0;
    }
    return -1;
}

#define log_i(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define log_e(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#define min(a,b) (((a)<(b))?(a):(b))


#ifdef __cplusplus
}
#endif

#endif //PARVIZ_PICO_SPECIFIC_H