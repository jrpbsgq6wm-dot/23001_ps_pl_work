#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "tempsensor.h"

#ifndef I2C_SLAVE_FORCE
#define I2C_SLAVE_FORCE 0x0706
#endif

#define NST175_I2C_DEVICE "/dev/i2c-0"
#define NST175_RETRY_COUNT 5

int fd_icc = -1;
float dry_temp = 0.0f;
uint8 i2c_read_reg = 0;

static int nst175_write_register(int fd, uint8 reg, uint8 value)
{
    int retries;
    uint8 data[2];

    data[0] = reg;
    data[1] = value;

    for (retries = 0; retries < NST175_RETRY_COUNT; retries++)
    {
        if (write(fd, data, sizeof(data)) == (ssize_t)sizeof(data))
        {
            return 0;
        }

        usleep(10000);
    }

    return -1;
}

static int nst175_read_registers(int fd, uint8 reg, uint8 *data,
                                 unsigned int length)
{
    int retries;

    for (retries = 0; retries < NST175_RETRY_COUNT; retries++)
    {
        if (write(fd, &reg, sizeof(reg)) == (ssize_t)sizeof(reg) &&
            read(fd, data, length) == (ssize_t)length)
        {
            return 0;
        }

        usleep(10000);
    }

    return -1;
}

/*
 * Open the I2C device, select NST175 and configure continuous 12-bit
 * conversion. The previous descriptor is closed before reconnecting.
 */
int NST175_Init(void)
{
    if (fd_icc >= 0)
    {
        close(fd_icc);
        fd_icc = -1;
    }

    fd_icc = open(NST175_I2C_DEVICE, O_RDWR);
    if (fd_icc < 0)
    {
        perror("open /dev/i2c-0");
        return -1;
    }

    if (ioctl(fd_icc, I2C_SLAVE_FORCE, NST175_I2C_ADDR) < 0)
    {
        perror("set NST175 I2C address");
        close(fd_icc);
        fd_icc = -1;
        return -1;
    }

    if (nst175_write_register(fd_icc, NST175_REG_CONFIG,
                              NST175_CONFIG_CONTINUOUS_12BIT) != 0)
    {
        perror("configure NST175");
        close(fd_icc);
        fd_icc = -1;
        return -1;
    }

    i2c_read_reg = 0;
    return 0;
}

/*
 * NST175 temperature register contains a signed 12-bit value, left aligned
 * in two bytes. Dividing the signed 16-bit value by 256 gives degrees C.
 */
int get_sensor_data(void)
{
    uint8 data[2];
    int16_t raw_temperature;

    if (fd_icc < 0)
    {
        i2c_read_reg = 1;
        return -1;
    }

    if (nst175_read_registers(fd_icc, NST175_REG_TEMPERATURE, data,
                              sizeof(data)) != 0)
    {
        i2c_read_reg = 1;
        return -1;
    }

    raw_temperature = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    dry_temp = (float)raw_temperature / 256.0f;
    i2c_read_reg = 0;

    return 0;
}
