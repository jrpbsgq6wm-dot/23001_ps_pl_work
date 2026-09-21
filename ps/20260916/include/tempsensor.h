#ifndef __TEMPSENSOR_H
#define __TEMPSENSOR_H

typedef unsigned char uint8;

extern int fd_icc;
extern float dry_temp;
extern uint8 i2c_read_reg;

/* NST175 on I2C0, 7-bit slave address 1001110b. */
#define NST175_I2C_ADDR                 0x4E
#define NST175_REG_TEMPERATURE          0x00
#define NST175_REG_CONFIG               0x01
#define NST175_CONFIG_CONTINUOUS_12BIT  0x60

int NST175_Init(void);
int get_sensor_data(void);

#endif
