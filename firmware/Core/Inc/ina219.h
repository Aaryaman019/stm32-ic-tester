#ifndef INA219_H
#define INA219_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/* Configures 16 V / 400 mA range (0.1 ohm shunt, 50 uA current LSB). Returns false if no ACK. */
bool  ina219_begin(I2C_HandleTypeDef *hi2c, uint8_t addr7);
float ina219_current_mA(void);   /* signed */
float ina219_bus_V(void);        /* voltage at VIN- (= DUT pin 14) */
uint32_t ina219_errors(void);    /* running count of failed I2C transfers */

#endif /* INA219_H */
