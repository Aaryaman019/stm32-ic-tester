/* Minimal INA219 driver (HAL I2C). Same calibration as Adafruit setCalibration_16V_400mA(). */
#include "ina219.h"

#define REG_CONFIG   0x00u
#define REG_BUSV     0x02u
#define REG_CURRENT  0x04u
#define REG_CAL      0x05u

#define CAL_VALUE    8192u     /* 0.04096 / (50 uA * 0.1 ohm) */
#define CFG_VALUE    0x01DFu   /* BRNG=16V | PGA=/1 (+-40mV) | BADC=12bit | SADC=12bit x8 avg (4.3 ms) | continuous */
#define CURR_LSB_MA  0.05f

static I2C_HandleTypeDef *s_i2c;
static uint16_t           s_addr;
static volatile uint32_t  s_err;

uint32_t ina219_errors(void) { return s_err; }

static bool wr16(uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    if (HAL_I2C_Mem_Write(s_i2c, s_addr, reg, I2C_MEMADD_SIZE_8BIT, b, 2, 20) == HAL_OK) return true;
    s_err++;
    return false;
}

static bool rd16(uint8_t reg, uint16_t *v)
{
    uint8_t b[2];
    if (HAL_I2C_Mem_Read(s_i2c, s_addr, reg, I2C_MEMADD_SIZE_8BIT, b, 2, 20) != HAL_OK) { s_err++; return false; }
    *v = (uint16_t)((b[0] << 8) | b[1]);
    return true;
}

bool ina219_begin(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    s_i2c  = hi2c;
    s_addr = (uint16_t)(addr7 << 1);
    if (HAL_I2C_IsDeviceReady(s_i2c, s_addr, 3, 10) != HAL_OK) return false;
    return wr16(REG_CAL, CAL_VALUE) && wr16(REG_CONFIG, CFG_VALUE);
}

float ina219_current_mA(void)
{
    uint16_t r;
    if (!rd16(REG_CURRENT, &r)) return 0.0f;
    return (float)(int16_t)r * CURR_LSB_MA;
}

float ina219_bus_V(void)
{
    uint16_t r;
    if (!rd16(REG_BUSV, &r)) return 0.0f;           /* 0 V -> VCC_OUT_OF_RANGE abort (fail-safe) */
    return (float)(r >> 3) * 0.004f;               /* 4 mV LSB */
}
