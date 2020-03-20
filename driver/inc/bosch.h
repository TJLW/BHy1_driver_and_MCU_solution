#ifndef BOSCH_H
#define BOSCH_H

#include <stdint.h>

int8_t bosch_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
int8_t bosch_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
void bosch_delay(uint32_t msec);

int8_t bosch_i2c_write_ex(void*, uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
int8_t bosch_i2c_read_ex(void*, uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
#endif
