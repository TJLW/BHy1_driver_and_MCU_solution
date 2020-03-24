#ifndef BOSCH_H
#define BOSCH_H

#include <stdint.h>



#define BHY_INT 36

int i2c_fd;


int open_i2c_device();
void close_i2c_device();


int8_t ioport_get_pin_level(int gpio_pin);


int8_t linux_i2c_write(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
int8_t linux_i2c_read(uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
void linux_delay(uint32_t msec);

void linux_printf(const u8 * string);

// int8_t li_i2c_write_ex(void*, uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
// int8_t bosch_i2c_read_ex(void*, uint8_t addr, uint8_t reg, uint8_t *p_buf, uint16_t size);
#endif
