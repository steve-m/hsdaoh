#ifndef __I2C_H
#define __I2C_H

int hsdaoh_i2c_write_fn(void *dev, uint8_t i2c_addr, uint8_t *buffer, uint16_t len);
int hsdaoh_i2c_read_fn(void *dev, uint8_t i2c_addr, uint8_t *buffer, uint16_t len);

#endif
