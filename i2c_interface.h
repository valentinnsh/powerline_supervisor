#ifndef I2C_INTERFACE_H_INCLUDED
#define I2C_INTERFACE_H_INCLUDED


int write_to_one_register(int file, unsigned char register_num,
			  unsigned char to_register);

int read_from_one_register(int file, unsigned char register_num,
			   unsigned char *result);

int8_t bmi160_read_array(uint8_t dev_addr, uint8_t register_num,
		      uint8_t *arr, uint16_t len);

int8_t bmi160_write_array(uint8_t dev_addr, uint8_t register_num,
		       uint8_t *arr, uint16_t len);

void bmi160_delay(unsigned int time_ms);

int8_t set_tap_config(struct bmi160_dev *ctx, uint8_t feature_enable);


#endif
