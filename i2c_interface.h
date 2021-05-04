#ifndef I2C_INTERFACE_H_INCLUDED
#define I2C_INTERFACE_H_INCLUDED

int write_to_one_register(int file, unsigned char register_num,
			  unsigned char to_register);

int read_from_one_register(int file, unsigned char register_num,
			   unsigned char *result);

int bmi160_read_array(uint8_t dev_addr, uint8_t register_num,
		      uint8_t *arr, uint8_t len);

int bmi160_write_array(uint8_t dev_addr, uint8_t register_num,
		       uint8_t *arr, uint8_t len);

void bmi160_delay(unsigned int time_ms);


#endif
