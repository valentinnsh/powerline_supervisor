/*
 * Copyright (c) 2021 V.Shishkin
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stddef.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>

#include "i2c_interface.h"
#include "bmi160_defs.h"

#define BMI160_ADDRESS 0x68 //from i2cdetect 0

#define BMI160_DEV_ADDR      BMI160_ADDRESS

int file_po;

int8_t bmi160_init(struct bmi160_dev *dev);

int bmi160_open(struct bmi160_dev *ctx)
{
  int8_t check;

  char* name = "/dev/i2c-0";

  //Open up the I2C
  file_po = open(name, O_RDWR);
  if (file_po == -1)
  {
    perror(name);
    return -1;
  }

  // Specify the address of the slave device.(Another shaman stuff)
  if (ioctl(file_po, I2C_SLAVE, BMI160_ADDRESS) < 0)
  {
    perror("Failed to acquire bus access and/or talk to slave");
    return -1;
  }

  /* I2C setup */
  ctx->write = bmi160_write_array;
  ctx->read = bmi160_read_array;
  ctx->delay_ms = bmi160_delay;

  /* set correct i2c address */
  ctx->id = BMI160_DEV_ADDR;
  ctx->intf = BMI160_I2C_INTF;

  check = bmi160_init(ctx);

  if (check == BMI160_OK)
  {
    printf("BMI160 initialization success !\n");
    printf("Chip ID 0x%X\n", ctx->chip_id);
  }
  else
  {
    printf("BMI160 initialization failure !\n");
    return -1;
  }

  return 0;
}

int write_to_one_register(int file, unsigned char register_num, unsigned char to_register)
{
  int check;
  unsigned char buf[2];

  buf[0] = register_num;
  buf[1] = to_register;

  check = write(file, buf, 2);
  if(check != 2){
    perror("Failed to write to the i2c bus");
    return -1;
  }
  return 0;
}


int read_from_one_register(int file, unsigned char register_num, unsigned char *result)
{

  if(write(file, &register_num, 1) < 0)
  {
    perror("Fail, while writing to single register in read_from_one_register function");
    return -1;
  }

  if(read(file, result, 1) < 0)
  {
    perror("Fail, while reading from single register in read_from_one_register function");
    return -1;
  }

  return 0;
}

int bmi160_read_array(uint8_t register_num, uint8_t *arr, uint8_t len)
{
  int check;

  if(write(file_po, &register_num, 1) < 0)
  {
    perror("Fail, while reading from single register in read_from_one_register function");
    return -1;
  }

  check = read(file_po, arr, len);
  printf("register = %d, file_po = %d, len =  %d, check = %d\n", register_num, file_po, len, check);
  errno = 0;
  if (check != len)
  {
    perror("Failed to read from the i2c bus");
    return -1;
  }

  return 0;
}

int bmi160_write_array(uint8_t register_num, uint8_t *arr, uint8_t len)
{
  int check;

  unsigned char buf[len+1];
  buf[0] = register_num;
  memcpy(buf + 1, arr, len);

  check = write(file_po, buf, len+1);
  if(check != len+1)
  {
    perror("Failed to write to the i2c bus");
    return -1;
  }
  return 0;

}

void bmi160_delay(unsigned int time_ms)
{
  usleep(time_ms * 1000);
}
