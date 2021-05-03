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
#include <sys/ioctl.h>
#include <fcntl.h>

#include "i2c_interface.h"
int write_to_register(int file, unsigned char register_num, unsigned char to_register)
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

int bme280_read_array(bme280_t *ctx, uint8_t register_num, uint8_t *arr, uint8_t len)
{
  int check;

  if(write(ctx->file_po, &register_num, 1) < 0)
  {
    perror("Fail, while reading from single register in read_from_one_register function");
    return -1;
  }

  check = read(ctx->file_po, arr, len);
  if (check < 0)
  {
    perror("Failed to read from the i2c bus");
    return -1;
  }

  return 0;
}
