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
#include <stddef.h>
#include <stdint.h>

#include "i2c_interface.h"
#include "bmi160_defs.h"


#define BMI160_DEV_ADDR 0x68

extern int file_po;
int bmi160_open(struct bmi160_dev *ctx);

int main(int argc, char **argv)
{
  struct bmi160_dev sensor;
  uint8_t buf[32];

  int check_bmi;

  check_bmi = bmi160_open(&sensor);

  if (check_bmi == -1){
    printf("\nError occured, while openning sensor as i2c slave\n");
  }

  return 0;
}
