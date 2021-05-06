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

#include "bmi160_defs.h"
#include "bmi160.h"
#include "i2c_interface.h"


#define BMI160_DEV_ADDR 0x68

extern int file_po;
int bmi160_open(struct bmi160_dev *ctx);

int main(int argc, char **argv)
{
  (void)argv[argc];

  struct bmi160_dev sensor;

  int check_bmi;
  int rslt;

  check_bmi = bmi160_open(&sensor);
  if (check_bmi == -1){
    printf("\nError occured, while openning sensor as i2c slave\n");
  }
  rslt = set_tap_config(&sensor, BMI160_ENABLE);

  if (rslt == BMI160_OK)
  {
    union bmi160_int_status int_status;
    uint8_t loop = 0;

    printf("Do Single or Double Tap the board\n");
    fflush(stdout);
    while (1)
    {
      /* Read interrupt status */
      memset(int_status.data, 0x00, sizeof(int_status.data));
      rslt = bmi160_get_int_status(BMI160_INT_STATUS_ALL, &int_status, &sensor);

      /* Enters only if the obtained interrupt is single-tap */
      if (rslt == BMI160_OK)
      {
	/* Enters only if the obtained interrupt is single-tap */
	if (int_status.bit.s_tap)
	{
	  printf("Single tap, iter:%d, int_status:0x%x\n",
		 loop++,
		 int_status.data[0]);
	}
	/* Enters only if the obtained interrupt is double-tap */
	else if (int_status.bit.d_tap)
	{
	  printf("Double tap, iter:%d, int_status:0x%x\n",
		 loop++,
		 int_status.data[0]);
	}

	fflush(stdout);
      }
      else
      {
	break;
      }
    }

    /* Disable tap feature */
    printf("\nDisable tap test...\n");
    rslt = set_tap_config(&sensor, BMI160_DISABLE);
    printf("bmi160_set_int_config(tap enable) status:%d\n", rslt);

    fflush(stdout);
  }

  return 0;
}
