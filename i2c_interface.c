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
#include "bmi160.h"

#define BMI160_ADDRESS 0x68 //from i2cdetect 0

#define BMI160_DEV_ADDR      BMI160_ADDRESS
#define LITTLE_ENDIAN 1

int file_po;

int8_t bmi160_init(struct bmi160_dev *dev);
int8_t bmi160_get_sensor_data(uint8_t select_sensor,
                              struct bmi160_sensor_data *accel,
                              struct bmi160_sensor_data *gyro,
                              const struct bmi160_dev *dev);

struct bmi160_sensor_data bmi160_accel;
struct bmi160_sensor_data bmi160_gyro;

int bmi160_open(struct bmi160_dev *ctx)
{
  int8_t check;
  int rslt;
  char* name = "/dev/i2c-0";

  //Open up the I2C
  errno = 0;
  file_po = open(name, O_RDWR);
  if (file_po == -1)
  {
    perror(name);
    return -1;
  }

  // Specify the address of the slave device.(Another shaman stuff)
  errno = 0;
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
/* Select the Output data rate, range of accelerometer sensor */
  ctx->accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
  ctx->accel_cfg.range = BMI160_ACCEL_RANGE_16G;
  ctx->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

  /* Select the power mode of accelerometer sensor */
  ctx->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

  /* Select the Output data rate, range of Gyroscope sensor */
  //ctx->gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
  //ctx->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
  //ctx->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;


  /* Select the power mode of Gyroscope sensor */
  //ctx->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

  rslt = bmi160_set_sens_conf(ctx);
  printf("res %d\n", rslt);
  return 0;
}

int write_to_one_register(int file, unsigned char register_num, unsigned char to_register)
{
  int check;
  unsigned char buf[2];

  buf[0] = register_num;
  buf[1] = to_register;

  errno = 0;
  check = write(file, buf, 2);
  if(check != 2){
    perror("Failed to write to the i2c bus");
    return -1;
  }
  return 0;
}


int read_from_one_register(int file, unsigned char register_num, unsigned char *result)
{
  errno = 0;
  if(write(file, &register_num, 1) < 0)
  {
    perror("Fail, while writing to single register in read_from_one_register function");
    return -1;
  }

  errno = 0;
  if(read(file, result, 1) < 0)
  {
    perror("Fail, while reading from single register in read_from_one_register function");
    return -1;
  }

  return 0;
}

int8_t bmi160_read_array(uint8_t dev_addr, uint8_t register_num, uint8_t *arr, uint16_t len)
{
  int check;

  (void)dev_addr;
  if(write(file_po, &register_num, 1) < 0)
  {
    perror("Fail, while reading from single register in read_from_one_register function");
    return -1;
  }

  check = read(file_po, arr, len);
  errno = 0;
  if (check != len)
  {
    perror("Failed to read from the i2c bus");
    return -1;
  }

  return 0;
}


int8_t bmi160_write_array(uint8_t dev_addr, uint8_t register_num, uint8_t *arr, uint16_t len)
{
  int check;

  unsigned char buf[len+1];
  (void)dev_addr;

  buf[0] = register_num;
  memcpy(buf + 1, arr, len);

  errno = 0;
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

int8_t set_tap_config(uint8_t feature_enable)
{
    int8_t rslt = BMI160_OK;
    struct bmi160_int_settg int_config;

    if (feature_enable > 0)
    {
        /* Select the Interrupt channel/pin */
        int_config.int_channel = BMI160_INT_CHANNEL_1; /* Interrupt channel/pin 1 */

        /* Select the interrupt channel/pin settings */
        int_config.int_pin_settg.output_en = BMI160_ENABLE; /* Enabling interrupt pins to act as output pin */
        int_config.int_pin_settg.output_mode = BMI160_DISABLE; /* Choosing push-pull mode for interrupt pin */
        int_config.int_pin_settg.output_type = BMI160_ENABLE; /* Choosing active low output */
        int_config.int_pin_settg.edge_ctrl = BMI160_DISABLE; /* Choosing edge triggered output */
        int_config.int_pin_settg.input_en = BMI160_DISABLE; /* Disabling interrupt pin to act as input */
        int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE; /* non-latched output */

        /* Select the Interrupt type */
        int_config.int_type = BMI160_ACC_SINGLE_TAP_INT; /* Choosing tap interrupt */

        /* Select the Any-motion interrupt parameters */
        int_config.int_type_cfg.acc_tap_int.tap_en = BMI160_ENABLE; /* 1- Enable tap, 0- disable tap */
        int_config.int_type_cfg.acc_tap_int.tap_thr = 2; /* Set tap threshold */
        int_config.int_type_cfg.acc_tap_int.tap_dur = 2; /* Set tap duration */
        int_config.int_type_cfg.acc_tap_int.tap_shock = 0; /* Set tap shock value */
        int_config.int_type_cfg.acc_tap_int.tap_quiet = 0; /* Set tap quiet duration */
        int_config.int_type_cfg.acc_tap_int.tap_data_src = 1; /* data source 0 : filter or 1 : pre-filter */

        /* Set the Any-motion interrupt */
        rslt = bmi160_set_int_config(&int_config, &bmi160dev); /* sensor is an instance of the structure bmi160_dev  */
        printf("bmi160_set_int_config(tap enable) status:%d\n", rslt);
    }
    else
    {
        /* Select the Interrupt channel/pin */
        int_config.int_channel = BMI160_INT_CHANNEL_1;
        int_config.int_pin_settg.output_en = BMI160_DISABLE; /* Disabling interrupt pins to act as output pin */
        int_config.int_pin_settg.edge_ctrl = BMI160_DISABLE; /* Choosing edge triggered output */

        /* Select the Interrupt type */
        int_config.int_type = BMI160_ACC_SINGLE_TAP_INT; /* Choosing Tap interrupt */
        int_config.int_type_cfg.acc_tap_int.tap_en = BMI160_DISABLE; /* 1- Enable tap, 0- disable tap */

        /* Set the Data ready interrupt */
        rslt = bmi160_set_int_config(&int_config, &bmi160dev); /* sensor is an instance of the structure bmi160_dev */
        printf("bmi160_set_int_config(tap disable) status:%d\n", rslt);
    }

    return rslt;
}
