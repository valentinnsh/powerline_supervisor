/* ZBOSS Zigbee software protocol stack
 *
 * Copyright (c) 2012-2020 DSR Corporation, Denver CO, USA.
 * www.dsr-zboss.com
 * www.dsr-corporation.com
 * All rights reserved.
 *
 * This is unpublished proprietary source code of DSR Corporation
 * The copyright notice does not evidence any actual or intended
 * publication of such source code.
 *
 * ZBOSS is a registered trademark of Data Storage Research LLC d/b/a DSR
 * Corporation
 *
 * Commercial Usage
 * Licensees holding valid DSR Commercial licenses may use
 * this file in accordance with the DSR Commercial License
 * Agreement provided with the Software or, alternatively, in accordance
 * with the terms contained in a written agreement between you and
 * DSR.
 */

#define ZB_TRACE_FILE_ID 600
#include "zboss_api.h"
#include "zb_led_button.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

#include "bmi160_defs.h"
#include "bmi160.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

#define BMI160_ADDRESS 0x69 //from i2cdetect 0

#define BMI160_DEV_ADDR      BMI160_ADDRESS

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}



zb_void_t send_toogle_req(zb_uint8_t param);
zb_void_t button_press_handler(zb_uint8_t param);

int alarm_busy;
/* #if ! defined ZB_ROUTER_ROLE */
/* #error define ZB_ROUTER_ROLE to compile ze tests */
/* #endif */

/* Handler for specific zcl commands */
zb_uint8_t zcl_specific_cluster_cmd_handler(zb_uint8_t param);

/* Parse read attributes response */
zb_void_t on_off_read_attr_resp_handler(zb_buf_t *cmd_buf);
zb_void_t test_restart_join_nwk(zb_uint8_t param);

zb_bool_t cmd_in_progress = ZB_FALSE;


/** [COMMON_DECLARATION] */
/******************* Declare attributes ************************/

/* Switch config cluster attributes data */
zb_uint8_t attr_switch_type =
    ZB_ZCL_ON_OFF_SWITCH_CONFIGURATION_SWITCH_TYPE_TOGGLE;
zb_uint8_t attr_switch_actions =
    ZB_ZCL_ON_OFF_SWITCH_CONFIGURATION_SWITCH_ACTIONS_DEFAULT_VALUE;

ZB_ZCL_DECLARE_ON_OFF_SWITCH_CONFIGURATION_ATTRIB_LIST(switch_cfg_attr_list, &attr_switch_type, &attr_switch_actions);

/* Basic cluster attributes data */
zb_uint8_t attr_zcl_version  = ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
zb_uint8_t attr_power_source = ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE;

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST(basic_attr_list, &attr_zcl_version, &attr_power_source);

/* Identify cluster attributes data */
zb_uint16_t attr_identify_time = 0;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &attr_identify_time);

/********************* Declare device **************************/
#define ZB_SWITCH_ENDPOINT          10

ZB_HA_DECLARE_ON_OFF_SWITCH_CLUSTER_LIST(on_off_switch_clusters, switch_cfg_attr_list, basic_attr_list, identify_attr_list);

ZB_HA_DECLARE_ON_OFF_SWITCH_EP(on_off_switch_ep, ZB_SWITCH_ENDPOINT, on_off_switch_clusters);

ZB_HA_DECLARE_ON_OFF_SWITCH_CTX(on_off_switch_ctx, on_off_switch_ep);
/** [COMMON_DECLARATION] */

zb_ieee_addr_t g_ed_addr = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};

/* Example of user-defined main loop.
 * If you don't need more complicated functionality inside your main loop, use zboss_main_loop() function. */
/* [zboss_main_loop_iteration] */
void my_main_loop()
{
  while (1)
  {
    /* ... User code ... */
    zboss_main_loop_iteration();
    /* ... User code ... */
  }
}
/* [zboss_main_loop_iteration] */

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//ret_code_t 	nrf_drv_twi_rx (nrf_drv_twi_t const *p_instance, uint8_t address, uint8_t *p_data, uint8_t length)
//ret_code_t 	nrf_drv_twi_tx (nrf_drv_twi_t const *p_instance, uint8_t address, uint8_t const *p_data, uint8_t length, bool no_stop)
int8_t bmi160_read_from_reg(uint8_t dev_addr, uint8_t register_num, uint8_t *arr, uint16_t len)
{
  int check;

  check = nrf_drv_twi_tx(&m_twi, dev_addr, &register_num, 1, false);
  if(check != NRF_SUCCESS)
  {
    NRF_LOG_INFO("Failed to write register num in bmi160_read_from_reg()");
    return -1;
  }

  check = nrf_drv_twi_rx(&m_twi, dev_addr, arr, len);
  if (check != NRF_SUCCESS)
  {
    NRF_LOG_INFO("Failed to bmi160_read_from_reg()");
    return -1;
  }
  return 0;
}

//typedef int8_t (*bmi160_write_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
//ret_code_t 	nrf_drv_twi_tx (nrf_drv_twi_t const *p_instance, uint8_t address, uint8_t const *p_data, uint8_t length, bool no_stop)
int8_t bmi160_write_to_reg(uint8_t dev_addr, uint8_t register_num, uint8_t *data, uint16_t len)
{
  int check;

  unsigned char buf[len+1];

  buf[0] = register_num;
  memcpy(buf + 1, data, len);

  check = nrf_drv_twi_tx(&m_twi, dev_addr, buf, len+1, false);
  if(check != NRF_SUCCESS)
  {
    NRF_LOG_INFO("Failed to write to register in bmi160_write_to_reg()");
    return -1;
  }
  return 0;

}

void bmi160_delay(uint32_t period)
{
  //usleep(time_ms * 1000);
  nrf_delay_ms(period);
}

int8_t set_tap_config(struct bmi160_dev *ctx, uint8_t feature_enable)
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
        int_config.int_type = BMI160_ACC_ANY_MOTION_INT; /* Choosing tap interrupt */

        /* Select the Any-motion interrupt parameters */
	#if 0
        int_config.int_type_cfg.acc_tap_int.tap_en = BMI160_ENABLE; /* 1- Enable tap, 0- disable tap */
        int_config.int_type_cfg.acc_tap_int.tap_thr = 2; /* Set tap threshold */
        int_config.int_type_cfg.acc_tap_int.tap_dur = 2; /* Set tap duration */
        int_config.int_type_cfg.acc_tap_int.tap_shock = 0; /* Set tap shock value */
        int_config.int_type_cfg.acc_tap_int.tap_quiet = 0; /* Set tap quiet duration */
        int_config.int_type_cfg.acc_tap_int.tap_data_src = 1; /* data source 0 : filter or 1 : pre-filter */
	#endif
	/*! 1 any-motion enable, 0 - any-motion disable */
      int_config.int_type_cfg.acc_any_motion_int.anymotion_en  = BMI160_ENABLE;
      int_config.int_type_cfg.acc_any_motion_int.anymotion_x  = BMI160_ENABLE;
      int_config.int_type_cfg.acc_any_motion_int.anymotion_y  = BMI160_ENABLE;
      int_config.int_type_cfg.acc_any_motion_int.anymotion_z  = BMI160_ENABLE;
      int_config.int_type_cfg.acc_any_motion_int.anymotion_dur  = 2;
      int_config.int_type_cfg.acc_any_motion_int.anymotion_data_src  = 1;
      int_config.int_type_cfg.acc_any_motion_int.anymotion_thr  = 2;

        /* Set the Any-motion interrupt */
        rslt = bmi160_set_int_config(&int_config, ctx); /* sensor is an instance of the structure bmi160_dev  */
	//static int8_t set_accel_any_motion_int(struct bmi160_int_settg *int_config, struct bmi160_dev *dev)

        printf("bmi160_set_int_config(any_motion) status:%d\n", rslt);
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
        rslt = bmi160_set_int_config(&int_config, ctx); /* sensor is an instance of the structure bmi160_dev */
        printf("bmi160_set_int_config(tap disable) status:%d\n", rslt);
    }

    return rslt;
}

int bmi160_open(struct bmi160_dev *ctx)
{
  int8_t check;
  int rslt;

  /* I2C setup */
  ctx->write = bmi160_write_to_reg;
  ctx->read = bmi160_read_from_reg;
  ctx->delay_ms = bmi160_delay;

  /* set correct i2c address */
  ctx->id = BMI160_DEV_ADDR;
  ctx->intf = BMI160_I2C_INTF;

  check = bmi160_init(ctx);

  if (check == BMI160_OK)
  {
    NRF_LOG_INFO("BMI160 initialization success !\n");
    NRF_LOG_INFO("Chip ID 0x%X\n", ctx->chip_id);
  }
  else
  {
    NRF_LOG_INFO("BMI160 initialization failure !\n");
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
  NRF_LOG_INFO("res %d\n", rslt);
  return 0;
}

int get_motion_status(struct bmi160_dev *sensor)
{
  int rslt;
  union bmi160_int_status int_status;

  /* Read interrupt status */
  memset(int_status.data, 0x00, sizeof(int_status.data));
  rslt = bmi160_get_int_status(BMI160_INT_STATUS_ALL, &int_status, sensor);

  /* Enters only if the obtained interrupt is single-tap */
  if (rslt == BMI160_OK)
  {
    /* Enters only if the obtained interrupt is single-tap */
    if (int_status.bit.anym)
    {
      NRF_LOG_INFO("Anymotion int_status:0x%x x %d y %d z %d\n",
		   int_status.data[0],
		   int_status.bit.anym_first_x,
		   int_status.bit.anym_first_y,
		   int_status.bit.anym_first_z);
      return 1;
    }
  }

  return 0;
}

void clear_alarm_busy(uint8_t unused)
{
  (void)unused;
  alarm_busy = 0;
  NRF_LOG_INFO("clear_alarm_busy()\n");
}

void send_powerline_fail(uint8_t param)
{
  send_toogle_req(param);
  NRF_LOG_INFO("send_powerline_fail(%d)\n", param);
}



MAIN()
{
  ARGV_UNUSED;

  log_init();
  NRF_LOG_INFO("LOG INITIATED");

  twi_init();
  NRF_LOG_INFO("TWI INITIATED");

  struct bmi160_dev sensor;
  int check_bmi;

  check_bmi = bmi160_open(&sensor);
  if (check_bmi == -1){
    NRF_LOG_INFO("\nError occured, while openning sensor as i2c slave\n");
  }
  check_bmi = set_tap_config(&sensor, BMI160_ENABLE);

  if (check_bmi != BMI160_OK)
  {
    NRF_LOG_INFO("\n set_tap_config() error! \n");
  }

  ZB_SET_TRACE_OFF();
  ZB_SET_TRAF_DUMP_OFF();

  ZB_INIT("on_off_switch_zed");

  zb_set_long_address(g_ed_addr);
  zb_set_network_ed_role(1l<<21);
   zb_set_nvram_erase_at_start(ZB_TRUE);

  zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
  zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));
  //zb_set_rx_on_when_idle(ZB_FALSE);

  /****************** Register Device ********************************/
  /** [REGISTER] */
  ZB_AF_REGISTER_DEVICE_CTX(&on_off_switch_ctx);
  ZB_AF_SET_ENDPOINT_HANDLER(ZB_SWITCH_ENDPOINT, zcl_specific_cluster_cmd_handler);
  /** [REGISTER] */

  if (zboss_start() != RET_OK)
  {
    TRACE_MSG(TRACE_ERROR, "zboss_start failed", (FMT__0));
    NRF_LOG_INFO("zboss_start faied");

  }
  else
  {
    NRF_LOG_INFO("zboss_start success!");
    while(1)
    {
      zboss_main_loop_iteration();
      if(get_motion_status(&sensor) != 0)
      {
	if(!alarm_busy)
	{
	  alarm_busy = 1;
	  ZB_SCHEDULE_ALARM(clear_alarm_busy, 0, 1 * ZB_TIME_ONE_SECOND);
	  ZB_GET_OUT_BUF_DELAYED(send_powerline_fail);
	}
      }
      UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
  }

  TRACE_DEINIT();

  MAIN_RETURN(0);
}

static zb_bool_t finding_binding_cb(zb_int16_t status,
                                    zb_ieee_addr_t addr,
                                    zb_uint8_t ep,
                                    zb_uint16_t cluster)
{
  TRACE_MSG(TRACE_ZCL1, "finding_binding_cb status %d addr " TRACE_FORMAT_64 " ep %hd cluster %d",
            (FMT__D_A_H_D, status, TRACE_ARG_64(addr), ep, cluster));
  return ZB_TRUE;
}

zb_uint8_t zcl_specific_cluster_cmd_handler(zb_uint8_t param)
{
  zb_buf_t *zcl_cmd_buf = (zb_buf_t *)ZB_BUF_FROM_REF(param);
  zb_zcl_parsed_hdr_t *cmd_info = ZB_GET_BUF_PARAM(zcl_cmd_buf, zb_zcl_parsed_hdr_t);
  zb_bool_t unknown_cmd_received = ZB_TRUE;

  TRACE_MSG(TRACE_ZCL1, "> zcl_specific_cluster_cmd_handler %i", (FMT__H, param));
  TRACE_MSG(TRACE_ZCL3, "payload size: %i", (FMT__D, ZB_BUF_LEN(zcl_cmd_buf)));

  if (cmd_info->cmd_direction == ZB_ZCL_FRAME_DIRECTION_TO_CLI)
  {
    if (cmd_info->cmd_id == ZB_ZCL_CMD_DEFAULT_RESP)
    {
      unknown_cmd_received = ZB_FALSE;

      cmd_in_progress = ZB_FALSE;

      zb_free_buf(zcl_cmd_buf);
    }
  }

  TRACE_MSG(TRACE_ZCL1, "< zcl_specific_cluster_cmd_handler %i", (FMT__H, param));
  return ! unknown_cmd_received;
}


zb_void_t send_toogle_req(zb_uint8_t param)
{
  zb_buf_t *buf = ZB_BUF_FROM_REF(param);
  zb_uint16_t addr = 0;

  ZB_ASSERT(param);

  if (ZB_JOINED() /* && !cmd_in_progress */)
  {
    cmd_in_progress = ZB_TRUE;

    /* Dst addr and endpoint are unknown; command will be sent via binding */
    ZB_ZCL_ON_OFF_SEND_TOGGLE_REQ(
      buf,
      addr,
      ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
      0,
      ZB_SWITCH_ENDPOINT,
      ZB_AF_HA_PROFILE_ID,
      ZB_FALSE, NULL);
  }
  else
  {
    zb_free_buf(buf);
  }
}

zb_void_t button_press_handler(zb_uint8_t param)
{
  if (!param)
  {
    /* Button is pressed, get buffer for outgoing command */
    ZB_GET_OUT_BUF_DELAYED(button_press_handler);
  }
  else
  {
    send_toogle_req(param);
#ifndef ZB_USE_BUTTONS
    /* Do not have buttons in simulator - just start periodic on/off sending */
    ZB_SCHEDULE_ALARM(button_press_handler, 0, 7 * ZB_TIME_ONE_SECOND);
#endif
  }
}

zb_void_t test_leave_nwk(zb_uint8_t param)
{
  TRACE_MSG(TRACE_ERROR, ">> test_leave_nwk param %hd", (FMT__H, param));

  /* We are going to leave */
  if (!param)
  {
    ZB_GET_OUT_BUF_DELAYED(test_leave_nwk);
  }
  else
  {
    zb_buf_t *buf = ZB_BUF_FROM_REF(param);
    zb_zdo_mgmt_leave_param_t *req_param;

    req_param = ZB_GET_BUF_PARAM(buf, zb_zdo_mgmt_leave_param_t);
    ZB_BZERO(req_param, sizeof(zb_zdo_mgmt_leave_param_t));

    /* Set dst_addr == local address for local leave */
    req_param->dst_addr = ZB_PIBCACHE_NETWORK_ADDRESS();
    zdo_mgmt_leave_req(param, NULL);
  }

  TRACE_MSG(TRACE_ERROR, "<< test_leave_nwk", (FMT__0));
}

zb_void_t test_restart_join_nwk(zb_uint8_t param)
{
  TRACE_MSG(TRACE_ERROR, "test_restart_join_nwk %hd", (FMT__H, param));
  if (param == ZB_NWK_LEAVE_TYPE_RESET)
  {
    bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
  }
}

zb_void_t test_leave_and_join(zb_uint8_t param)
{
  TRACE_MSG(TRACE_ERROR, ">> test_leave_and_join param %hd", (FMT__H, param));
  if (ZB_JOINED())
  {
    test_leave_nwk(param);
  }
  else
  {
    test_restart_join_nwk(ZB_NWK_LEAVE_TYPE_RESET);
    if (param)
    {
      zb_free_buf(ZB_BUF_FROM_REF(param));
    }
  }
  TRACE_MSG(TRACE_ERROR, "<< test_leave_and_join", (FMT__0));
}

void permit_joining_cb(zb_uint8_t param)
{
  TRACE_MSG(TRACE_ERROR, "permit joining done", (FMT__0));
  zb_free_buf(ZB_BUF_FROM_REF(param));
}

void zboss_signal_handler(zb_uint8_t param)
{
  zb_zdo_app_signal_hdr_t *sg_p = NULL;
  zb_zdo_app_signal_type_t sig = zb_get_app_signal(param, &sg_p);
  zb_buf_t *buf;
  zb_zdo_mgmt_permit_joining_req_param_t *req_param;

#if 0
  /* Now register handlers for buttons */
  for (int i = 0; i < 5; ++i)
  {
    zb_button_register_handler(i, 0, button_press_handler);
  }
#endif

  if (ZB_GET_APP_SIGNAL_STATUS(param) == 0)
  {
    switch(sig)
    {
//! [signal_first]
      case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        TRACE_MSG(TRACE_APP1, "Device STARTED OK", (FMT__0));
        bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
#ifdef TEST_APS_FRAGMENTATION
        ZB_SCHEDULE_ALARM(send_frag_data, 0, ZB_TIME_ONE_SECOND / 2);
#endif

        buf = ZB_GET_OUT_BUF();
        if (!buf)
        {
          TRACE_MSG(TRACE_APP1, "no buffer available", (FMT__0));
          break;
        }

        /* Example: send permit join request to close network */
        /** [zb_zdo_mgmt_permit_joining_req] */
        req_param = ZB_GET_BUF_PARAM(buf, zb_zdo_mgmt_permit_joining_req_param_t);

        req_param->dest_addr = 0xfffc;
        req_param->permit_duration = 0;
        req_param->tc_significance = 1;

        zb_zdo_mgmt_permit_joining_req(ZB_REF_FROM_BUF(buf), permit_joining_cb);
        /** [zb_zdo_mgmt_permit_joining_req] */

        break;
//! [signal_first]
//! [signal_reboot]
      case ZB_BDB_SIGNAL_DEVICE_REBOOT:
        TRACE_MSG(TRACE_APP1, "Device RESTARTED OK", (FMT__0));
#if 0
        /* Do not have buttons in simulator - just start periodic on/off sending */
        cmd_in_progress = ZB_FALSE;
        ZB_SCHEDULE_ALARM_CANCEL(button_press_handler, ZB_ALARM_ANY_PARAM);
        ZB_SCHEDULE_ALARM(button_press_handler, 0, 7 * ZB_TIME_ONE_SECOND);
#endif
        break;
//! [signal_reboot]
      case ZB_BDB_SIGNAL_STEERING:
        TRACE_MSG(TRACE_APP1, "Successfull steering, start f&b initiator", (FMT__0));
        zb_bdb_finding_binding_initiator(ZB_SWITCH_ENDPOINT, finding_binding_cb);
        break;

      case ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
      {
        TRACE_MSG(TRACE_APP1, "Finding&binding done", (FMT__0));
#if 0
        /* Do not have buttons in simulator - just start periodic on/off sending */
        cmd_in_progress = ZB_FALSE;
        ZB_SCHEDULE_ALARM_CANCEL(button_press_handler, ZB_ALARM_ANY_PARAM);
        ZB_SCHEDULE_ALARM(button_press_handler, 0, 7 * ZB_TIME_ONE_SECOND);
#endif
      }
      break;

      case ZB_ZDO_SIGNAL_LEAVE:
      {
        zb_zdo_signal_leave_params_t *leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(sg_p, zb_zdo_signal_leave_params_t);
        test_restart_join_nwk(leave_params->leave_type);
      }
      break;

      case ZB_COMMON_SIGNAL_CAN_SLEEP:
      {
        zb_zdo_signal_can_sleep_params_t *can_sleep_params = ZB_ZDO_SIGNAL_GET_PARAMS(sg_p, zb_zdo_signal_can_sleep_params_t);
        TRACE_MSG(TRACE_ERROR, "Can sleep for %ld ms", (FMT__L, can_sleep_params->sleep_tmo));
#ifdef ZB_USE_SLEEP
        zb_sleep_now();
#endif
        break;
      }

      case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
      {
        TRACE_MSG(TRACE_APP1, "Production config is ready", (FMT__0));
        break;
      }

      default:
        TRACE_MSG(TRACE_ERROR, "Unknown signal %hd, do nothing", (FMT__H, sig));
    }
  }
  else if (sig == ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY)
  {
    TRACE_MSG(TRACE_APP1, "Production config is not present or invalid", (FMT__0));
  }
  else
  {
    TRACE_MSG(TRACE_ERROR, "Device started FAILED status %d", (FMT__D, ZB_GET_APP_SIGNAL_STATUS(param)));
    ZB_SCHEDULE_ALARM(test_leave_and_join, 0, ZB_TIME_ONE_SECOND);
  }

  if (param)
  {
    zb_free_buf(ZB_BUF_FROM_REF(param));
  }
}
