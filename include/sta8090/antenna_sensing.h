/********************************************//**
 * \file antenna_sensing.h
 * \brief This module provides the API for antenna sensing module.
 ***********************************************/

#ifndef ANTENNA_SENSING_H
#define ANTENNA_SENSING_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"
#include "gpOS.h"
#include "lld_gpio.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define ANTENNA_SENSING_DIAG_ON             0
#define ANTENNA_SENSING_SWITCH_CTRL         1
#define ANTENNA_SENSING_EXT_DIAG_SHORT      2
#define ANTENNA_SENSING_EXT_DIAG_OPEN       3
#define ANTENNA_SENSING_MAX_INOUT_CTRL_PINS 4

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum antenna_status_e
{
  ANTENNA_SENSING_STATUS_NORMAL,
  ANTENNA_SENSING_STATUS_OPEN,
  ANTENNA_SENSING_STATUS_SHORT,
  ANTENNA_SENSING_STATUS_UNINIT
} antenna_status_t;

typedef enum antenna_sensing_mode_e
{
  ANTENNA_SENSING_MODE_OFF      = 0,
  ANTENNA_SENSING_MODE_ON       = 1,
} antenna_sensing_mode_t;

typedef enum antenna_sensing_control_e
{
  ANTENNA_SENSING_CONTROL_RF = 0,
  ANTENNA_SENSING_CONTROL_ADC = 1,
  ANTENNA_SENSING_CONTROL_GPIO = 2
} antenna_sensing_control_t;

typedef struct antenna_sensing_config_e
{
  antenna_sensing_control_t   ctrl;
  tU16 clk_div;
} antenna_sensing_config_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t           antenna_sensing_init                  ( gpOS_partition_t *part, antenna_sensing_config_t cfg );
extern tVoid                  antenna_sensing_set_mode              ( antenna_sensing_mode_t mode);
extern gpOS_error_t           antenna_sensing_set_gpio              ( tInt index, LLD_GPIO_ChanTy gpio, LLD_GPIO_ModeTy);
extern gpOS_error_t           antenna_sensing_set_gpio_active_level ( tInt index, LLD_GPIO_StateTy gpio);
extern tVoid                  antenna_sensing_configure_gpio        ( tVoid);
extern tVoid                  antenna_sensing_set_adc_thresholds    ( tU32, tU32 );
extern gpOS_error_t           antenna_sensing_set_adc_input         ( tInt, tInt);
//extern tVoid                  antenna_status_update                 ( tU32 threshold_min, tU32 threshold_max);
extern antenna_status_t       antenna_get_status                    ( tVoid);
extern tBool                  antenna_get_switch_status             ( tVoid);
extern tVoid                  antenna_sensing_set_switch            ( tBool antenna_switch);
extern tVoid                  antenna_sensing_switch                ( tVoid);
extern tVoid                  antenna_sensing_update_status         ( tVoid);
extern antenna_sensing_mode_t antenna_sensing_get_mode              ( tVoid);
#endif /* ANTENNA_SENSING_H */
