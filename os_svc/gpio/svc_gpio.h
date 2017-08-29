/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/
/********************************************//**
 * \file svc_gpio.h
 * \brief This module provides the API for GPIO svc_mcu.
 ***********************************************/

#ifndef SVC_GPIO_H
#define SVC_GPIO_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld_gpio.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief Define as tU32 the svc_gpio_id_t type.
//typedef tS32 svc_gpio_id_t;

typedef enum
{
#if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  SVC_GPIO_PORT_0  = 0,
  SVC_GPIO_PORT_1  = 1,
  SVC_GPIO_PORT_2  = 2,
  SVC_GPIO_PORT_3  = 3,
#endif

#if defined( __STA8088__) || defined( __STA8090__ )
  SVC_GPIO_PORT_0  = 0,
  SVC_GPIO_PORT_1  = 1,
#endif
} svc_gpio_port_id_t;

typedef enum
{
  SVC_GPIO_PIN_0   = 0U,
  SVC_GPIO_PIN_1   = 1U,
  SVC_GPIO_PIN_2   = 2U,
  SVC_GPIO_PIN_3   = 3U,
  SVC_GPIO_PIN_4   = 4U,
  SVC_GPIO_PIN_5   = 5U,
  SVC_GPIO_PIN_6   = 6U,
  SVC_GPIO_PIN_7   = 7U,
  SVC_GPIO_PIN_8   = 8U,
  SVC_GPIO_PIN_9   = 9U,
  SVC_GPIO_PIN_10  = 10U,
  SVC_GPIO_PIN_11  = 11U,
  SVC_GPIO_PIN_12  = 12U,
  SVC_GPIO_PIN_13  = 13U,
  SVC_GPIO_PIN_14  = 14U,
  SVC_GPIO_PIN_15  = 15U,
  SVC_GPIO_PIN_16  = 16U,
  SVC_GPIO_PIN_17  = 17U,
  SVC_GPIO_PIN_18  = 18U,
  SVC_GPIO_PIN_19  = 19U,
  SVC_GPIO_PIN_20  = 20U,
  SVC_GPIO_PIN_21  = 21U,
  SVC_GPIO_PIN_22  = 22U,
  SVC_GPIO_PIN_23  = 23U,
  SVC_GPIO_PIN_24  = 24U,
  SVC_GPIO_PIN_25  = 25U,
  SVC_GPIO_PIN_26  = 26U,
  SVC_GPIO_PIN_27  = 27U,
  SVC_GPIO_PIN_28  = 28U,
  SVC_GPIO_PIN_29  = 29U,
  SVC_GPIO_PIN_30  = 30U,
  SVC_GPIO_PIN_31  = 31U,
} svc_gpio_pin_id_t;

typedef void (*svc_gpio_isr_t)( const void *);

typedef enum
{
  SVC_GPIO_RISING_AND_FALLING_EDGE = 1,
  SVC_GPIO_RISING_EDGE             = 2,
  SVC_GPIO_FALLING_EDGE            = 3,
  SVC_GPIO_EDGE_COUNT              = 4,
} svc_gpio_edge_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   svc_gpio_init              ( gpOS_partition_t *part);
extern gpOS_error_t   svc_gpio_open_port         ( svc_gpio_port_id_t gpio_id, gpOS_interrupt_priority_t irq_pri);
extern gpOS_error_t   svc_gpio_interrupt_install ( svc_gpio_port_id_t port_id, svc_gpio_pin_id_t pin_id, svc_gpio_isr_t gpio_isr);
extern gpOS_error_t   svc_gpio_pin_enable        ( svc_gpio_port_id_t port_id, svc_gpio_pin_id_t pin_id);
extern gpOS_error_t   svc_gpio_interrupt_enable  ( svc_gpio_port_id_t port_id, svc_gpio_pin_id_t pin_id, svc_gpio_edge_t gpio_edge);
extern gpOS_error_t   svc_gpio_interrupt_disable ( svc_gpio_port_id_t port_id, svc_gpio_pin_id_t pin_id);

#endif /* SVC_GPIO_H */
