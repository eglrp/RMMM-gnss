/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/
/********************************************//**
 * \file svc_ssp.h
 * \brief This module provides the API for SSP svc_mcu.
 ***********************************************/

#ifndef SVC_I2C_H
#define SVC_I2C_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld_i2c.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef struct svc_i2c_com_handler_s svc_i2c_com_handler_t;

typedef void (*svc_i2c_hook_t)( void *);
typedef void *svc_i2c_hook_param_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t               svc_i2c_init             ( gpOS_partition_t *partition, tU32 bus_id);
extern gpOS_error_t               svc_i2c_open_port        ( tUInt i2c_port, gpOS_interrupt_priority_t irq_pri, tUInt slave_addr, LLD_I2C_BusCtrlModeTy start_mode);
extern svc_i2c_com_handler_t *    svc_i2c_create_com       ( tUInt i2c_port, LLD_I2C_SpeedModeTy speed_mode, tU16 address);
extern svc_i2c_com_handler_t *    svc_i2c_create_com_speed ( tUInt i2c_port, LLD_I2C_SpeedModeTy speed_mode, tU32 out_freq, tU16 address);
extern gpOS_error_t               svc_i2c_reset_port       ( tUInt i2c_port);
extern gpOS_error_t               svc_i2c_set_port_mode    ( tUInt i2c_port, LLD_I2C_BusCtrlModeTy mode);
extern gpOS_error_t               svc_i2c_write            ( svc_i2c_com_handler_t *i2c_com_hdlr, tU32 addr_start, tU32 addr_bytes, tU8 *out_buf, tU32 len, tU32 bytes_in_a_row, gpOS_clock_t *timeout);
extern gpOS_error_t               svc_i2c_read             ( svc_i2c_com_handler_t *i2c_com_hdlr, tU32 addr_start, tU32 addr_bytes, tU8 *in_buf, tU32 len, gpOS_clock_t *timeout);
extern gpOS_error_t               svc_i2c_reset            ( tUInt i2c_port);

#endif /* SVC_I2C_H */
