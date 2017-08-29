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

#ifndef SVC_MSP_H
#define SVC_MSP_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld_msp.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct svc_msp_com_handler_s svc_msp_com_handler_t;

typedef void *  svc_msp_hook_param_t;

typedef void    (*svc_msp_hook_t)  ( void *);


typedef struct svc_msp_comconfig_s
{
  tU32                    out_clk;
  LLD_MSP_Bits4ElemTy     data_size;
  LLD_MSP_SPIClockModeTy  spi_clk_mode;
  tU16                    frame_period;
  svc_msp_hook_t          pre_cb;
  svc_msp_hook_param_t    pre_cb_param;
  svc_msp_hook_t          post_cb;
  svc_msp_hook_param_t    post_cb_param;
}svc_msp_comconfig_t;

typedef enum svc_msp_protocol_e {
  SVC_MSP_PROTOCOL_SPI,
  SVC_MSP_PROTOCOL_I2S,
} svc_msp_protocol_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t             svc_msp_init              ( gpOS_partition_t *partition, tU32 bus_speed);
extern gpOS_error_t             svc_msp_open_port         ( tUInt msp_port, gpOS_interrupt_priority_t irq_pri);
extern svc_msp_com_handler_t *  svc_msp_create_com_config ( tUInt msp_port, svc_msp_protocol_t msp_mode, svc_msp_comconfig_t msp_init_comconfig);
extern svc_msp_com_handler_t *  svc_msp_create_com        ( tUInt msp_port, svc_msp_protocol_t msp_mode, tU32 out_clk, LLD_MSP_Bits4ElemTy data_size, svc_msp_hook_t pre_cb, svc_msp_hook_param_t pre_cb_param, svc_msp_hook_t post_cb, svc_msp_hook_param_t post_cb_param);
extern gpOS_error_t             svc_msp_write             ( svc_msp_com_handler_t *msp_com_hdlr, tVoid *out_buf, tU32 len, tVoid *in_buf, gpOS_clock_t *timeout);

#endif /* SVC_MSP_H */
