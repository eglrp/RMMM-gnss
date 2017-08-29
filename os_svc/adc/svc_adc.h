/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/
/********************************************//**
 * \file svc_adc.h
 * \brief This module provides the API for ADC svc_mcu.
 ***********************************************/

#ifndef SVC_ADC_H
#define SVC_ADC_H

/*****************************************************************************
   includes
*****************************************************************************/

#if defined( __STA8088__ )
#include "lld_adc_sta8088.h"
#endif
#if defined( __STA8090__ )
#include "lld_adc_sta8090.h"
#endif
#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/********************************************//**
 * \brief ADC Channel handler
 ***********************************************/
typedef struct svc_adc_chan_handler_s
{
  tUInt                 adc_chan_id;      /**< Chan ID */

  gpOS_semaphore_t *    done_sem;         /**< semaphore to signal end of transfer */
  void *                buf_ptr;          /**< Pointer to data buffer */
  tU32                  buf_ptr_pos;      /**< Position in data buffer */
  tU32                  buf_average_len;  /**< Length of I/O request */

  tU32                  out_data;         /**< Output data averaged on one buffer page (256 samples) or on the max filtering length
                                               compatible with the number of masked channels */

  // ADC config infos
  #if defined( __STA8088__ )
  LLD_ADC_OffsetPageTy  offset_page;      /**< Buffer page read selector parameter */
  #elif defined( __STA8090__)
  tBool                 irq_src;
  #endif
}svc_adc_chan_handler_t;

typedef enum svc_adc_chan_cfg_e
{
  ADC_8CHAN_AVAILABLE = 0,
  ADC_4CHAN_AVAILABLE = 1,
  ADC_2CHAN_AVAILABLE = 3,
  ADC_1CHAN_AVAILABLE = 7
} svc_adc_chan_cfg_t;

typedef enum svc_adc_mode_cfg_e
{
  ADC_NOINTERRUPT     = 0,
  ADC_INTERRUPT       = 1
} svc_adc_mode_cfg_t;

typedef struct svc_adc_int_mode_cfg_s
{
  svc_adc_mode_cfg_t    adc_int_mode;
  #if defined( __STA8090__)
  LLD_ADC_CHIRQSrcIdTy  adc_chan_irq_src;
  tU32                  Hth0;
  tU32                  Hth1;
  tU32                  Lth0;
  tU32                  Lth1;
  LLD_ADC_ChanSelTy     adc_chan_sel_switch;
  #endif
}svc_adc_int_mode_cfg_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   svc_adc_init  ( gpOS_partition_t *partition, tInt adc_port, gpOS_interrupt_priority_t irq_pri, svc_adc_chan_cfg_t num_chan, tU32 adc_clk_div, /*LLD_ADC_ChanSelTy adc_chan_sel_switch,*/ svc_adc_int_mode_cfg_t*  adc_functional_mode);
extern gpOS_error_t   svc_adc_read  ( tInt chan_id, tU32 len_average, tU32 *out_data, tVoid *out_buf);
#endif /* SVC_ADC_H */
