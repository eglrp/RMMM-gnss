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

#ifndef SVC_SSP_H
#define SVC_SSP_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld_ssp.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define SVC_SSP_AVAIL_PORTS        6

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct svc_ssp_com_handler_s svc_ssp_com_handler_t;

//typedef struct svc_spi_comconfig_s svc_spi_comconfig_t;
typedef void (*svc_ssp_hook_t)( void *);
typedef void *svc_ssp_hook_param_t;

typedef struct svc_ssp_comconfig_s
{
    tU32                           out_clk;
    LLD_SSP_DataSizeTy             data_size;          /**< size of data elements (4 to 32 bits)         */
    LLD_SSP_ClkPhaseTy             clk_phase;          /**< Motorola SPI interface Clock phase           */
    LLD_SSP_ClkPolarityTy          clk_pol;            /**< Motorola SPI interface Clock polarity        */
    LLD_SSP_MicrowireCtrlTy        ctrl_len;           /**< Microwire interface: Control length          */
    LLD_SSP_MicrowireWaitStatelTy  wait_state;         /**< Microwire interface: Wait state              */
    LLD_SSP_DuplexTy               duplex;             /**< Microwire interface: Full/Half duplex        */
    svc_ssp_hook_t                 pre_cb;
    svc_ssp_hook_param_t           pre_cb_param;
    svc_ssp_hook_t                 post_cb;
    svc_ssp_hook_param_t           post_cb_param;
}svc_ssp_comconfig_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t             svc_ssp_init              ( gpOS_partition_t *partition, tU32 bus_id);
extern void                     svc_ssp_refresh           ( tU32 bus_speed);

extern gpOS_error_t             svc_ssp_open_port         ( tUInt ssp_port, gpOS_interrupt_priority_t irq_pri);

extern svc_ssp_com_handler_t *  svc_ssp_create_com_config ( tUInt ssp_port, LLD_SSP_InterfaceTy ssp_mode, svc_ssp_comconfig_t ssp_init_comconfig);
extern svc_ssp_com_handler_t *  svc_ssp_create_com        ( tUInt ssp_port, LLD_SSP_InterfaceTy ssp_mode, tU32 out_clk, LLD_SSP_DataSizeTy data_size, svc_ssp_hook_t pre_cb, svc_ssp_hook_param_t pre_cb_param, svc_ssp_hook_t post_cb, svc_ssp_hook_param_t post_cb_param);
extern gpOS_error_t             svc_ssp_com_setmode       ( svc_ssp_com_handler_t *ssp_com_hdlr, LLD_SSP_InterfaceTy ssp_mode);
extern gpOS_error_t             svc_ssp_com_getmode       ( svc_ssp_com_handler_t *ssp_com_hdlr, LLD_SSP_InterfaceTy *ssp_mode_ptr);
extern gpOS_error_t             svc_ssp_write             ( svc_ssp_com_handler_t *ssp_com_hdlr, tVoid *out_buf, tU32 len, tVoid *in_buf, gpOS_clock_t *timeout);
extern boolean_t                svc_ssp_is_busy           ( svc_ssp_com_handler_t *ssp_com_hdlr);

extern void                     svc_ssp_lock              ( svc_ssp_com_handler_t *ssp_com_hdlr);
extern void                     svc_ssp_release           ( svc_ssp_com_handler_t *ssp_com_hdlr);

#endif /* SVC_SSP_H */
