/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/
/********************************************//**
 * \file svc_mtu.h
 * \brief This module provides the API for MTU svc_mcu
 ***********************************************/

#ifndef SVC_MTU_H
#define SVC_MTU_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "lld_mtu.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
typedef struct svc_mtu_cfg_s
{
  LLD_MTU_ModeTy        mode;
  LLD_MTU_PrescalerTy   prescaler;
  LLD_MTU_SizeTy        size;
  tU32                  load_value;
  tU32                  ext_freq;
} svc_mtu_cfg_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t     svc_mtu_init                 ( gpOS_partition_t *partition, const tU32 bus_id);
extern gpOS_error_t     svc_mtu_open_port            ( const tU32 mtu_port);
extern gpOS_error_t     svc_mtu_start                ( const tU32 mtu_port);
extern gpOS_error_t     svc_mtu_stop                 ( const tU32 mtu_port);
extern gpOS_error_t     svc_mtu_port_config          ( const tU32 mtu_port, const svc_mtu_cfg_t *cfg_ptr);
extern gpOS_error_t     svc_mtu_port_set_irq         ( const tU32 mtu_port, gpOS_interrupt_priority_t priority, gpOS_interrupt_callback_t mtu_callback, const void *mtu_callback_param);
extern gpOS_error_t     svc_mtu_port_disable_irq     ( const tU32 mtu_port);
extern gpOS_error_t     svc_mtu_port_get_value       ( const tU32 mtu_port, tU32 *value_ptr);
extern gpOS_error_t     svc_mtu_port_get_ticks       ( const tU32 mtu_port, tU32 *ticks_ptr);
extern gpOS_error_t     svc_mtu_port_set_load_value  ( const tU32 mtu_port, tU32 load_value);
extern gpOS_error_t     svc_mtu_port_get_phy_params  ( const tU32 mtu_port, LLD_MTU_IPTy *mtu_phy_bank_ptr, LLD_MTU_IdTy *mtu_phy_id_ptr);
extern gpOS_error_t     svc_mtu_port_set_mode        ( const tU32 mtu_port, const LLD_MTU_ModeTy mode);

#endif /* __SVC_MTU_H__ */
