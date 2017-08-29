/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/
/********************************************//**
 * \file svc_fsmc.h
 * \brief This module provides the API for FSMC svc_mcu.
 ***********************************************/

#ifndef SVC_SQI_H
#define SVC_SQI_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld_sqi_ctrl.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   svc_sqi_init              ( gpOS_partition_t *part);
extern gpOS_error_t   svc_sqi_create_region     ( tUInt start_addr, tUInt size, tU8 spm_conf);
extern gpOS_error_t   svc_sqi_reset_region      ( void *start_addr);
extern gpOS_error_t   svc_sqi_unlock            ( void *dest_addr, tUInt size);
extern gpOS_error_t   svc_sqi_lock              ( void *dest_addr, tUInt size);
extern gpOS_error_t   svc_sqi_read              ( void *dest_addr, void *src_addr, tUInt size);
extern gpOS_error_t   svc_sqi_write             ( void *dest_addr, void *src_addr, tUInt size);
extern gpOS_error_t   svc_sqi_erase             ( void *dest_addr, tUInt size, boolean_t wait_for_completion);
extern gpOS_error_t   svc_sqi_subsector_erase   ( void *dest_addr, tUInt size, boolean_t wait_for_completion);
extern gpOS_error_t   svc_sqi_reset             ( void);
extern gpOS_error_t   svc_sqi_power_down        ( void);
extern gpOS_error_t   svc_sqi_set_sw_protection ( tU8 spm);
extern gpOS_error_t   svc_sqi_get_sw_protection ( tU8 *spm);

#endif /* __SVC_SQI_H__ */
