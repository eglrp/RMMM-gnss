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

#ifndef SVC_FSMC_H
#define SVC_FSMC_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld_fsmc_ctrl.h"
#include "lld_fsmc_nor.h"
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

extern gpOS_error_t   svc_fsmc_nor_init          ( gpOS_partition_t *part);
extern gpOS_error_t   svc_fsmc_nor_create_region ( LLD_FSMC_SRAMNORBankTy bank, tUInt start_addr, tUInt size);
extern gpOS_error_t   svc_fsmc_nor_reset_region  ( void *addr, tUInt size);
extern gpOS_error_t   svc_fsmc_nor_unlock        ( void *addr, tUInt size);
extern gpOS_error_t   svc_fsmc_nor_lock          ( void *addr, tUInt size);
extern gpOS_error_t   svc_fsmc_nor_erase         ( void *addr, tUInt size, boolean_t wait_for_completion);
extern gpOS_error_t   svc_fsmc_nor_write         ( void *dest_addr, void *src_addr, tUInt size);

#endif /* SVC_FSMC_H */
