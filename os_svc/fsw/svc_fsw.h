/*******************************************************************************
 *                            (C) 2013 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/
/********************************************//********************
 * \file os20_fsw.h
 * \brief This module provides the API for runtime switch feature.
 *****************************************************************/

#ifndef SVC_FSW_H
#define SVC_FSW_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"
#include "svc_mcu.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef void (*svc_fsw_set_clkcfg_cb_t)   ( svc_mcu_clkcfg_t *cfg_ptr, tBool increase_freq);

typedef void (*svc_fsw_set_pwrcfg_cb_t)   ( svc_mcu_corefreqcfg_t cfg_ptr);

typedef struct svc_fsw_cfg_s
{
  svc_mcu_clkcfg_t *        clkcfg_table;
  svc_mcu_corefreqcfg_t     clkcfg_start;

  svc_fsw_set_clkcfg_cb_t   set_clkcfg_cb;
  svc_fsw_set_pwrcfg_cb_t   set_pwrcfg_cb;

  tU8                       mclkid;
  svc_mcu_corefreqcfg_t     pwrlowfreqid;

  tU32                      freqratiofactor;
  tU32                      maxoscifreq;
  tU32                      systimer_extfreq;
} svc_fsw_cfg_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
extern gpOS_error_t           svc_fsw_init                ( gpOS_partition_t* part, const svc_fsw_cfg_t *cfg_ptr);
extern tVoid                  svc_fsw_setextfreq          ( tUInt);
extern svc_fsw_cfg_t *        svc_fsw_get_fswcfg          ( tVoid);
extern svc_mcu_clkcfg_t *     svc_fsw_get_clkcfg          ( tVoid);
extern gpOS_error_t           svc_fsw_set_corefreq        ( svc_mcu_corefreqcfg_t config , boolean_t forced_mode);
extern tVoid                  svc_fsw_save_configuration  ( svc_mcu_corefreqcfg_t config_number);
extern svc_mcu_corefreqcfg_t  svc_fsw_get_configuration   ( tVoid);

#endif /* __OS20_FSW_H__ */
