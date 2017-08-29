/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/
/********************************************//**
 * \file svc_pwr.h
 * \brief This module provides the API for PWR svc_mcu.
 ***********************************************/

#ifndef SVC_PWR_H
#define SVC_PWR_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"
#include "gnss_defs.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef enum svc_pwr_startup_mode_e {
  SVC_PWR_STARTUP_POWER_ON,   /* Classic power on */
  SVC_PWR_STARTUP_WAKEUP_RTC, /* Standby wake-up by RTC */
  SVC_PWR_STARTUP_WAKEUP_PIN  /* Standby wake-up by PIN */
} svc_pwr_startup_mode_t;

typedef tU8 peripherallockid_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   svc_pwr_init                    ( gpOS_partition_t *partition);

/* Lowpower handling section */
extern gpOS_error_t   svc_pwr_set_lowpower_allowed    ( boolean_t status);
extern gpOS_error_t   svc_pwr_get_lowpower_allowed    ( boolean_t *status);


/* peripheral handling section */
extern gpOS_error_t   svc_pwr_peripherallock_register     ( peripherallockid_t *id );
extern void           svc_pwr_peripherallock_acquire      ( peripherallockid_t id);
extern void           svc_pwr_peripherallock_release      ( peripherallockid_t id);
extern void           svc_pwr_isr_peripherallock_release  ( peripherallockid_t id);

/* Lowpower handling section */
extern gpOS_error_t   svc_pwr_set_lowpower_allowed    ( boolean_t status);


/* Standby handling section */
extern gpOS_error_t   svc_pwr_force_standby           ( tU16 duration);
extern void           svc_pwr_enter_standby           ( void );
extern void           svc_pwr_pre_WFI                 ( void );
extern void           svc_pwr_post_WFI                ( void );
extern gpOS_error_t   svc_pwr_set_standby_allowed     ( boolean_t status);
extern boolean_t      svc_pwr_get_standby_allowed     ( void );
extern gpOS_error_t   svc_pwr_get_standby_status      ( boolean_t *status );
extern boolean_t      svc_pwr_IsStandbyWakeup         ( void );
extern boolean_t      svc_pwr_StandbyWakeupState      ( boolean_t * WakeUpRTC, boolean_t * WakeUpPin);
extern boolean_t      svc_pwr_get_timer_adjustment    ( gpOS_clock_t * StandbyDuration_rtcbase, gpOS_clock_t * VirtualPreviousOsTime );
extern void           svc_pwr_StartupTime             ( tUInt StartupTime );

extern svc_pwr_startup_mode_t   svc_pwr_StartupMode   ( void );

#endif /* SVC_PWR_H */
