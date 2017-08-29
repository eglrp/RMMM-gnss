/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/
/********************************************//**
 * \file svc_sdi.h
 * \brief This module provides the API for SDI svc_mcu.
 ***********************************************/

#ifndef SVC_SDI_H
#define SVC_SDI_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld_sdi.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   svc_sdi_init            ( gpOS_partition_t *partition, tU32 bus_id);
extern void           svc_sdi_refresh         ( tU32 bus_speed);

extern gpOS_error_t   svc_sdi_open_port       ( tUInt sdi_port, gpOS_interrupt_priority_t irq_pri);
extern gpOS_error_t   svc_sdi_read            ( tUInt sdi_port, tUInt sector_start, tUInt sector_number, tUInt sector_size, tU8 *buffer_ptr);
extern gpOS_error_t   svc_sdi_write           ( tUInt sdi_port, tUInt sector_start, tUInt sector_number, tUInt sector_size, tU8 *buffer_ptr);
extern boolean_t      svc_sdi_is_port_busy    ( tUInt sdi_port);

#endif /* SVC_SDI_H */
