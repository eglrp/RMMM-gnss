/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/

/********************************************//**
 * \file clibs.h
 * \brief This header provides retargetting APIs for C standard libs.
 ***********************************************/

#ifndef CLIBS_H
#define CLIBS_H

/*****************************************************************************
   includes
*****************************************************************************/

#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

// climits.h retargeting
#define _clibs_INT_MAX()      INT_MAX;

// stdio.h retargeting
#define _clibs_sprintf        sprintf
#define _clibs_sscanf         sscanf

// stdlib.h retargeting
#define _clibs_div_t          div_t
#define _clibs_abs            abs
#define _clibs_div            div
#define _clibs_malloc         malloc
#define _clibs_free           free

// string.h retargeting
#define _clibs_memcmp         memcmp
#define _clibs_memcpy         memcpy
#define _clibs_memmove        memmove
#define _clibs_memset         memset
#define _clibs_strchr         strchr
#define _clibs_strcpy         strcpy
#define _clibs_strncpy        strncpy
#define _clibs_strlen         strlen
#define _clibs_strncmp        strncmp

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* CLIBS_H */

