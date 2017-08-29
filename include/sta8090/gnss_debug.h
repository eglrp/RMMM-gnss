/*{{{  COMMENT Standard Header*/

/************************************************************************
COPYRIGHT (C) ST Microelectronics 1995,1996,1997,1998,1999
                   All Rights Reserved

Source file name : gp_debug.h
Author :           P Bagnall

Header file for the gps debug message routines

Date        Modification                                    Initials
----        ------------                                    --------
01.10.97    Created                                         PB

************************************************************************/

/*}}}  */

#ifndef GNSS_DEBUG_H
#define GNSS_DEBUG_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"


/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define DEBUG_MACRO(debug_string)          if(gnss_debug_mode != GNSS_DEBUG_OFF) {gnss_debug_msg debug_string;}
#define DEBUG_ERROR_MACRO(debug_string)    if(gnss_debug_mode != GNSS_DEBUG_OFF) {gnss_debug_msg(debug_string);}

#define DEBUG_BUFFER_SIZE 512

#define GNSS_DEBUG
#define GNSS_BIN_DEBUG
//#define BACKUP_DEBUG
//#define KF_DEBUG
//#define LMS_DEBUG
#define WAAS_DEBUG
#define WAAS_DEBUG_L1
//#define WAAS_DEBUG_L2
//#define WAAS_DEBUG_L3
//#define DR_DEBUG
//#define KF_5Hz_DEBUG
//#define RAM_ALLOC_DEBUG


#if (defined(UART_DEBUG) || defined(LINK_DEBUG) || defined(SDMMC_DEBUG) || defined(DR_DEBUG))

  #if (defined(GNSS_DEBUG))
    #define GPS_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define GPS_DEBUG_MSG(macro_string)
  #endif

  #if (defined(GNSS_BIN_DEBUG))
    #define GPS_DEBUG_BIN_DATA(header,data,size) gnss_debug_bin_data(header,(tChar *)data,size)
  #else
    #define GPS_DEBUG_BIN_DATA(header,data,size)
  #endif

  #if (defined(WAAS_DEBUG))
    #define WAAS_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define WAAS_DEBUG_MSG(macro_string)
  #endif

  #if (defined(WAAS_DEBUG_L1))
    #define WAAS_DEBUG_LEVEL1_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define WAAS_DEBUG_LEVEL1_MSG(macro_string)
  #endif

  #if (defined(WAAS_DEBUG_L2))
    #define WAAS_DEBUG_LEVEL2_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define WAAS_DEBUG_LEVEL2_MSG(macro_string)
  #endif

  #if (defined(WAAS_DEBUG_L3))
    #define WAAS_DEBUG_LEVEL3_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define WAAS_DEBUG_LEVEL3_MSG(macro_string)
  #endif

  #if (defined(TEST_DEBUG))
    #define TEST_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define TEST_DEBUG_MSG(macro_string)
  #endif

  #if (defined(NMEA_DEBUG))
    #define NMEA_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define NMEA_DEBUG_MSG(macro_string)
  #endif

  #if (defined(FIX_STATUS_DEBUG))
    #define FIX_STATUS_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define FIX_STATUS_DEBUG_MSG(macro_string)
  #endif

  #if (defined(KF_DEBUG))
    #define KF_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define KF_DEBUG_MSG(macro_string)
  #endif

  #if (defined(KF_5Hz_DEBUG))
    #define KF_5Hz_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define KF_5Hz_DEBUG_MSG(macro_string)
  #endif

  #if (defined(LMS_DEBUG))
    #define LMS_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define LMS_DEBUG_MSG(macro_string)
  #endif

  #if (defined(DGPS_DEBUG))
    #define DGPS_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define DGPS_DEBUG_MSG(macro_string)
  #endif

  #if (defined(BACKUP_DEBUG))
    #define BACKUP_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define BACKUP_DEBUG_MSG(macro_string)
  #endif

  #if (defined(RAM_ALLOC_DEBUG))
    #define RAM_ALLOC_DEBUG_MSG(macro_string) printf macro_string
  #else
    #define RAM_ALLOC_DEBUG_MSG(macro_string)
  #endif

  #if (defined(DR_DEBUG))
    #define DR_DEBUG_MSG(macro_string) DEBUG_MACRO(macro_string)
  #else
    #define DR_DEBUG_MSG(macro_string)
  #endif

  #ifdef LINK_ERROR
    #define ERROR_MSG(msg) {gnss_debug_msg(msg); debugbreak();}
  #else
    #define ERROR_MSG(msg) DEBUG_ERROR_MACRO(msg)
  #endif

#else
  #define GPS_DEBUG_MSG(macro_string)
  #define GPS_DEBUG_BIN_DATA(header,data,size)
  #define NMEA_DEBUG_MSG(macro_string)
  #define FIX_STATUS_DEBUG_MSG(macro_string)
  #define KF_DEBUG_MSG(macro_string)
  #define KF_5Hz_DEBUG_MSG(macro_string)
  #define LMS_DEBUG_MSG(macro_string)
  #define DGPS_DEBUG_MSG(macro_string)
  #define BACKUP_DEBUG_MSG(macro_string)
  #define ERROR_MSG(msg)
  #define WAAS_DEBUG_MSG(msg)
  #define WAAS_DEBUG_LEVEL1_MSG(msg)
  #define WAAS_DEBUG_LEVEL2_MSG(msg)
  #define WAAS_DEBUG_LEVEL3_MSG(msg)
  #define DR_DEBUG_MSG(macro_string)
  #define RAM_ALLOC_DEBUG_MSG(macro_string)
#endif

  #if (defined(_WIN32_WCE) && defined(OSA_DEBUG_MSG))
    #define OSA_DEBUG_MSG(macro_string) debug_msg macro_string
  #else
    #define OSA_DEBUG_MSG(macro_string)
  #endif

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum gnss_debug_mode_e
{
  GNSS_DEBUG_ON = 0,
  GNSS_DEBUG_OFF = 1,
  GNSS_DEBUG_LINK_ONLY = 2,
  GNSS_DEBUG_MASK = 0xF
} gnss_debug_mode_t;

typedef tU32    (*gnss_debug_writeout_t)    ( tChar *, tU32, gpOS_clock_t *);
typedef tU32    (*gnss_debug_inout_t)       ( tChar *, tU32, gpOS_clock_t *);

#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
   exported variables
*****************************************************************************/
extern gnss_debug_mode_t  gnss_debug_mode;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
extern gpOS_error_t      gnss_debug_init              ( gpOS_partition_t *part, const gnss_debug_mode_t, gnss_debug_writeout_t);
extern void              gnss_debug_msg               ( const tChar *, ...);
extern gnss_debug_mode_t gnss_debug_get_status        ( void);
extern void              gnss_debug_set_status        ( const gnss_debug_mode_t debug_mode);
extern void              gnss_debug_bin_data          ( const tChar *header,tChar * data, tInt size);
extern void              gnss_debug_send_msg_to_uart  ( tChar *, const tInt );

extern void              gnss_debug_direct_uart_init  ( tUInt );
extern void              gnss_debug_direct_uart_enable( tUInt );
extern void              gnss_debug_direct_write      ( tChar );

#ifdef __cplusplus
}
#endif
#endif /* _GNSS_DEBUG_H_ */
