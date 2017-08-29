/*****************************************************************************
   FILE:          defines.h
   PROJECT:       ARM GPS library
   SW PACKAGE:    Common Header
------------------------------------------------------------------------------
   DESCRIPTION:   Define with project scope.
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Developers:
      CS:   Fulvio Boggia
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
-------------+------+---------------------------------------------------------
 2005.09.07  |  FB  | Original version
*****************************************************************************/

#ifndef GPSLIB_COMMON_DEFINES_H
#define GPSLIB_COMMON_DEFINES_H

/************************************************************************
| defines (scope: global)
|-----------------------------------------------------------------------*/

#ifndef NULL
#ifdef __cplusplus
#define NULL      0
#else
#define NULL      ((tVoid*)0)
#endif
#endif

#ifndef SET
#define SET         1
#endif
#ifndef CLEAR
#define CLEAR       0
#endif

#ifndef TRUE
#define TRUE        ((boolean_t)(1U))
#endif
#ifndef true
#define true        ((boolean_t)(1U))
#endif
#ifndef FALSE
#define FALSE       ((boolean_t)(0U))
#endif
#ifndef false
#define false       ((boolean_t)(0U))
#endif


#if defined( __ARMCC_VERSION)
#define TOOLCHAIN_STRING "ARM"
#endif
#if defined( __GNUC__)
#define TOOLCHAIN_STRING "GNU"
#endif
#if defined( _WIN32_WCE)
#define TOOLCHAIN_STRING "Windows CE"
#endif
#if defined( _WIN32_EMU)
#define TOOLCHAIN_STRING "Emulator"
#endif

#if !defined( TOOLCHAIN_STRING)
#define TOOLCHAIN_STRING "UNKNOWN"
#endif

#endif  /* GPSLIB_COMMON_DEFINES_H */

/* End of file */
