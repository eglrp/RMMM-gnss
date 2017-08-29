/**
 * @file    os20togpOS_timei.h
 * @brief   OS20 time internal definitions and macros.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_TIMEI_H
#define OS20TOGPOS_TIMEI_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define OS20_TIMEOUT_INFINITY   gpOS_TIMEOUT_INFINITY
#define OS20_TIMEOUT_IMMEDIATE  gpOS_TIMEOUT_IMMEDIATE

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

#define os20_clock_t    gpOS_clock_t

/*****************************************************************************
   exported variables
*****************************************************************************/

extern os20_clock_t  _ST_TimeoutImmediate;
extern os20_clock_t  _ST_TimeoutInfinity;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* OS20TOGPOS_TIMEI_H */
