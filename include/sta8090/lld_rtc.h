//!
//!   \file     lld_rtc.h
//!   \brief    <i><b>RTC low level driver header file</b></i>
//!   \author   Fulvio Boggia
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_RTC
#define LLD_RTC

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
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

extern tVoid       LLD_RTC_Enable                ( tUInt);
extern tBool       LLD_RTC_IsEnabled             ( tVoid);
extern tU32        LLD_RTC_GetDataRegister       ( tVoid);
extern tVoid       LLD_RTC_SetMatchRegister      ( const tU32 value);
extern tVoid       LLD_RTC_SetLoadRegister       ( const tU32 value);
extern tBool       LLD_RTC_LoadRegisterLocked    ( tVoid);
extern tVoid       LLD_RTC_GetCkDiv              ( tU32 * value);
extern tVoid       LLD_RTC_EnableInterrupt       ( tVoid);
extern tVoid       LLD_RTC_DisableInterrupt      ( tVoid);
extern tU32        LLD_RTC_GetInterruptStatus    ( tVoid);
extern tVoid       LLD_RTC_ClearInterrupt        ( tVoid);

#if defined( LLD_RTC_VER_2_0)
extern tU16        LLD_RTC_GetDataFracRegister   ( tVoid);
extern tVoid       LLD_RTC_Set_DRI_DRF_Registers ( const tU32 dri, const tU32 drf);
extern tVoid       LLD_RTC_Get_DRI_DRF_Registers ( tU32 *dri, tU32 *drf);
#endif
#endif  // _LLD_RTC_
