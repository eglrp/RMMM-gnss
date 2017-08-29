//!
//!   \file       lld_wdg.h
//!   \brief      <i><b>WDG low level driver header file</b></i>
//!   \author     Maristella Frazzetto
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup LLD
//!   \{
//!   \addtogroup WDG
//!   \{
//!

#ifndef LLD_WDG_H
#define LLD_WDG_H

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

typedef tU32 LLD_WDG_IdTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid  LLD_WDG_Init                    ( tVoid);
extern tVoid  LLD_WDG_StartWatchdog           ( tVoid);

extern tVoid  LLD_WDG_EnableWatchdogMode      ( tVoid);

extern tVoid  LLD_WDG_SetPrescalerVal         ( tU16);
extern tVoid  LLD_WDG_SetPreLoadVal           ( tU16);
extern tU16   LLD_WDG_GetCounterReg           ( tVoid);

extern tVoid  LLD_WDG_EnableEndOfCountIRQ     ( tVoid);
extern tVoid  LLD_WDG_DisableEndOfCountIRQ    ( tVoid);
extern tVoid  LLD_WDG_ClearEndOfCountIRQ      ( tVoid);
extern tBool  LLD_WDG_GetEndOfCountIRQStatus  ( tVoid);

extern tVoid  LLD_WDG_SetKeyReg               ( tU16);

#endif // LLD_WDG_H

//!   \}

//!   \}

