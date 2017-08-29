//!
//!   \file     lld_rtt.h
//!   \brief    <i><b>RTT low level driver header file</b></i>
//!   \author   Fulvio Boggia
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_RTT
#define LLD_RTT

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

typedef enum
{
  LLD_RTT_MODE_PERIODIC,
  LLD_RTT_MODE_ONESHOT
} LLD_RTT_ModeTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid  LLD_RTT_Enable                ( tVoid);
extern tVoid  LLD_RTT_Disable               ( tVoid);
extern tBool  LLD_RTT_IsEnabled             ( tVoid);
extern tVoid  LLD_RTT_SetMode               ( LLD_RTT_ModeTy mode);
extern tU32   LLD_RTT_GetDataRegister       ( tVoid);
extern tVoid  LLD_RTT_SetLoadRegister       ( const tU32 value);
extern tVoid  LLD_RTT_EnableInterrupt       ( tVoid);
extern tVoid  LLD_RTT_DisableInterrupt      ( tVoid);
extern tU32   LLD_RTT_GetInterruptStatus    ( tVoid);
extern tVoid  LLD_RTT_ClearInterrupt        ( tVoid);
#endif  // LLD_RTT
