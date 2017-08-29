//!
//!   \file     lld_mtu.h
//!   \brief    <i><b>MTU low level driver header file</b></i>
//!   \author   Fulvio Boggia
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_MTU_H
#define LLD_MTU_H

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

/*! @brief It's the type definition for MTU timer IP (it's the MTU base address) */
typedef void * LLD_MTU_IPTy;

/*! @brief It's the type definition for MTU timer IDs (there are 3 available timers for each MTU IP) */
typedef enum
{
  LLD_MTU_TIMER0,
  LLD_MTU_TIMER1,
  LLD_MTU_TIMER2,
  LLD_MTU_TIMER3,
} LLD_MTU_IdTy;

/*! @brief It's the type definition for MTU prescaler values */
typedef enum
{
  LLD_MTU_PRESCALER_DIV_1,        /*!< The prescaler is equal to 1 */
  LLD_MTU_PRESCALER_DIV_16,       /*!< The prescaler is equal to 16 */
  LLD_MTU_PRESCALER_DIV_256,      /*!< The prescaler is equal to 256 */
} LLD_MTU_PrescalerTy;

/*! @brief It's the type definition for MTU counter size */
typedef enum
{
  LLD_MTU_16_BITS,                /*!< The counter size is 16 bits (0..0xFFFF) */
  LLD_MTU_32_BITS                 /*!< The counter size is 32 bits (0..0xFFFFFFFF) */
} LLD_MTU_SizeTy;

/*! @brief It's the type definition for MTU operating mode */
typedef enum
{
  LLD_MTU_FREE_RUNNING,           /*!< The timer is set in Free-running mode */
  LLD_MTU_ONE_SHOT,               /*!< The timer is set in One-shot mode */
  LLD_MTU_PERIODIC,               /*!< The timer is set in Periodic mode */
} LLD_MTU_ModeTy;

/*! @brief It's the type definition for MTU interrupts masks */
typedef enum
{
  LLD_MTU_TIMER0_MASK = BIT_0,    /*!< The bit mask that select timer 0 */
  LLD_MTU_TIMER1_MASK = BIT_1,    /*!< The bit mask that select timer 1 */
  LLD_MTU_TIMER2_MASK = BIT_2,    /*!< The bit mask that select timer 2 */
  LLD_MTU_TIMER3_MASK = BIT_3     /*!< The bit mask that select timer 3 */
} LLD_MTU_InterruptMaskIdTy;

typedef tU32 LLD_MTU_InterruptMaskTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid  LLD_MTU_Enable                      ( const LLD_MTU_IPTy ip, LLD_MTU_IdTy id);
extern tVoid  LLD_MTU_Disable                     ( const LLD_MTU_IPTy ip, LLD_MTU_IdTy id);
extern tBool  LLD_MTU_IsEnabled                   ( const LLD_MTU_IPTy ip, const LLD_MTU_IdTy id);
extern tVoid  LLD_MTU_Reset                       ( const LLD_MTU_IPTy ip);
extern tVoid  LLD_MTU_Config                      ( const LLD_MTU_IPTy ip, LLD_MTU_IdTy id, LLD_MTU_SizeTy counter_size, LLD_MTU_PrescalerTy prescaler, LLD_MTU_ModeTy mode);
extern tVoid  LLD_MTU_SetLoadRegister             ( const LLD_MTU_IPTy ip, LLD_MTU_IdTy id, tU32 value);
extern tVoid  LLD_MTU_SetMode                     ( const LLD_MTU_IPTy ip, const LLD_MTU_IdTy id, const LLD_MTU_ModeTy mode);
extern tVoid  LLD_MTU_SetBackgroundLoadRegister   ( const LLD_MTU_IPTy ip, LLD_MTU_IdTy id, tU32 value);
extern tU32   LLD_MTU_GetValueRegister            ( const LLD_MTU_IPTy ip, LLD_MTU_IdTy id);
extern tVoid  LLD_MTU_SetInterruptMask            ( const LLD_MTU_IPTy ip, LLD_MTU_InterruptMaskTy mask);
extern tVoid  LLD_MTU_ClearInterruptMask          ( const LLD_MTU_IPTy ip, LLD_MTU_InterruptMaskTy mask);
extern tVoid  LLD_MTU_ClearInterrupt              ( const LLD_MTU_IPTy ip, LLD_MTU_InterruptMaskTy mask);
extern tU8    LLD_MTU_GetRawInterruptStatus       ( const LLD_MTU_IPTy ip);
extern tU8    LLD_MTU_GetMaskedInterruptStatus    ( const LLD_MTU_IPTy ip);

#endif  // _LLD_MTU_H_

// End of file
