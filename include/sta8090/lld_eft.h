//!
//!   \file     lld_eft.h
//!   \brief    <i><b>EFT low level driver header file</b></i>
//!   \author   Fulvio Boggia
//!   \author   (original version) Massimo De Martino
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_EFT_H
#define LLD_EFT_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief Interrupt sources
typedef enum LLD_EFT_ISR_src_e {
  LLD_EFT_ISR_SRC_OVERFLOW,         /**< Overflow interrupt */
  LLD_EFT_ISR_SRC_OUTCMP_A,         /**< Output compare A interrupt */
  LLD_EFT_ISR_SRC_OUTCMP_B,         /**< Output compare B interrupt */
  #if !defined( LLD_EFT_STA2062_SLV )
  LLD_EFT_ISR_SRC_INPUTCAPT_A,      /**< Input capture A interrupt */
  LLD_EFT_ISR_SRC_INPUTCAPT_B,      /**< Input capture B interrupt */
  #endif
  LLD_EFT_ISR_SRC_NUMBER
} LLD_EFT_ISR_src_t;

//! \brief Output compares available
typedef enum LLD_EFT_outcmp_e {
  LLD_EFT_OUTCMP_A,                 /**< Output compare A */
  LLD_EFT_OUTCMP_B                  /**< Output compare B */
} LLD_EFT_outcmp_t;

//! \brief Output compares value
typedef enum LLD_EFT_outcmp_val_e {
  LLD_EFT_OUTCMP_VAL_LOW  = 0x0,
  LLD_EFT_OUTCMP_VAL_HIGH = 0x1,
  LLD_EFT_OUTCMP_VAL_OFF  = 0x2
} LLD_EFT_outcmp_val_t;

#if defined( LLD_I2C_ICSUPPORT )
//! \brief Input captures available
typedef enum LLD_EFT_inputcapt_e {
  LLD_EFT_INPUTCAPT_A,              /**< Input capture A */
  LLD_EFT_INPUTCAPT_B               /**< Input capture B */
} LLD_EFT_inputcapt_t;

typedef enum LLD_EFT_ICEdge_e {
  LLD_EFT_ICEDGE_FALLING  = 0x0,
  LLD_EFT_ICEDGE_RISING   = 0x1
} LLD_EFT_ICEdge_t;
#endif

//! \brief Type of EFT interrupt handler
typedef tVoid (*LLD_EFT_ISR_t)(tVoid);

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid              LLD_EFT_Init                    ( tVoid);
extern tVoid              LLD_EFT_ResetReg                ( tVoid);

extern tVoid              LLD_EFT_InstallISR              ( const LLD_EFT_ISR_src_t, LLD_EFT_ISR_t);
extern tVoid              LLD_EFT_ISR_handler             ( tVoid);

extern tVoid              LLD_EFT_TimerConfig             ( tU8);
extern tVoid              LLD_EFT_TimerStart              ( tVoid);
extern tVoid              LLD_EFT_TimerStop               ( tVoid);
extern tU32               LLD_EFT_TimerRead               ( tVoid);

extern tVoid              LLD_EFT_EnableIRQ               ( const LLD_EFT_ISR_src_t);
extern tVoid              LLD_EFT_DisableIRQ              ( const LLD_EFT_ISR_src_t);
extern tVoid              LLD_EFT_ClearIRQ                ( const LLD_EFT_ISR_src_t);
extern tBool              LLD_EFT_GetIRQStatus            ( const LLD_EFT_ISR_src_t);

extern tVoid              LLD_EFT_OutCmpConfig            ( const LLD_EFT_outcmp_t, const LLD_EFT_outcmp_val_t);
extern tVoid              LLD_EFT_OutCmpSet               ( const LLD_EFT_outcmp_t, const tU32);
extern tU32               LLD_EFT_OutCmpGet               ( const LLD_EFT_outcmp_t);
extern tVoid              LLD_EFT_OutCmpForce             ( const LLD_EFT_outcmp_t oc);

#if defined( LLD_I2C_ICSUPPORT )
extern tVoid              LLD_EFT_InputCaptSet            ( const LLD_EFT_inputcapt_t, const LLD_EFT_ICEdge_t);
extern LLD_EFT_ICEdge_t   LLD_EFT_InputCaptGet            ( const LLD_EFT_inputcapt_t);
extern tU32               LLD_EFT_InputCaptRead           ( const LLD_EFT_inputcapt_t);

extern tVoid              LLD_EFT_PWMmodeEn               ( tU32, tU32);
#endif

#endif // _LLD_EFT_H_

/* End of file */
