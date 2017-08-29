//!
//!   \file       lld_adc_sta8090.h
//!   \brief      <i><b>ADC low level driver header file</b></i>
//!   \author     Maristella Frazzetto
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup LLD
//!   \{
//!   \addtogroup ADC
//!   \{
//!

#ifndef LLD_ADC_STA8090_H
#define LLD_ADC_STA8090_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief Type for ADC peripheral
typedef void * LLD_ADC_IdTy;

typedef enum
{
  LLD_ADC_NORMAL_MODE_ENABLED = 0x0,
  LLD_ADC_SW_MODE_ENABLED = 0x1
} LLD_ADC_ModeTy;

typedef enum
{
  LLD_ADC_IRQ_SRC_CH0        = 0x10,
  LLD_ADC_IRQ_SRC_CH1        = 0x80,
  LLD_ADC_IRQ_SRC_ALL        = 0x90
} LLD_ADC_CHIRQSrcIdTy;

typedef enum
{
  LLD_ADC_IRQ_SRC_Hthch0      = 0x20,
  LLD_ADC_IRQ_SRC_Lthch0      = 0x40,
  LLD_ADC_IRQ_SRC_Hthch1      = 0x100,
  LLD_ADC_IRQ_SRC_Lthch1      = 0x200
} LLD_ADC_IRQSrcIdTy;

typedef tU32 LLD_ADC_IRQSrcTy;

typedef enum
{
  LLD_ADC_CH0IntDISABLED,
  LLD_ADC_CH0IntENABLED
} LLD_ADC_CH0IntEnableTy;

typedef enum
{
  LLD_ADC_CH1IntDISABLED,
  LLD_ADC_CH1IntENABLED
} LLD_ADC_CH1IntEnableTy;

typedef enum
{
  LLD_ADC_CH0 = 0x0,
  LLD_ADC_CH1 = 0x1,
  LLD_ADC_CH2 = 0x2,
  LLD_ADC_CH3 = 0x3,
  LLD_ADC_CH4 = 0x4,
  LLD_ADC_CH5 = 0x5,
  LLD_ADC_CH6 = 0x6,
  LLD_ADC_CH7 = 0x7
} LLD_ADC_ChanIdTy;

typedef enum
{
  LLD_ADC_ChanswitchSel_64  = 0x0,
  LLD_ADC_ChanswitchSel_128 = 0x1,
  LLD_ADC_ChanswitchSel_256 = 0x2,
  LLD_ADC_NoChanswitch   = 0x3
} LLD_ADC_ChanSelTy;

typedef enum
{
  LLD_ADC_chanswitchmask_64  = 0x40,
  LLD_ADC_chanswitchmask_128 = 0x80,
  LLD_ADC_chanswitchmask_256 = 0x100
} LLD_ADC_ChanswitchTy;

typedef enum
{
  LLD_ADC_chan1_cntmask_64  = 0x7F,
  LLD_ADC_chan1_cntmask_128 = 0xFF,
  LLD_ADC_chan1_cntmask_256 = 0x1FF
} LLD_ADC_Chan1_cntmask_Ty;

typedef struct
{
  LLD_ADC_ModeTy mode;
  LLD_ADC_ChanIdTy chid;
  LLD_ADC_CHIRQSrcIdTy chirqsrc;
  LLD_ADC_ChanSelTy sel;
} LLD_ADC_ConfigTy;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
extern tVoid             LLD_ADC_Enable              ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_Disable             ( const LLD_ADC_IdTy);
extern tBool             LLD_ADC_IsADCEnabled        ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetMode             ( const LLD_ADC_IdTy, LLD_ADC_ModeTy);
extern tU32              LLD_ADC_GetMode             ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_Enable_SW_Mode      ( const LLD_ADC_IdTy);
extern tBool             LLD_ADC_IsADCNormalMode     ( const LLD_ADC_IdTy);
extern tBool             LLD_ADC_IsADCSWMode         ( const LLD_ADC_IdTy);
extern LLD_ErrorTy       LLD_ADC_EnableIRQSrc        ( const LLD_ADC_IdTy, LLD_ADC_CHIRQSrcIdTy);
extern tVoid             LLD_ADC_DisableIRQSrc       ( const LLD_ADC_IdTy, LLD_ADC_CHIRQSrcIdTy);
extern tBool             LLD_ADC_GetIRQSrc           ( const LLD_ADC_IdTy, LLD_ADC_IRQSrcIdTy);
extern tVoid             LLD_ADC_ClearIRQSrc         ( const LLD_ADC_IdTy, LLD_ADC_IRQSrcTy);
extern tBool             LLD_ADC_IsPendingIRQSrc     ( const LLD_ADC_IdTy, LLD_ADC_IRQSrcIdTy);
extern LLD_ErrorTy       LLD_ADC_Set_Chan_id         ( const LLD_ADC_IdTy, LLD_ADC_ChanIdTy);
extern LLD_ErrorTy       LLD_ADC_SetChanSel          ( const LLD_ADC_IdTy, LLD_ADC_ChanSelTy);
extern LLD_ADC_ChanSelTy LLD_ADC_GetChanSel          ( const LLD_ADC_IdTy);
extern tU32              LLD_ADC_GetCntReg           ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetCntReg           ( const LLD_ADC_IdTy, tU32);
extern tVoid             LLD_ADC_switchChan0         ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_switchChan1         ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetClkReg           ( const LLD_ADC_IdTy, tU32);
extern tU32              LLD_ADC_GetClkReg           ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_ClkDivEnable        ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_ClkDivDisable       ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetHth0Reg          ( const LLD_ADC_IdTy, tU32);
extern tU32              LLD_ADC_GetHth0Reg          ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetLth0Reg          ( const LLD_ADC_IdTy, tU32);
extern tU32              LLD_ADC_GetLth0Reg          ( const LLD_ADC_IdTy);
extern tU32              LLD_ADC_GetAvg0SampleReg    ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetHth1Reg          ( const LLD_ADC_IdTy, tU32);
extern tU32              LLD_ADC_GetHth1Reg          ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetLth1Reg          ( const LLD_ADC_IdTy, tU32);
extern tU32              LLD_ADC_GetLth1Reg          ( const LLD_ADC_IdTy);
extern tU32              LLD_ADC_GetAvg1SampleReg    ( const LLD_ADC_IdTy);
extern tU32              LLD_ADC_GetADCD0Reg         ( const LLD_ADC_IdTy);
extern tU32              LLD_ADC_GetADCD1Reg         ( const LLD_ADC_IdTy);
extern tU32              LLD_ADC_GetADCD2Reg         ( const LLD_ADC_IdTy);
extern tU32              LLD_ADC_GetADCD3Reg         ( const LLD_ADC_IdTy);
extern tU32              LLD_ADC_GetAvgSampleReg     ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetConfiguration    ( const LLD_ADC_IdTy, LLD_ADC_ConfigTy *);
extern tVoid             LLD_ADC_ResetReg            ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_SetCFGReg           ( const LLD_ADC_IdTy, tU32);
extern tVoid             LLD_ADC_EnablePullUpAIN0    ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_EnablePullUpAIN1    ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_DisablePullUpAIN0   ( const LLD_ADC_IdTy);
extern tVoid             LLD_ADC_DisablePullUpAIN1   ( const LLD_ADC_IdTy);
#endif // LLD_ADC_STA8090_H

//!   \}

//!   \}

