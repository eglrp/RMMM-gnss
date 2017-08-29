//!
//!   \file       lld_prcc_sta8090.h
//!   \brief      <i><b>Power, reset and clock controller driver header file</b></i>
//!   \author     Fulvio Boggia
//!   \version    1.0
//!   \date       2013.05.24
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup LLD
//!   \{
//!   \addtogroup PRCC
//!   \{
//!

#ifndef LLD_PRCC_STA8090_H
#define LLD_PRCC_STA8090_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define LLD_PRCC_ROMT3VERSION_ADDR        0x30003FF8
#define LLD_PRCC_RINGOSCWRONGFREQ         0xFF

#define LLD_PRCC_RFEXTCFG_GALGPS          BIT_1
#define LLD_PRCC_RFEXTCFG_GNSCOM          BIT_2
#define LLD_PRCC_RFEXTCFG_64F0CLK         BIT_0

#define LLD_PRCC_RFEXTCFG_ALLINT          0x0
#define LLD_PRCC_RFEXTCFG_ALLEXT          (LLD_PRCC_RFEXTCFG_GALGPS | LLD_PRCC_RFEXTCFG_GNSCOM | LLD_PRCC_RFEXTCFG_64F0CLK)

#define LLD_PRCC_MTU_0                    BIT_0       /**< MTU1 unit 0 */
#define LLD_PRCC_MTU_1                    BIT_1       /**< MTU1 unit 1 */
#define LLD_PRCC_MTU_2                    BIT_2       /**< MTU1 unit 2 */
#define LLD_PRCC_MTU_3                    BIT_3       /**< MTU1 unit 3 */
#define LLD_PRCC_MTU_4                    BIT_4       /**< MTU2 unit 0 */
#define LLD_PRCC_MTU_5                    BIT_5       /**< MTU2 unit 1 */
#define LLD_PRCC_MTU_6                    BIT_6       /**< MTU2 unit 2 */
#define LLD_PRCC_MTU_7                    BIT_7       /**< MTU2 unit 3 */

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief TCM configuration values for ARM9
typedef enum {
  LLD_PRCC_TCMCFG_I016_D240 = 0x0U,        /**< ITCM  16k, DTCM 240k */
  LLD_PRCC_TCMCFG_I032_D224 = 0x1U,        /**< ITCM  32k, DTCM 224k */
  LLD_PRCC_TCMCFG_I048_D208 = 0x2U,        /**< ITCM  48k, DTCM 208k */
  LLD_PRCC_TCMCFG_I064_D192 = 0x3U,        /**< ITCM  64k, DTCM 192k */
  LLD_PRCC_TCMCFG_I080_D176 = 0x4U,        /**< ITCM  80k, DTCM 176k */
  LLD_PRCC_TCMCFG_I096_D160 = 0x5U,        /**< ITCM  96k, DTCM 160k */
  LLD_PRCC_TCMCFG_I112_D144 = 0x6U,        /**< ITCM 112k, DTCM 144k */
  LLD_PRCC_TCMCFG_I128_D128 = 0x7U         /**< ITCM 128k, DTCM 128k */
} LLD_PRCC_TCMCfgTy;

//! \brief Configuration for accessing G3 acquisition memory
typedef enum {
  LLD_PRCC_MEMMST_G3        = 0x0U,        /**< G3 masters acquisition memory */
  LLD_PRCC_MEMMST_ARM       = 0x1U         /**< ARM masters acquisition memory */
} LLD_PRCC_MemMstTy;

//! \brief Type to identify MTU unit
typedef tU32 LLD_PRCC_MTUTy;

//! \brief Type to select AF for UART1 or USB
typedef enum {
  LLD_PRCC_UART1USBPINS_UART1   = 0x0U,    /**< Pins are driven by UART1 */
  LLD_PRCC_UART1USBPINS_USB     = 0x1U     /**< Pins are driven by USB */
} LLD_PRCC_UART1USBPinsTy;

typedef enum {
  LLD_PRCC_CLKSEL_192   = 0x0U,
  LLD_PRCC_CLKSEL_96    = 0x1U,
  LLD_PRCC_CLKSEL_64    = 0x2U,
  LLD_PRCC_CLKSEL_48    = 0x3U
} LLD_PRCC_ClkSelTy;

typedef enum {
  LLD_PRCC_ARMCLKSRC_192f0,
  LLD_PRCC_ARMCLKSRC_TCXO,
  LLD_PRCC_ARMCLKSRC_RTC,
  LLD_PRCC_ARMCLKSRC_RING
} LLD_PRCC_ARMClkSrcTy;

typedef enum {
  LLD_PRCC_HCLKDIV_DIV1       = 0x0U,
  LLD_PRCC_HCLKDIV_DIV2       = 0x1U,
  LLD_PRCC_HCLKDIV_DIV4       = 0x2U
} LLD_PRCC_HCLKDivTy;

typedef enum {
  LLD_PRCC_SQICLKDIV_DIV1     = 0x0U,
  LLD_PRCC_SQICLKDIV_DIV2     = 0x1U
} LLD_PRCC_SQIClkDivTy;

typedef enum {
  LLD_PRCC_FSMCCLKDIV_DIV1    = 0x0U,
  LLD_PRCC_FSMCCLKDIV_DIV2    = 0x1U
} LLD_PRCC_FSMCClkDivTy;

typedef enum {
  LLD_PRCC_PERIPHID_ARM           = 0U,
  LLD_PRCC_PERIPHID_VIC           = 1U,
  LLD_PRCC_PERIPHID_SQIO          = 2U,
  LLD_PRCC_PERIPHID_APB1          = 3U,
  LLD_PRCC_PERIPHID_GPIO0         = 4U,
  LLD_PRCC_PERIPHID_GPIO1         = 5U,
  LLD_PRCC_PERIPHID_APB2          = 6U,
  LLD_PRCC_PERIPHID_AHB           = 7U,
  LLD_PRCC_PERIPHID_RTC           = 8U,
  LLD_PRCC_PERIPHID_SRAM          = 9U,
  LLD_PRCC_PERIPHID_G3EP          = 10U,
  LLD_PRCC_PERIPHID_G3EP_AHB      = 11U,
  LLD_PRCC_PERIPHID_USB           = 12U,
  LLD_PRCC_PERIPHID_WDOG          = 13U,
  LLD_PRCC_PERIPHID_EFT           = 14U,
  LLD_PRCC_PERIPHID_SDMC          = 15U,
  LLD_PRCC_PERIPHID_SSP           = 16U,
  LLD_PRCC_PERIPHID_MTU0          = 17U,
  LLD_PRCC_PERIPHID_MSP           = 18U,
  LLD_PRCC_PERIPHID_I2C           = 19U,
  LLD_PRCC_PERIPHID_UART0         = 20U,
  LLD_PRCC_PERIPHID_UART1         = 21U,
  LLD_PRCC_PERIPHID_UART2         = 22U,
  LLD_PRCC_PERIPHID_CAN0          = 23U,
  LLD_PRCC_PERIPHID_CAN1          = 24U,
  LLD_PRCC_PERIPHID_ADC           = 25U,
  LLD_PRCC_PERIPHID_THS           = 26U,
  LLD_PRCC_PERIPHID_FSMC          = 27U,
  LLD_PRCC_PERIPHID_RIOSC_REF     = 28U,
  LLD_PRCC_PERIPHID_REC_PLAY      = 29U,
  LLD_PRCC_PERIPHID_TCXOCLK2G3BB  = 30U,
  LLD_PRCC_PERIPHID_CLK_CTRL_FSM  = 31U,
  LLD_PRCC_PERIPHID_MTU1          = 32U
} LLD_PRCC_PeriphIDTy;

typedef tU64 LLD_PRCC_PeriphMaskTy;

typedef tU32 LLD_PRCC_RFExtCfgTy;

typedef enum {
  LLD_PRCC_ARMBOOT_GNSS_ROM     = 0x0U,
  LLD_PRCC_ARMBOOT_FSMC         = 0x1U,
  LLD_PRCC_ARMBOOT_UART         = 0x2U,
  LLD_PRCC_ARMBOOT_SQI          = 0x3U
} LLD_PRCC_ARMBootTy;

typedef enum {
  LLD_PRCC_T3CUT_AA             = 0x0U,
  LLD_PRCC_T3CUT_AB             = 0x1U,
  LLD_PRCC_T3CUT_BA             = 0x2U,
  LLD_PRCC_T3CUT_BB             = 0x3U,
  LLD_PRCC_T3CUT_BC             = 0x4U,
  LLD_PRCC_T3CUT_BD             = 0x5U
} LLD_PRCC_T3CutTy;

typedef enum {
  LLD_PRCC_LDO1       = BIT_0,  // Low Voltage Regulator 1
  LLD_PRCC_LDO2       = BIT_1,  // Low Voltage Regulator 2
  LLD_PRCC_BKLDO      = BIT_2,  // Backup Low Voltage Regulator
  LLD_PRCC_BKLDO_CL   = BIT_3,  // Backup Low Voltage Regulator Current Limiter
  LLD_PRCC_HM1        = BIT_4,  // HardMacro1. It embeds SMPS regulator and its circuitry
  LLD_PRCC_OLVDL1     = BIT_5,  // Output Low Voltage Detector for LDO1
  LLD_PRCC_ILVDM      = BIT_6,  // Input Low Voltage Detector for SMPS (HardMacro1)
  LLD_PRCC_OLVDM      = BIT_7,  // Output Low Voltage Detector for SMPS (HardMacro1)
  LLD_PRCC_ILVDB      = BIT_8,  // Input Low Voltage Detector for backup LDO
  LLD_PRCC_OLVDB      = BIT_9   // Output Low Voltage Detector for backup LDO
} LLD_PRCC_LDOTy;

typedef enum {
  LLD_PRCC_WDG_RESET_NOT_FIRED  = 0x0U,
  LLD_PRCC_WDG_RESET_FIRED      = 0x1U
} LLD_PRCC_WDGResetTy;

typedef enum {
  LLD_PRCC_STA8089              = 0x0U,
  LLD_PRCC_STA8090              = 0x1U
} LLD_PRCC_Pin2PinStatusTy;

typedef enum {
  LLD_PRCC_THSENSMODE_FSM_OFF   = 0x0U,
  LLD_PRCC_THSENSMODE_FSM_ON    = 0x1U
} LLD_PRCC_THSENSFSMModeTy;

typedef enum {
  LLD_PRCC_THSENSPadDirInput    = 0x0U,
  LLD_PRCC_THSENSPadDirOutput   = 0x1U
} LLD_PRCC_THSENSPADDirTy;

typedef enum {
  LLD_PRCC_CLKTOARM_RING        = 0x0U,
  LLD_PRCC_CLKTOARM_OSCI        = 0x1U,
  LLD_PRCC_CLKTOARM_TCXO        = 0x2U
} LLD_PRCC_ClkToArmTy;

typedef enum {
  LLD_PRCC_ROMPATCHMODE_PURE_ROM       = 0x0U,
  LLD_PRCC_ROMPATCHMODE_PATCH          = 0x1U,
  LLD_PRCC_ROMPATCHMODE_FULL_AVAIL     = 0x2U
} LLD_PRCC_ROMPatchMode;

typedef tU32 LLD_PRCC_ClkToArmMaskTy;

typedef enum {
  LLD_PRCC_COREVOLTAGE_1_0V    = 1U,
  LLD_PRCC_COREVOLTAGE_1_1V    = 2U,
  LLD_PRCC_COREVOLTAGE_1_2V    = 3U
} LLD_PRCC_CoreVoltageTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid                      LLD_PRCC_SetTCMSize             ( LLD_PRCC_TCMCfgTy);
extern LLD_PRCC_TCMCfgTy          LLD_PRCC_GetTCMSize             ( tVoid);

extern tVoid                      LLD_PRCC_EnableMTU              ( LLD_PRCC_MTUTy);
extern tVoid                      LLD_PRCC_DisableMTU             ( LLD_PRCC_MTUTy);
extern tVoid                      LLD_PRCC_EnableG3ctrldMTU       ( LLD_PRCC_MTUTy);
extern tVoid                      LLD_PRCC_DisableG3ctrldMTU      ( LLD_PRCC_MTUTy);
extern tVoid                      LLD_PRCC_SetUART1USBPins        ( LLD_PRCC_UART1USBPinsTy);
extern LLD_PRCC_UART1USBPinsTy    LLD_PRCC_GetUART1USBPins        ( tVoid);

extern tVoid                      LLD_PRCC_SetAllG3Mem            ( LLD_PRCC_MemMstTy);
extern LLD_PRCC_MemMstTy          LLD_PRCC_GetAllG3Mem            ( tVoid);
extern tVoid                      LLD_PRCC_SetSingleG3Mem         ( tU32, LLD_PRCC_MemMstTy);
extern LLD_PRCC_MemMstTy          LLD_PRCC_GetSingleG3Mem         ( tU32);

extern tVoid                      LLD_PRCC_SetUSBCfg              ( boolean_t);
extern boolean_t                  LLD_PRCC_GetUSBCfg              ( tVoid);

extern tVoid                      LLD_PRCC_EnablePeripheral       ( LLD_PRCC_PeriphIDTy);
extern tVoid                      LLD_PRCC_DisablePeripheral      ( LLD_PRCC_PeriphIDTy);
extern tVoid                      LLD_PRCC_PeripheralClkEn        ( LLD_PRCC_PeriphIDTy);
extern tVoid                      LLD_PRCC_PeripheralClkDis       ( LLD_PRCC_PeriphIDTy);
extern tVoid                      LLD_PRCC_PeripheralRstEn        ( LLD_PRCC_PeriphIDTy);
extern tVoid                      LLD_PRCC_PeripheralRstDis       ( LLD_PRCC_PeriphIDTy);

extern tVoid                      LLD_PRCC_EnablePerMask          ( LLD_PRCC_PeriphMaskTy);
extern tVoid                      LLD_PRCC_DisablePerMask         ( LLD_PRCC_PeriphMaskTy);
extern tVoid                      LLD_PRCC_ClkEnPerMask           ( LLD_PRCC_PeriphMaskTy);
extern tVoid                      LLD_PRCC_ClkDisPerMask          ( LLD_PRCC_PeriphMaskTy);

extern tVoid                      LLD_PRCC_SetOscillator          ( boolean_t);
extern tVoid                      LLD_PRCC_SetRTCStatus           ( boolean_t);
extern tVoid                      LLD_PRCC_EnableWakeup           ( tVoid);
extern tVoid                      LLD_PRCC_DisableWakeup          ( tVoid);
extern tVoid                      LLD_PRCC_EnableWakeupFromRTC    ( tVoid);
extern tVoid                      LLD_PRCC_DisableWakeupFromRTC   ( tVoid);

extern tVoid                      LLD_PRCC_SetRingOscillator      ( boolean_t);
extern tU32                       LLD_PRCC_GetRingOscFreq         ( tVoid);
extern tVoid                      LLD_PRCC_GenerateSoftReset      ( tVoid);
extern tVoid                      LLD_PRCC_SetExternalRF          ( const LLD_PRCC_RFExtCfgTy);
extern tVoid                      LLD_PRCC_RFEnable               ( tVoid);
extern tVoid                      LLD_PRCC_RFDisable              ( tVoid);
extern tVoid                      LLD_PRCC_RFSPIIfEnable          ( tVoid);
extern tVoid                      LLD_PRCC_RFSPIIfDisable         ( tVoid);
extern tVoid                      LLD_PRCC_RFFullyOperative       ( tVoid);
extern tVoid                      LLD_PRCC_RFPowerMode            ( tVoid);
extern tVoid                      LLD_PRCC_RFChipEnable           ( tVoid);
extern tVoid                      LLD_PRCC_RFChipDisable          ( tVoid);

extern tVoid                      LLD_PRCC_RFRegRstEn             ( tVoid);
extern tVoid                      LLD_PRCC_RFRegRstDis            ( tVoid);

extern LLD_ErrorTy                LLD_PRCC_SetARMClkSrcStatus     ( const LLD_PRCC_ARMClkSrcTy, const tBool);
extern LLD_ErrorTy                LLD_PRCC_SelectARMClkSrc        ( const LLD_PRCC_ARMClkSrcTy, const tBool);
extern LLD_PRCC_ARMClkSrcTy       LLD_PRCC_GetARMClkSrc           ( boolean_t *clk_sel);
extern LLD_ErrorTy                LLD_PRCC_SetARMClkDiv           ( const LLD_PRCC_ARMClkSrcTy, const tU32);
extern tVoid                      LLD_PRCC_SetSQICLkDiv           ( const LLD_PRCC_SQIClkDivTy);
extern tVoid                      LLD_PRCC_SetHCLKClkDiv          ( const LLD_PRCC_HCLKDivTy);

extern tVoid                      LLD_PRCC_EnableReset            ( tVoid);
extern tVoid                      LLD_PRCC_DisableReset           ( tVoid);

extern tU32                       LLD_PRCC_GetBOOTPins            ( tVoid);
extern LLD_PRCC_ARMBootTy         LLD_PRCC_GetHWBootStatus        ( tVoid);
extern LLD_PRCC_T3CutTy           LLD_PRCC_GetT3Cut               ( tVoid);

extern tU8                        LLD_PRCC_GetAntSens0Pin         ( tVoid);
extern tU8                        LLD_PRCC_GetAntSens1Pin         ( tVoid);
extern boolean_t                  LLD_PRCC_GetExtSQIStatus        ( tVoid);

extern tVoid                      LLD_PRCC_WDGResetEnable         ( tVoid);
extern tVoid                      LLD_PRCC_WDGResetDisable        ( tVoid);
extern LLD_PRCC_WDGResetTy        LLD_PRCC_WDGResetGetFlag        ( tVoid);
extern tVoid                      LLD_PRCC_WDGResetClearFlag      ( tVoid);

extern tVoid                      LLD_PRCC_G3EPClkEnable          ( tVoid);
extern tVoid                      LLD_PRCC_G3EPClkDisable         ( tVoid);
extern tVoid                      LLD_PRCC_G3TBClkEnable          ( tVoid);
extern tVoid                      LLD_PRCC_G3TBClkDisable         ( tVoid);
extern tU32                       LLD_PRCC_TCXOTBClkRead          ( tVoid);

extern tVoid                      LLD_PRCC_LDOEnable              ( LLD_PRCC_LDOTy);
extern tVoid                      LLD_PRCC_LDODisable             ( LLD_PRCC_LDOTy);
extern tVoid                      LLD_PRCC_StopModeEnable         ( tVoid);
extern tVoid                      LLD_PRCC_StopModeDisable        ( tVoid);
extern tVoid                      LLD_PRCC_EnterStandby           ( tVoid);
extern tVoid                      LLD_PRCC_Clk2ArmEn              ( LLD_PRCC_ClkToArmMaskTy);
extern tVoid                      LLD_PRCC_Clk2ArmDis             ( LLD_PRCC_ClkToArmMaskTy);

extern tVoid                      LLD_PRCC_SetWakeupFromRTC       ( boolean_t);
extern tVoid                      LLD_PRCC_SetWakeupFromSTDBYpin  ( boolean_t);
extern tVoid                      LLD_PRCC_IsSTDBYWakeUpSource    ( boolean_t*,boolean_t*);
extern tVoid                      LLD_PRCC_StandByEnable          ( tVoid);
extern tVoid                      LLD_PRCC_SetClkCtrlClkSel       ( tUChar );
extern tVoid                      LLD_PRCC_BigMosEnable           ( tVoid);
extern tVoid                      LLD_PRCC_BigMosDisable          ( tVoid);
extern tVoid                      LLD_PRCC_SetCoreVoltage         ( LLD_PRCC_CoreVoltageTy);
extern tVoid                      LLD_PRCC_EnableSMPS             ( LLD_PRCC_CoreVoltageTy, boolean_t);
extern tVoid                      LLD_PRCC_DisableSMPS            ( tVoid);
extern tVoid                      LLD_PRCC_RemapBackupEnable      ( tVoid);
extern tVoid                      LLD_PRCC_RemapBackupDisable     ( tVoid);
extern tBool                      LLD_PRCC_LDO1REGVoltageIsOK     ( tVoid);

extern tU32                       LLD_PRCC_RunRIOSCCalibration    ( tU32 , tU32, tU32 *);

extern tU32                       LLD_PRCC_GetPCLKRingOscFreq     ( tVoid);
extern tU32                       LLD_PRCC_GetPCLK_RIOSCFrequency ( tU32* trim_out);
extern LLD_PRCC_Pin2PinStatusTy   LLD_PRCC_GetPin2PinStatus       ( tVoid);

extern boolean_t                  LLD_PRCC_GetOscillatorStatus    ( tVoid);
extern tVoid                      LLD_PRCC_SetTHSensFSMMode       ( LLD_PRCC_THSENSFSMModeTy);
extern tVoid                      LLD_PRCC_SetTHSensCNTPreset     ( tU32 value);
extern tVoid                      LLD_PRCC_SetTHSensPADOutputVal  ( tU32 value);
extern tVoid                      LLD_PRCC_SetTHSensEN            ( LLD_PRCC_THSENSPADDirTy);
extern tVoid                      LLD_PRCC_SetTHSensSTART         ( tU32 value);
extern boolean_t                  LLD_PRCC_GetTHSensRDYStatus     ( tVoid);
extern tInt                       LLD_PRCC_GetTHSensCNTOut        ( tVoid);
extern boolean_t                  LLD_PRCC_GetTHSensUPDACK        ( tVoid);
extern tInt                       LLD_PRCC_GetTHSensFSMSTATE      ( tVoid);

extern tVoid                      LLD_PRCC_SetROMPatchMode        ( LLD_PRCC_ROMPatchMode mode);
#endif  // LLD_PRCC_STA8090_H

//!   \}

//!   \}

