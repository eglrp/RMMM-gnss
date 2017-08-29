//!
//!   \file       lld_fsmc_ctrl.h
//!   \brief      <i><b>FSMC controller Low Level Driver header file</b></i>
//!   \author     Fulvio Boggia
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup LLD
//!   \{
//!   \addtogroup FSMC
//!   \{
//!

#ifndef LLD_FSMC_CTRL_H
#define LLD_FSMC_CTRL_H

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

//! \brief Availables banks
typedef enum {
  LLD_FSMC_SRAMNOR_BANK0,
  LLD_FSMC_SRAMNOR_BANK1,
  LLD_FSMC_SRAMNOR_BANK2,
  LLD_FSMC_SRAMNOR_BANK_NUMBER,
} LLD_FSMC_SRAMNORBankTy;

//! \brief Possible types of memory module
typedef enum {
  LLD_FSMC_SRAMNOR_TYPE_SRAM = 0x0,
  LLD_FSMC_SRAMNOR_TYPE_NOR  = 0x2
} LLD_FSMC_SRAMNORMemTypeTy;

//! \brief Allowed data bus configurations
typedef enum {
  LLD_FSMC_SRAMNOR_DATABUS_8BIT,
  LLD_FSMC_SRAMNOR_DATABUS_16BIT
} LLD_FSMC_SRAMNORMemDataBusSizeTy;

//! \brief State of bank
typedef enum {
  LLD_FSMC_STATE_DISABLED,
  LLD_FSMC_STATE_ENABLED
} LLD_FSMC_StateTy;

//! \brief Signal polarity
typedef enum {
  LLD_FSMC_POLARITY_LOW,
  LLD_FSMC_POLARITY_HIGH
} LLD_FSMC_PolarityTy;

//! \brief Bank configuration type
typedef struct
{
  union
  {
    struct
    {
      tU32 bank_status                : 1;              /**< Bank enable bit                          */
      tU32 addr_data_muxed            : 1;              /**< Address/Data muxed enable bit            */
      tU32 mem_type                   : 2;              /**< Memory type                              */
      tU32 mem_data_bus_width         : 2;              /**< Memory data bus width                    */
      tU32 res0                       : 2;
      tU32 burst_mode                 : 1;              /**< Burst enable bit                         */
      tU32 wait_sig_pol               : 1;              /**< Wait signal polarity                     */
      tU32 res1                       : 1;
      tU32 wait_tim_cfg               : 1;              /**< Wait timing configuration                */
      tU32 write_mode                 : 1;              /**< Write enable bit                         */
      tU32 bus_turnaround_enable      : 1;              /**< Bus turn around enable bit               */
      tU32 res2                       : 1;
      tU32 wait_mode                  : 1;              /**< Wait async enable bit                    */
    } BIT;
    tU32 REG;
  } config;                                           /**< SRAM/NOR BANK configuration parameters   */
  union
  {
    struct
    {
      tU32 address_setup_phase        : 4;              /**< Address setup phase duration             */
      tU32 address_hold_phase         : 4;              /**< Address hold phase duration              */
      tU32 data_phase                 : 8;              /**< Data phase duration                      */
      tU32 bus_turnaround_phase       : 4;              /**< Bus turn around phase duration           */
      tU32 clock_divider              : 4;              /**< Clock divide ratio                       */
      tU32 data_latency               : 4;              /**< Data latency                             */
    } BIT;
    tU32 REG;
  } timings;                                          /**< SRAM/NOR BANK timings parameters         */
} LLD_FSMC_CfgTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid                              LLD_FSMC_ConfigureBank            ( const LLD_FSMC_SRAMNORBankTy bank, const LLD_FSMC_CfgTy *bank_cfg_ptr);
extern tVoid                              LLD_FSMC_EnableBank               ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_DisableBank              ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_EnableADMux              ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_DisableADMux             ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetMemType               ( const LLD_FSMC_SRAMNORBankTy bank, const LLD_FSMC_SRAMNORMemTypeTy type);
extern LLD_FSMC_SRAMNORMemTypeTy          LLD_FSMC_GetMemType               ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetMemDataBusSize        ( const LLD_FSMC_SRAMNORBankTy bank, const LLD_FSMC_SRAMNORMemDataBusSizeTy size);
extern LLD_FSMC_SRAMNORMemDataBusSizeTy   LLD_FSMC_GetMemDataBusSize        ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_EnableBurstMode          ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_DisableBurstMode         ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetFlashWaitPolHigh      ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetFlashWaitPolLow       ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_EnableWrite              ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_DisableWrite             ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_EnableBusTurnAround      ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_DisableBusTurnAround     ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_EnableWaitAsync          ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_DisableWaitAsync         ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetAddrSetupPhase        ( const LLD_FSMC_SRAMNORBankTy bank, const tU32 clk_ticks);
extern tU32                               LLD_FSMC_GetAddrSetupPhase        ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetAddrSetupHold         ( const LLD_FSMC_SRAMNORBankTy bank, const tU32 clk_ticks);
extern tU32                               LLD_FSMC_GetAddrSetupHold         ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetDataPhase             ( const LLD_FSMC_SRAMNORBankTy bank, const tU32 clk_ticks);
extern tU32                               LLD_FSMC_GetDataPhase             ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetBusTurnAround         ( const LLD_FSMC_SRAMNORBankTy bank, const tU32 clk_ticks);
extern tU32                               LLD_FSMC_GetBusTurnAround         ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetClockDivisor          ( const LLD_FSMC_SRAMNORBankTy bank, const tU32 clk_ticks);
extern tU32                               LLD_FSMC_GetClockDivisor          ( const LLD_FSMC_SRAMNORBankTy bank);
extern tVoid                              LLD_FSMC_SetDataLatency           ( const LLD_FSMC_SRAMNORBankTy bank, const tU32 clk_ticks);
extern tU32                               LLD_FSMC_GetDataLatency           ( const LLD_FSMC_SRAMNORBankTy bank);

#endif  // _LLD_FSMC_CTRL_H_

//!   \}

//!   \}
