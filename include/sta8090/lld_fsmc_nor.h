//!
//!   \file     lld_fsmc_nor.h
//!   \brief    <i><b>FSMC NOR Low Level Driver header file</b></i>
//!   \author   Fulvio Boggia
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_FSMC_NOR_H
#define LLD_FSMC_NOR_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"
#include "lld_fsmc_ctrl.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*! @brief It's the FSMC controller IP base address */
#define LLD_FSMC_ADDRESS                    (FSMC_CNTRL)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  LLD_FSMC_NOR_SAM_FLASH          = 0x00EC,
  LLD_FSMC_NOR_ST_FLASH           = 0x0020
} LLD_FSMC_NOR_MfcIdTy;

typedef enum
{
  LLD_FSMC_NOR_PARAMS_FROM_BASE,
  LLD_FSMC_NOR_PARAMS_FROM_TOP
} LLD_FSMC_NOR_ParamPosTy;

typedef enum
{
  LLD_FSMC_NOR_TWO_EIGHT_BIT      = 0x0,
  LLD_FSMC_NOR_SINGLE_SIXTEEN_BIT = 0X1
} LLD_FSMC_NOR_IfModeTy;

typedef enum
{
  LLD_FSMC_NOR_SIZE_BYTE = 0x0,
  LLD_FSMC_NOR_SIZE_HALF = 0x1,
  LLD_FSMC_NOR_SIZE_WORD = 0x2
} LLD_FSMC_NOR_SizeTy;

typedef struct
{
  tU32 block_num;
  tU32 block_size;
} LLD_FSMC_NOR_RegionTy;

typedef enum
{
  LLD_FSMC_NOR_CF0,
  LLD_FSMC_NOR_CF1,
  LLD_FSMC_NOR_CF2,
  LLD_FSMC_NOR_CF3
} LLD_FSMC_NOR_CFISetTy;

typedef struct
{
  LLD_FSMC_NOR_MfcIdTy    mfc_id;
  tU16                    device_id;
  tBool                   flash_supported;

  LLD_FSMC_NOR_CFISetTy   command_set;
  LLD_FSMC_NOR_SizeTy     step_size;

  tU32                    total_blocks;

  tU8                     number_of_regions;
  LLD_FSMC_NOR_RegionTy   regions[4];

  LLD_FSMC_NOR_ParamPosTy parameter_position;
  LLD_FSMC_NOR_IfModeTy   interface_mode;
} LLD_FSMC_ParamsTy;

typedef enum
{
  LLD_FSMC_NORSTATUS_READ,
  LLD_FSMC_NORSTATUS_BUSY,
  LLD_FSMC_NORSTATUS_PROGRAMMING,
  LLD_FSMC_NORSTATUS_ERASING,
  LLD_FSMC_NORSTATUS_SUSPENDED,
  LLD_FSMC_NORSTATUS_ERROR
} LLD_FSMC_NorStatusTy;

typedef struct
{
  LLD_FSMC_SRAMNORBankTy        bank_no;
  volatile tU32 *               bank_start_addr;
  LLD_FSMC_ParamsTy             params;
} LLD_FSMC_NorCfgTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tU32                   LLD_FSMC_NorBlockAddr         ( const LLD_FSMC_NorCfgTy * const cfg_ptr, const tU16 block_num);
extern tU32                   LLD_FSMC_NorBlockSize         ( const LLD_FSMC_NorCfgTy * const cfg_ptr, const tU16 block_num);
extern LLD_ErrorTy            LLD_FSMC_NorAddrToBlockPos    ( const LLD_FSMC_NorCfgTy * const cfg_ptr, const tU32 addr, tU32 *block_idx, tU32 *block_pos);

extern LLD_ErrorTy            LLD_FSMC_NorConfigure         ( LLD_FSMC_NorCfgTy * const cfg_ptr);

extern LLD_ErrorTy            LLD_FSMC_NorUnlockBlock       ( const LLD_FSMC_NorCfgTy * const cfg_ptr, const tU16 block_num);
extern LLD_ErrorTy            LLD_FSMC_NorEraseBlock        ( LLD_FSMC_NorCfgTy * const cfg_ptr, const tU32 block_num);
extern LLD_ErrorTy            LLD_FSMC_NorEraseSuspend      ( LLD_FSMC_NorCfgTy * const cfg_ptr, const tU32 block_num);
extern LLD_ErrorTy            LLD_FSMC_NorEraseResume       ( LLD_FSMC_NorCfgTy * const cfg_ptr, const tU32 block_num);
extern LLD_ErrorTy            LLD_FSMC_NorProgram           ( LLD_FSMC_NorCfgTy * const cfg_ptr, const tU32 nor_dest_addr, const tVoid *buf_ptr);
extern LLD_ErrorTy            LLD_FSMC_NorResetAddrStatus   ( LLD_FSMC_NorCfgTy * const cfg_ptr, const void *addr);
extern LLD_ErrorTy            LLD_FSMC_NorResetBlockStatus  ( LLD_FSMC_NorCfgTy * const cfg_ptr, const tU32 block_num);

extern LLD_FSMC_NorStatusTy   LLD_FSMC_NorGetAddrStatus     ( LLD_FSMC_NorCfgTy * const cfg_ptr, const void *addr, LLD_FSMC_NorStatusTy operation_ongoing);
extern LLD_FSMC_NorStatusTy   LLD_FSMC_NorGetBlockStatus    ( LLD_FSMC_NorCfgTy * const cfg_ptr, const tU32 block_num, LLD_FSMC_NorStatusTy operation_ongoing);

#endif  // _LLD_FSMC_NOR_H_

// End of file
