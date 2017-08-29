//!
//!   \file     lld_sqi_ctrl.h
//!   \brief    <i><b>SQI Controller low level driver header file</b></i>
//!   \author   Giovanni De Angelis
//!   \version  2.0
//!   \date     2011.06.13
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_SQI_CTRL_H
#define LLD_SQI_CTRL_H

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

#define LLD_SQI_SECTOR_SIZE           (64 * 1024)
#define LLD_SQI_SUBSECTOR_SIZE        (4 * 1024)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  LLD_SQI_MODE_SPI        = 0x0U,
  LLD_SQI_MODE_QPI        = 0x1U,
  LLD_SQI_MODE_FULL_QSPI  = 0x2U,
  LLD_SQI_MODE_QSPI       = 0x3U
} LLD_SQI_ModeTy;

typedef enum
{
  LLD_SQI_SPIMODE_0 = 0x0U,
  LLD_SQI_SPIMODE_3 = 0x1U
} LLD_SQI_SPIModeTy;

typedef enum
{
  LLD_SQI_DUMMYCYCLES_NONE      = 0x00U,
  LLD_SQI_DUMMYCYCLES_QPI_2     = 0x10U,
  LLD_SQI_DUMMYCYCLES_QPI_4     = 0x11U,
  LLD_SQI_DUMMYCYCLES_QPI_6     = 0x12U,
  LLD_SQI_DUMMYCYCLES_QPI_8     = 0x13U,
  LLD_SQI_DUMMYCYCLES_SPI_16    = 0x20U,
  LLD_SQI_DUMMYCYCLES_SPI_32    = 0x21U,
  LLD_SQI_DUMMYCYCLES_SPI_64    = 0x22U,
  LLD_SQI_DUMMYCYCLES_SPI_128   = 0x23U
} LLD_SQI_DummyCyclesTy;

typedef enum
{
  LLD_SQI_TYPE_MACRONIX,
  LLD_SQI_TYPE_MICRON,
  LLD_SQI_TYPE_SST,
  LLD_SQI_TYPE_WINBOND,
  LLD_SQI_TYPE_SPANSION,
  LLD_SQI_TYPE_MACRONIX_QSPI,
  LLD_SQI_TYPE_ISSI
} LLD_SQI_TypeTy;

typedef enum
{
  LLD_SQI_READY,
  LLD_SQI_BUSY,
  LLD_SQI_SUSPENDED,
  LLD_SQI_ERROR
} LLD_SQI_StatusTy;

typedef struct
{
  LLD_SQI_ModeTy          mode;
  LLD_SQI_TypeTy          type;
  LLD_SQI_DummyCyclesTy   dummy_cycles;
  tU8                     divisor;
} LLD_SQI_ConfigTy;

typedef enum
{
  LLD_SQI_WORDSIZE_BYTE,    /**< Word to write is a byte */
  LLD_SQI_WORDSIZE_HALF,    /**< Word to write is a 16 bit word */
  LLD_SQI_WORDSIZE_WORD     /**< Word to write is a 32 bit word */
} LLD_SQI_WordSizeTy;

typedef enum
{
  LLD_SQI_STEP_SIZE_BYTE = 0x1U,
  LLD_SQI_STEP_SIZE_HALF = 0x2U,
  LLD_SQI_STEP_SIZE_WORD = 0x4U
} LLD_SQI_StepSizeTy;

typedef enum
{
  LLD_SQI_SIZE_UNKNOWN,
  LLD_SQI_SIZE_2_MBYTE,
  LLD_SQI_SIZE_4_MBYTE,
  LLD_SQI_SIZE_8_MBYTE,
  LLD_SQI_SIZE_16_MBYTE,
  LLD_SQI_SIZE_32_MBYTE
} LLD_SQI_SizeTy;

typedef struct
{
  LLD_SQI_StatusTy        operation_ongoing;
  LLD_SQI_StatusTy        last_status;
  tU32                    address;
} LLD_SQI_ProgEraseStatusTy;

typedef struct
{
  LLD_SQI_TypeTy          type;
  LLD_SQI_StepSizeTy      step_size;
  LLD_SQI_SizeTy          size;
} LLD_SQI_CfgTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern LLD_ErrorTy      LLD_SQI_Configure             ( LLD_SQI_CfgTy *);
extern tVoid            LLD_SQI_InitStatusRegisters   ( LLD_SQI_TypeTy type, tU8 spm_conf);
extern tVoid            LLD_SQI_Reset                 ( LLD_SQI_TypeTy type);
extern tVoid            LLD_SQI_EnableQuadIO          ( LLD_SQI_TypeTy type);
extern tVoid            LLD_SQI_EnableQPI             ( LLD_SQI_TypeTy type);
extern tVoid            LLD_SQI_EnableQSPI            ( LLD_SQI_TypeTy type);
extern tVoid            LLD_SQI_DisableQPI            ( LLD_SQI_TypeTy type);
extern tVoid            LLD_SQI_SendAtomicCommand     ( tU8 command);
extern tVoid            LLD_SQI_SendDataCommand       ( tU8 command, tU8 *data, tU32 size, tBool write_en);
extern LLD_ErrorTy      LLD_SQI_WritePageBuffer       ( tU32 *src_ptr, tU32 size);
extern tVoid            LLD_SQI_SetReadOpCode         ( tU8 code);
extern tVoid            LLD_SQI_SetMode               ( tU8 mode);
extern tVoid            LLD_SQI_SetCommand            ( tU8 cmd);
extern tVoid            LLD_SQI_SetTransferLength     ( tU8 len);
extern tVoid            LLD_SQI_SetAddress            ( tU32 addr);
extern tVoid            LLD_SQI_EnableInterrupt       ( tVoid);
extern tVoid            LLD_SQI_DisableInterrupt      ( tVoid);
extern tBool            LLD_SQI_GetRawInterruptStatus ( tVoid);
extern tVoid            LLD_SQI_SwReset               ( tVoid);
extern tVoid            LLD_SQI_SetDummyCycles        ( tBool enable, LLD_SQI_DummyCyclesTy dummy_cycles);
extern tVoid            LLD_SQI_SetDivisor            ( tU8 divisor);
extern tVoid            LLD_SQI_ChipErase             ( void);
extern tVoid            LLD_SQI_SectorErase           ( tU32);
extern tVoid            LLD_SQI_SubSectorErase        ( tU32);
extern tVoid            LLD_SQI_AddrToSectorNum       ( tU32, tU32 *);
extern tVoid            LLD_SQI_AddrToSubSectorNum    ( tU32, tU32 *);
extern tVoid            LLD_SQI_WriteArea             ( tU32 *, tU32, tU32);
extern LLD_ErrorTy      LLD_SQI_WriteWord             ( tU32, tU32, LLD_SQI_WordSizeTy);
extern LLD_ErrorTy      LLD_SQI_Program               ( LLD_SQI_CfgTy *, tU32, tVoid *);
extern tVoid            LLD_SQI_ResetStatus           ( LLD_SQI_TypeTy type);
extern tVoid            LLD_SQI_ProgramEraseSuspend   ( LLD_SQI_TypeTy type);
extern tVoid            LLD_SQI_ProgramEraseResume    ( LLD_SQI_TypeTy type);
extern tVoid            LLD_SQI_SetSwProtectionMode   ( LLD_SQI_TypeTy type, tU8 spm);

extern LLD_SQI_StatusTy LLD_SQI_GetStatus             ( LLD_SQI_TypeTy type);
extern tU8              LLD_SQI_GetSwProtectionMode   ( LLD_SQI_TypeTy type);

#endif  // LLD_SQI_CTRL_H

// End of file
