//!
//!   \file     lld_i2c.h
//!   \brief    <i><b>I2C low level driver header file</b></i>
//!   \author   Maristella Frazzetto
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_I2C_H
#define LLD_I2C_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

#ifdef __cplusplus
extern "C" {
#endif

//!   \addtogroup LLD
//!   \{
//!   \addtogroup I2C
//!   \{
//!   \addtogroup I2C_defines
//!   \{

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief I2C id
typedef void *     LLD_I2C_IdTy;

//! \brief Interrupt source
typedef tU32        LLD_I2C_IrqSrcTy;

//! \brief Interrupt types
typedef enum
{
  /*   For I2C0 HS controller  STA_2062 */
  LLD_I2C_IRQ_TRANSMIT_FIFO_EMPTY         = 0x1,
  LLD_I2C_IRQ_TRANSMIT_FIFO_NEARLY_EMPTY  = 0x2,
  LLD_I2C_IRQ_TRANSMIT_FIFO_FULL          = 0x4,
  LLD_I2C_IRQ_TRANSMIT_FIFO_OVERRUN       = 0x8,

  LLD_I2C_IRQ_RECEIVE_FIFO_EMPTY          = 0x10,
  LLD_I2C_IRQ_RECEIVE_FIFO_NEARLY_FULL    = 0x20,
  LLD_I2C_IRQ_RECEIVE_FIFO_FULL           = 0x40,

  LLD_I2C_IRQ_READ_FROM_SLAVE_REQUEST     = 0x10000,
  LLD_I2C_IRQ_READ_FROM_SLAVE_EMPTY       = 0x20000,
  LLD_I2C_IRQ_WRITE_TO_SLAVE_REQUEST      = 0x40000,
  LLD_I2C_IRQ_MASTER_TRANSACTION_DONE     = 0x80000,
  LLD_I2C_IRQ_SLAVE_TRANSACTION_DONE      = 0x100000,

  LLD_I2C_IRQ_MASTER_ARBITRATION_LOST     = 0x1000000,
  LLD_I2C_IRQ_BUS_ERROR                   = 0x2000000,
  LLD_I2C_IRQ_MASTER_TRANSACTION_DONE_WS  = 0x10000000,

  LLD_I2C_IRQ_ALL_CLEARABLE               = 0x131F0008,
  LLD_I2C_IRQ_ALL                         = 0x131F007F
} LLD_I2C_IntTy;

//! \brief Bus Control mode types
typedef enum
{
  LLD_I2C_BUSCTRLMODE_SLAVE,                 /**<               Slave Mode */
  LLD_I2C_BUSCTRLMODE_MASTER,                /**<              Master Mode */
  LLD_I2C_BUSCTRLMODE_MASTER_SLAVE           /**<  Dual Configuration Mode */
} LLD_I2C_BusCtrlModeTy;

//! \brief Speed mode types
typedef enum
{
  LLD_I2C_STANDARD_MODE     =  100000,  /**< 100 kb/s */
  LLD_I2C_FAST_MODE         =  400000,  /**< 400 kb/s */
  LLD_I2C_HIGH_SPEED_MODE   = 3400000   /**< 3.4 Mb/s */
} LLD_I2C_SpeedModeTy;

//! \brief General call mode types
typedef enum
{
  LLD_I2C_SGCM_TRANSPARENT,
  LLD_I2C_SGCM_DIRECT
} LLD_I2C_SGCMTy;

//! \brief Digital filter types
typedef enum
{
  LLD_I2C_DGTLFILTER_OFF,
  LLD_I2C_DGTLFILTER_1CLK_SPIKES,
  LLD_I2C_DGTLFILTER_2CLK_SPIKES,
  LLD_I2C_DGTLFILTER_4CLK_SPIKES
} LLD_I2C_DigitalFilterTy;

//! \brief Address mode types
typedef enum
{
  LLD_I2C_ADDRMODE_GENERALCALL,
  LLD_I2C_ADDRMODE_7BITS,
  LLD_I2C_ADDRMODE_10BITS
} LLD_I2C_AddrModeTy;

//! \brief Master mode types
typedef enum
{
  LLD_I2C_MSTMODE_WRITE,
  LLD_I2C_MSTMODE_READ
} LLD_I2C_MasterModeTy;

//! \brief Start procedure types
typedef enum
{
  LLD_I2C_STARTPROC_DISABLED,
  LLD_I2C_STARTPROC_ENABLED
} LLD_I2C_StartProcTy;

//! \brief Stop condition types
typedef enum
{
  LLD_I2C_STOPCOND_REPEATEDSTART,
  LLD_I2C_STOPCOND_STOP
} LLD_I2C_StopCondTy;

//! \brief Operation types
typedef enum
{
  LLD_I2C_OPSTATUS_MASTER_TX,
  LLD_I2C_OPSTATUS_MASTER_RX,
  LLD_I2C_OPSTATUS_SLAVE_RX,
  LLD_I2C_OPSTATUS_SLAVE_TX,
} LLD_I2C_OpStatusTy;

//! \brief Control status types
typedef enum
{
  LLD_I2C_CTRLSTATUS_NOP,
  LLD_I2C_CTRLSTATUS_ON_GOING,
  LLD_I2C_CTRLSTATUS_OK,
  LLD_I2C_CTRLSTATUS_ABORT,
} LLD_I2C_CtrlStatusTy;

//! \brief Abort causes
typedef enum
{
  LLD_I2C_ABORTCAUSE_NACK_ADDR,
  LLD_I2C_ABORTCAUSE_NACK_DATA,
  LLD_I2C_ABORTCAUSE_ACK_MCODE,
  LLD_I2C_ABORTCAUSE_ARB_LOST,
  LLD_I2C_ABORTCAUSE_BERR_START,
  LLD_I2C_ABORTCAUSE_BERR_STOP,
  LLD_I2C_ABORTCAUSE_OVFL,
} LLD_I2C_AbortCauseTy;

//! \brief Receive types
typedef enum
{
  LLD_I2C_RXTYPE_FRAME,
  LLD_I2C_RXTYPE_GCALL,
  LLD_I2C_RXTYPE_HW_GCALL,
} LLD_I2C_ReceiveTypeTy;

#if defined( LLD_I2C_DMASUPPORT)
//! \brief DMA state types
typedef enum
{
  LLD_I2C_DMASTATE_IDLE,
  LLD_I2C_DMASTATE_RUN
} LLD_I2C_DMAStateTy;

//! \brief DMA synchronization logic types
typedef enum
{
  LLD_I2C_DMASYNCLOGIC_OFF,
  LLD_I2C_DMASYNCLOGIC_ON
} LLD_I2C_DMASyncLogicTy;


//! \brief Burst mode types
typedef enum
{
  LLD_I2C_DMABURSTMODE_SINGLE,
  LLD_I2C_DMABURSTMODE_BURST
} LLD_I2C_DMABurstModeTy;
#endif


#if 0
/* Uart comunication control block struct definition */
typedef struct
{
  I2CMap *      id;              // I2C id (it corresponds with the peripheral's base address)
  tU8*          pu8WrBuffer;      /* Write buffer start address */
  tU8*          pu8WrBufferEnd;
  tU32          u32WrBuffSize;    /* Size of Write-Buffer */
  tU8*          pu8WrBufferISR;   /* Pointer to Tx Fifo buffer used by ISR */
  tU8*          pu8WrBufferDEV;   /* Pointer to Tx Fifo buffer used by DEVICE */
  tU32          u32WrNbytes;      /* Number of bytes to be Tx */
  /* Info's */
  //   tU32          u32TransmitBytes; /* Total Number of bytes transmitted */
  //   tU32          u32WrNumberOfOverflowBytes; /* Number of bytes thrown away because of buffer overflow */
  //  tU32          u32WrNumberOfOverflows;     /* Number of Buffer-overflow situations */

  tU8*          pu8RdBuffer;      /* Rx Fifo buffer start address */
  tU8*          pu8RdBufferEnd;
  tU32          u32RdBuffSize;    /* Size of Read-Buffer */
  tU8*          pu8RdBufferISR;   /* Pointer to Rx Fifo buffer used by ISR */
  tU8*          pu8RdBufferDEV;   /* Pointer to Rx Fifo buffer used by DEVICE */
  tU32          u32RdNbytes;      /* Number of Bytes in Read buffer */
  /* Info's */
  //  tU32           u32ReceivedBytes; /* Total Number of bytes received */
  // tU32           u32RdNumberOfOverflowBytes; /* Number of bytes thrown away because of buffer overflow */
  //  tU32           u32RdNumberOfOverflows;     /* Number of Buffer-overflow situations */

} I2C_trBuffCtrl;
#endif

typedef union
{
  struct
  {
    tU8 OP:1;
    tU8 A10:1;
    tU8 SB:1;
    tU8 P:1;
    tU8 LENGTH:1;
    tU8 reserved:3;
  }BIT;
  tU8 REG;
}tI2C_MCR_flags;


typedef struct
{
  LLD_I2C_DigitalFilterTy   digital_filter;
  #if defined( LLD_I2C_DMASUPPORT)
  LLD_I2C_DMASyncLogicTy    dma_sync_logic;
  #endif
  LLD_I2C_SpeedModeTy       speed_mode;
  tU32                      input_freq;
  tU32                      out_freq;
  tU16                      fifo_tx_threshold;
  tU16                      fifo_rx_threshold;
  tU16                      slave_address;
  tU16                      slave_data_setup_time;    /* Only valid for HS controller */
  tU8                       hs_master_code;           /* ONLY VALID FOR MASTER MODE TRANSACTIONS*/
  LLD_I2C_StartProcTy       start_byte_proc;          /* ONLY VALID FOR MASTER MODE TRANSACTIONS*/
} LLD_I2C_configTy;

#if 0
typedef struct
{
  curr_operation;
  active_event;
} LLD_I2C_statusTy;
#endif

/*****************************************************************************
   exported variables
*****************************************************************************/

//!   \}
//!   \}
//!   \}

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid                    LLD_I2C_ResetReg                    ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_Enable                      ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_Disable                     ( const LLD_I2C_IdTy id);
extern LLD_ErrorTy              LLD_I2C_SetOperatingMode            ( const LLD_I2C_IdTy id, LLD_I2C_BusCtrlModeTy mode);
extern LLD_I2C_BusCtrlModeTy    LLD_I2C_GetOperatingMode            ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetSpeedMode                ( const LLD_I2C_IdTy id, LLD_I2C_SpeedModeTy clock);
extern LLD_I2C_SpeedModeTy      LLD_I2C_GetSpeedMode                ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetSlaveGeneralCallMode     ( const LLD_I2C_IdTy id, LLD_I2C_SGCMTy mode);
extern LLD_I2C_SGCMTy           LLD_I2C_GetSlaveGeneralCallMode     ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_FlushTX                     ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_FlushRX                     ( const LLD_I2C_IdTy id);

extern tVoid                    LLD_I2C_EnableLoopBack              ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_DisableLoopBack             ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetDigitalFilter            ( const LLD_I2C_IdTy id, LLD_I2C_DigitalFilterTy Filter);
extern LLD_I2C_DigitalFilterTy  LLD_I2C_GetDigitalFilter            ( const LLD_I2C_IdTy id);

extern tBool                    LLD_I2C_SetSlaveAddr                ( const LLD_I2C_IdTy id, const tU32 addr);
extern tU32                     LLD_I2C_GetSlaveAddr                ( const LLD_I2C_IdTy id);
extern LLD_I2C_AddrModeTy       LLD_I2C_GetSlaveAddrMode            ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetSlaveDataSetupTime       ( const LLD_I2C_IdTy id, tU16 time);
extern tU16                     LLD_I2C_GetSlaveDataSetupTime       ( const LLD_I2C_IdTy id);

extern tVoid                    LLD_I2C_SetMasterMode               ( const LLD_I2C_IdTy id, const LLD_I2C_MasterModeTy mode);
extern LLD_I2C_MasterModeTy     LLD_I2C_GetMasterMode               ( const LLD_I2C_IdTy id);
extern tBool                    LLD_I2C_SetDestAddr                 ( const LLD_I2C_IdTy id, const tU32 addr);
extern tU32                     LLD_I2C_GetDestAddr                 ( const LLD_I2C_IdTy id);
extern LLD_I2C_AddrModeTy       LLD_I2C_GetDestAddrMode             ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetMasterStartByteProcedure ( const LLD_I2C_IdTy id, const LLD_I2C_StartProcTy proc);
extern LLD_I2C_StartProcTy      LLD_I2C_GetMasterStartByteProcedure ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetMasterStopCondition      ( const LLD_I2C_IdTy id, const LLD_I2C_StopCondTy cond);
extern LLD_I2C_StopCondTy       LLD_I2C_GetMasterStopCondition      ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetMasterTransactionLength  ( const LLD_I2C_IdTy id, const tU32 len);
extern tU32                     LLD_I2C_GetMasterTransactionLength  ( const LLD_I2C_IdTy id);

extern LLD_I2C_OpStatusTy       LLD_I2C_GetOperationStatus          ( const LLD_I2C_IdTy id);
extern LLD_I2C_CtrlStatusTy     LLD_I2C_GetControllerStatus         ( const LLD_I2C_IdTy id);
extern LLD_I2C_AbortCauseTy     LLD_I2C_GetAbortCause               ( const LLD_I2C_IdTy id);
extern LLD_I2C_ReceiveTypeTy    LLD_I2C_GetReceiveType              ( const LLD_I2C_IdTy id);
extern tU32                     LLD_I2C_GetTransmitLength           ( const LLD_I2C_IdTy id);

extern tVoid                    LLD_I2C_SetTxThreshold              ( const LLD_I2C_IdTy id, const tU32 threshold);
extern tU32                     LLD_I2C_GetTxThreshold              ( const LLD_I2C_IdTy id);

extern tVoid                    LLD_I2C_SetRxThreshold              ( const LLD_I2C_IdTy id, const tU32 threshold);
extern tU32                     LLD_I2C_GetRxThreshold              ( const LLD_I2C_IdTy id);

extern tVoid                    LLD_I2C_SetBaudRate                 ( const LLD_I2C_IdTy id, const tU32 InputFreq);
extern tVoid                    LLD_I2C_SetBaudRateSpeed            ( const LLD_I2C_IdTy id, const tU32 InputFreq, tU32 out_freq);

extern tVoid                    LLD_I2C_InterruptEnable             ( const LLD_I2C_IdTy id, const LLD_I2C_IrqSrcTy irq_mask);
extern tVoid                    LLD_I2C_InterruptDisable            ( const LLD_I2C_IdTy id, const LLD_I2C_IrqSrcTy irq_mask);
extern LLD_I2C_IrqSrcTy         LLD_I2C_GetInterruptStatus          ( const LLD_I2C_IdTy id, const LLD_I2C_IrqSrcTy irq_mask);
extern LLD_I2C_IrqSrcTy         LLD_I2C_GetRawInterruptStatus       ( const LLD_I2C_IdTy id, const LLD_I2C_IrqSrcTy irq_mask);
extern tVoid                    LLD_I2C_ClearInterrupt              ( const LLD_I2C_IdTy id, const LLD_I2C_IrqSrcTy irq_mask);

extern LLD_ErrorTy              LLD_I2C_SetMCR                      ( const LLD_I2C_IdTy id, const LLD_I2C_MasterModeTy op_mode, const tU16 dest_addr, const LLD_I2C_StartProcTy start_proc, const LLD_I2C_StopCondTy stop_cond, const tU32 len);

extern tU32                     LLD_I2C_WriteFifo                   ( const LLD_I2C_IdTy id, const tU8 * const buf_ptr, const tU32 buf_size);
extern tU32                     LLD_I2C_ReadFifo                    ( const LLD_I2C_IdTy id, tU8 * const buf_ptr, const tU32 buf_size);
extern tVoid                    LLD_I2C_ReadData                    ( const LLD_I2C_IdTy id, tU8 * p_data);
extern tVoid                    LLD_I2C_WriteDataFifo               ( const LLD_I2C_IdTy id, tU8 data);
extern tU8                      LLD_I2C_ReadDataFifo                ( const LLD_I2C_IdTy id);

extern tVoid                    LLD_I2C_Config                      ( const LLD_I2C_IdTy id, const LLD_I2C_configTy *cfg);
extern LLD_ErrorTy              LLD_I2C_MasterWriteIndex            ( const LLD_I2C_IdTy id, const tU16 dest_addr, const tU8 index, const tU8 data);

extern tVoid                    LLD_I2C_SetStartCondHoldTime        ( const LLD_I2C_IdTy id, const LLD_I2C_SpeedModeTy mode, const tU32 hold_time);
extern tVoid                    LLD_I2C_SetStartCondSetupTime       ( const LLD_I2C_IdTy id, const LLD_I2C_SpeedModeTy mode, const tU32 setup_time);
extern tVoid                    LLD_I2C_SetHoldTime                 ( const LLD_I2C_IdTy id, tU32 holdtime);
extern tVoid                    LLD_I2C_EnableTM                    ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_DisableTM                   ( const LLD_I2C_IdTy id);
extern tU32                     LLD_I2C_GetTMSDAIN                  ( const LLD_I2C_IdTy id);
extern tU32                     LLD_I2C_GetTMSCLIN                  ( const LLD_I2C_IdTy id);
extern tU32                     LLD_I2C_GetTMSDAOUT                 ( const LLD_I2C_IdTy id);
extern tU32                     LLD_I2C_GetTMSCLOUT                 ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetTMSDAOUT                 ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetTMSCLOUT                 ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_ClearTMSDAOUT               ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_ClearTMSCLOUT               ( const LLD_I2C_IdTy id);

#if defined( LLD_I2C_DMASUPPORT)
extern tVoid                    LLD_I2C_SetDMATXState               ( const LLD_I2C_IdTy id, LLD_I2C_DMAStateTy state);
extern LLD_I2C_DMAStateTy       LLD_I2C_GetDMATXState               ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetDMARXState               ( const LLD_I2C_IdTy id, LLD_I2C_DMAStateTy state);
extern LLD_I2C_DMAStateTy       LLD_I2C_GetDMARXState               ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetDMASyncLogic             ( const LLD_I2C_IdTy id, LLD_I2C_DMASyncLogicTy logic);
extern LLD_I2C_DMASyncLogicTy   LLD_I2C_GetDMASyncLogic             ( const LLD_I2C_IdTy id);

extern tVoid                    LLD_I2C_SetHighSpeedMasterCode      ( const LLD_I2C_IdTy id, const tU32 code);
extern tU32                     LLD_I2C_GetHighSpeedMasterCode      ( const LLD_I2C_IdTy id);

extern tVoid                    LLD_I2C_SetDMARxBurstSize           ( const LLD_I2C_IdTy id, const tU32 size);
extern tU32                     LLD_I2C_GetDMARxBurstSize           ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetDMARxBurstMode           ( const LLD_I2C_IdTy id, const LLD_I2C_DMABurstModeTy mode);
extern LLD_I2C_DMABurstModeTy   LLD_I2C_GetDMARxBurstMode           ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetDMATxBurstSize           ( const LLD_I2C_IdTy id, const tU32 size);
extern tU32                     LLD_I2C_GetDMATxBurstSize           ( const LLD_I2C_IdTy id);
extern tVoid                    LLD_I2C_SetDMATxBurstMode           ( const LLD_I2C_IdTy id, const LLD_I2C_DMABurstModeTy mode);
extern LLD_I2C_DMABurstModeTy   LLD_I2C_GetDMATxBurstMode           ( const LLD_I2C_IdTy id);
#endif

// End of file
#ifdef __cplusplus
}   /* Allow C++ to use this header */
#endif  /* __cplusplus              */

#endif  /* _LLD_I2C_H_               */
