//!
//!   \file     lld_i2c.h
//!   \brief    <i><b>MSP low level driver header file</b></i>
//!   \author   Fulvio Boggia
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_MSP_H
#define LLD_MSP_H

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

#define LLD_MSP_REGRESETVALUE   0x0
#define LLD_MSP_FIFOSIZE        8

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef void * LLD_MSP_IdTy;

typedef enum
{
  LLD_MSP_PHASEMODE_SINGLE              = 0,
  LLD_MSP_PHASEMODE_DUAL                = 1
} LLD_MSP_PhaseModeTy;

typedef enum
{
  /*   For MSP0 HS controller  STA_2062 */
  LLD_MSP_IRQ_RECEIVE                   = 0x1,
  LLD_MSP_IRQ_RECEIVE_OVERRUN_ERROR     = 0x2,
  LLD_MSP_IRQ_RECEIVE_FRAME_SYNC_ERROR  = 0x4,
  LLD_MSP_IRQ_RECEIVE_FRAME_SYNC        = 0x8,

  LLD_MSP_IRQ_TRANSMIT                  = 0x10,
  LLD_MSP_IRQ_TRANSMIT_OVERRUN_ERROR    = 0x20,
  LLD_MSP_IRQ_TRANSMIT_FRAME_SYNC_ERROR = 0x40,
  LLD_MSP_IRQ_TRANSMIT_FRAME_SYNC       = 0x80,

  LLD_MSP_IRQ_RX_CLEARABLE              = 0x0F,
  LLD_MSP_IRQ_TX_CLEARABLE              = 0xF0,
  LLD_MSP_IRQ_ALL_CLEARABLE             = 0xFF,

  LLD_MSP_IRQ_RECEIVE_FIFO_NOT_EMPTY    = 0x100,
  LLD_MSP_IRQ_TRANSMIT_FIFO_NOT_FULL    = 0x200,

  LLD_MSP_IRQ_ALL                       = 0x2FF
} LLD_MSP_IRQSrcIdTy;

typedef tUInt LLD_MSP_IRQSrcTy;

typedef enum
{
  LLD_MSP_FRAME_SYNC_POL_HIGH,
  LLD_MSP_FRAME_SYNC_POL_LOW
} LLD_MSP_FrameSyncPolTy;

typedef enum
{
  LLD_MSP_FRAME_SEL_EXTERNAL            = 0x0,
  LLD_MSP_FRAME_SEL_GEN_LOGIC           = 0x2,
  LLD_MSP_FRAME_SEL_GEN_LOGIC_PERIOD    = 0x3
} LLD_MSP_FrameSelTy;

typedef enum
{
  LLD_MSP_CLOCK_POL_RISE,
  LLD_MSP_CLOCK_POL_FALL
} LLD_MSP_ClockPolTy;

typedef enum
{
  LLD_MSP_CLOCK_SEL_EXT           = 0x0,
  LLD_MSP_CLOCK_SEL_INT           = 0x1,
  LLD_MSP_CLOCK_SEL_FREE_EXT      = 0x2,
  LLD_MSP_CLOCK_SEL_RESYNCRO_EXT  = 0x3
} LLD_MSP_ClockSelTy;

typedef enum
{
  LLD_MSP_SPICLOCKMDODE_NOCLOCK     = 0x0,
  LLD_MSP_SPICLOCKMDODE_ZERO_DELAY  = 0x2,
  LLD_MSP_SPICLOCKMDODE_HALF_DELAY  = 0x3
} LLD_MSP_SPIClockModeTy;

typedef enum
{
  LLD_MSP_SPIBURSTMODE_DISABLE  = 0,
  LLD_MSP_SPIBURSTMODE_ENABLE   = 1
} LLD_MSP_SPIBurstModeTy;

typedef enum
{
  LLD_MSP_BITS4ELEM_8   = 0x0,
  LLD_MSP_BITS4ELEM_10  = 0x1,
  LLD_MSP_BITS4ELEM_12  = 0x2,
  LLD_MSP_BITS4ELEM_14  = 0x3,
  LLD_MSP_BITS4ELEM_16  = 0x4,
  LLD_MSP_BITS4ELEM_20  = 0x5,
  LLD_MSP_BITS4ELEM_24  = 0x6,
  LLD_MSP_BITS4ELEM_32  = 0x7
} LLD_MSP_Bits4ElemTy;

typedef enum
{
  LLD_MSP_DATATYPE_NOCOMPANDING       = 0x0,
  LLD_MSP_DATATYPE_SIGNED_COMPANDING  = 0x1,
  LLD_MSP_DATATYPE_uLAW_COMPANDING    = 0x2,
  LLD_MSP_DATATYPE_ALAW_COMPANDING    = 0x3
} LLD_MSP_DataTypeTy;

typedef enum
{
  LLD_MSP_ENDIANFORM_MSB_FIRST,
  LLD_MSP_ENDIANFORM_LSB_FIRST
} LLD_MSP_EndianFormTy;

typedef enum
{
  LLD_MSP_DATADELAY_0_CLOCK,
  LLD_MSP_DATADELAY_1_CLOCK,
  LLD_MSP_DATADELAY_2_CLOCK,
  LLD_MSP_DATADELAY_3_CLOCK
} LLD_MSP_DataDelayTy;

typedef enum
{
  LLD_MSP_2PHSTARTMODE_IMMEDIATE,
  LLD_MSP_2PHSTARTMODE_DELAY,
} LLD_MSP_2PHStartModeTy;

typedef enum
{
  LLD_MSP_HALFWORDSWAP_NONE,
  LLD_MSP_HALFWORDSWAP_SWAP,
  LLD_MSP_HALFWORDSWAP_EACH_HALF_SWAP,
  LLD_MSP_HALFWORDSWAP_HALF_SWAP
} LLD_MSP_HalfWordSwapTy;

typedef enum
{
  LLD_MSP_STATUSFLAG_RECEIVE_BUSY       = 0x1,
  LLD_MSP_STATUSFLAG_RECEIVE_FIFOEMPTY  = 0x2,
  LLD_MSP_STATUSFLAG_RECEIVE_FIFOFULL   = 0x4,
  LLD_MSP_STATUSFLAG_TRANSMIT_BUSY      = 0x8,
  LLD_MSP_STATUSFLAG_TRANSMIT_FIFOEMPTY = 0x10,
  LLD_MSP_STATUSFLAG_TRANSMIT_FIFOFULL  = 0x20
} LLD_MSP_StatusFlagIdTy;

typedef tUInt LLD_MSP_StatusFlagTy;

typedef enum
{
  LLD_MSP_MCHSUBFRAME_0_TO_31,
  LLD_MSP_MCHSUBFRAME_32_TO_63,
  LLD_MSP_MCHSUBFRAME_64_TO_95,
  LLD_MSP_MCHSUBFRAME_98_TO_127,
} LLD_MSP_MCHSubFrameTy;

typedef enum
{
  LLD_MSP_MCHCOMMD_DISABLED      = 0x0,
  LLD_MSP_MCHCOMMD_B2B_FALSE     = 0x2,
  LLD_MSP_MCHCOMMD_B2B_TRUE      = 0x3
} LLD_MSP_MCHComMDTy;

typedef enum
{
  LLD_MSP_LOOPBACKMODE_DISABLE           = 0,
  LLD_MSP_LOOPBACKMODE_ENABLE            = 1
} LLD_MSP_LoopbackModeTy;

typedef enum
{
  LLD_MSP_DIRCOMPMODE_DISABLE  = 0,
  LLD_MSP_DIRCOMPMODE_ENABLE   = 1
} LLD_MSP_DirCompModeTy;

typedef enum
{
  LLD_MSP_DMAMODE_OFF                    = 0,
  LLD_MSP_DMAMODE_ON                     = 1
} LLD_MSP_DmaModeTy;

typedef enum
{
  LLD_MSP_UNEXPFRAMESYNCMODE_ABORT        = 0,
  LLD_MSP_UNEXPFRAMESYNCMODE_IGNORED      = 1
} LLD_MSP_UnexpFrameSyncModeTy;

typedef enum
{
  LLD_MSP_FIFOMODE_DISABLE                    = 0,
  LLD_MSP_FIFOMODE_ENABLE                     = 1
} LLD_MSP_FifoModeTy;

typedef enum
{
  LLD_MSP_TXEXTRADELAYMODE_OFF              = 0,
  LLD_MSP_TXEXTRADELAYMODE_ON               = 1
} LLD_MSP_TxExtraDelayModeTy;

typedef struct
{
  LLD_MSP_PhaseModeTy           rx_phase_mode;
  LLD_MSP_PhaseModeTy           tx_phase_mode;
  LLD_MSP_2PHStartModeTy        rx_phase2_start_mode;
  LLD_MSP_2PHStartModeTy        tx_phase2_start_mode;
  LLD_MSP_EndianFormTy          rx_bit_transfer_format;
  LLD_MSP_EndianFormTy          tx_bit_transfer_format;
  tU8                           rx_frame_length_1;
  tU8                           rx_frame_length_2;
  tU8                           tx_frame_length_1;
  tU8                           tx_frame_length_2;
  LLD_MSP_Bits4ElemTy           rx_element_length_1;
  LLD_MSP_Bits4ElemTy           rx_element_length_2;
  LLD_MSP_Bits4ElemTy           tx_element_length_1;
  LLD_MSP_Bits4ElemTy           tx_element_length_2;
  LLD_MSP_DataDelayTy           rx_data_delay;
  LLD_MSP_DataDelayTy           tx_data_delay;
  LLD_MSP_ClockPolTy            rx_clock_pol;
  LLD_MSP_ClockPolTy            tx_clock_pol;
  LLD_MSP_FrameSyncPolTy        rx_frame_sync_pol;
  LLD_MSP_FrameSyncPolTy        tx_frame_sync_pol;
  LLD_MSP_HalfWordSwapTy        rx_half_word_swap;
  LLD_MSP_HalfWordSwapTy        tx_half_word_swap;
  LLD_MSP_DataTypeTy            compression_mode;
  LLD_MSP_DataTypeTy            expansion_mode;
  LLD_MSP_SPIClockModeTy        spi_clk_mode;
  LLD_MSP_SPIBurstModeTy        spi_burst_mode;
  tU16                          frame_period;
  tU16                          frame_width;
} LLD_MSP_ProtocolTy;

typedef struct
{
  LLD_MSP_ClockSelTy            srg_clock_sel;
  LLD_MSP_ClockPolTy            sck_pol;
  LLD_MSP_LoopbackModeTy        msp_loopback_mode;
  LLD_MSP_DirCompModeTy         msp_direct_companding_mode;
  LLD_MSP_ClockSelTy            rx_clock_sel;
  LLD_MSP_ClockSelTy            tx_clock_sel;
  LLD_MSP_DmaModeTy             rx_msp_dma_mode;
  LLD_MSP_DmaModeTy             tx_msp_dma_mode;
  LLD_MSP_FrameSelTy            rx_frame_sync_sel;
  LLD_MSP_FrameSelTy            tx_frame_sync_sel;
  LLD_MSP_UnexpFrameSyncModeTy  rx_unexpect_frame_sync;
  LLD_MSP_UnexpFrameSyncModeTy  tx_unexpect_frame_sync;
  LLD_MSP_FifoModeTy            rx_fifo_config;
  LLD_MSP_FifoModeTy            tx_fifo_config;
  LLD_MSP_TxExtraDelayModeTy    tx_extra_delay;
} LLD_MSP_ConfigTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern tVoid                          LLD_MSP_ResetReg                      ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_Configure                     ( const LLD_MSP_IdTy id, LLD_MSP_ConfigTy *cfg, LLD_MSP_ProtocolTy *prot_desc);
extern tVoid                          LLD_MSP_WriteData                     ( const LLD_MSP_IdTy id, tU32 data);
extern tU32                           LLD_MSP_ReadData                      ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxFifoMode                 ( const LLD_MSP_IdTy id, LLD_MSP_FifoModeTy mode);
extern LLD_MSP_FifoModeTy             LLD_MSP_GetRxFifoMode                 ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxFrameSyncroPolarity      ( const LLD_MSP_IdTy id, LLD_MSP_FrameSyncPolTy polarity);
extern LLD_MSP_FrameSyncPolTy         LLD_MSP_GetRxFrameSyncroPolarity      ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetDirCompMode                ( const LLD_MSP_IdTy id, LLD_MSP_DirCompModeTy mode);
extern LLD_MSP_DirCompModeTy          LLD_MSP_GetDirCompMode                ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxFrameSyncroSelection     ( const LLD_MSP_IdTy id, LLD_MSP_FrameSelTy Selection);
extern LLD_MSP_FrameSelTy             LLD_MSP_GetRxFrameSyncroSelection     ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxClockPolarity            ( const LLD_MSP_IdTy id, LLD_MSP_ClockPolTy Polarity);
extern LLD_MSP_ClockPolTy             LLD_MSP_GetRxClockPolarity            ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxClock                    ( const LLD_MSP_IdTy id, LLD_MSP_ClockSelTy clock);
extern LLD_MSP_ClockSelTy             LLD_MSP_GetRxClock                    ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetLoopBackMode               ( const LLD_MSP_IdTy id, LLD_MSP_LoopbackModeTy mode);
extern LLD_MSP_LoopbackModeTy         LLD_MSP_GetLoopBackMode               ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxFifoMode                 ( const LLD_MSP_IdTy id, LLD_MSP_FifoModeTy mode);
extern LLD_MSP_FifoModeTy             LLD_MSP_GetTxFifoMode                 ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxFrameSyncroPolarity      ( const LLD_MSP_IdTy id, LLD_MSP_FrameSyncPolTy polarity);
extern LLD_MSP_FrameSyncPolTy         LLD_MSP_GetTxFrameSyncroPolarity      ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxFrameSyncroSelection     ( const LLD_MSP_IdTy id, LLD_MSP_FrameSelTy Selection);
extern LLD_MSP_FrameSelTy             LLD_MSP_GetTxFrameSyncroSelection     ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxClockPolarity            ( const LLD_MSP_IdTy id, LLD_MSP_ClockPolTy Polarity);
extern LLD_MSP_ClockPolTy             LLD_MSP_GetTxClockPolarity            ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxClock                    ( const LLD_MSP_IdTy id, LLD_MSP_ClockSelTy clock);
extern LLD_MSP_ClockSelTy             LLD_MSP_GetTxClock                    ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxDataExtraDelayMode       ( const LLD_MSP_IdTy id, LLD_MSP_TxExtraDelayModeTy mode);
extern LLD_MSP_TxExtraDelayModeTy     LLD_MSP_GetTxDataExtraDelayMode       ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_DisSampleRateGen              ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EnSampleRateGen               ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetSampleRateGenClockPolarity ( const LLD_MSP_IdTy id, LLD_MSP_ClockPolTy Polarity);
extern LLD_MSP_ClockPolTy             LLD_MSP_GetSampleRateGenClockPolarity ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetSampleRateGenClock         ( const LLD_MSP_IdTy id, LLD_MSP_ClockSelTy clock);
extern LLD_MSP_ClockSelTy             LLD_MSP_GetSampleRateGenClock         ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_DisFrameGen                   ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EnFrameGen                    ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetSPIClockMode               ( const LLD_MSP_IdTy id, LLD_MSP_SPIClockModeTy mode);
extern LLD_MSP_SPIClockModeTy         LLD_MSP_GetSPIClockMode               ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetSPIBurstMode               ( const LLD_MSP_IdTy id, LLD_MSP_SPIBurstModeTy mode);
extern LLD_MSP_SPIBurstModeTy         LLD_MSP_GetSPIBurstMode               ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxElemLength1PH            ( const LLD_MSP_IdTy id, LLD_MSP_Bits4ElemTy length);
extern LLD_MSP_Bits4ElemTy            LLD_MSP_GetTxElemLength1PH            ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxElemNumb1PH              ( const LLD_MSP_IdTy id, tU8 amount);
extern tU8                            LLD_MSP_GetTxElemNumb1PH              ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxDataType                 ( const LLD_MSP_IdTy id, LLD_MSP_DataTypeTy type);
extern LLD_MSP_DataTypeTy             LLD_MSP_GetTxDataType                 ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxEndianForm               ( const LLD_MSP_IdTy id, LLD_MSP_EndianFormTy type);
extern LLD_MSP_EndianFormTy           LLD_MSP_GetTxEndianForm               ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxDataDelay                ( const LLD_MSP_IdTy id, LLD_MSP_DataDelayTy delay);
extern LLD_MSP_DataDelayTy            LLD_MSP_GetTxDataDelay                ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxUnexpFrameSyncMode       ( const LLD_MSP_IdTy id, LLD_MSP_UnexpFrameSyncModeTy mode);
extern LLD_MSP_UnexpFrameSyncModeTy   LLD_MSP_GetTxUnexpFrameSyncMode       ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxElemLength2PH            ( const LLD_MSP_IdTy id, LLD_MSP_Bits4ElemTy length);
extern LLD_MSP_Bits4ElemTy            LLD_MSP_GetTxElemLength2PH            ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxElemNumb2PH              ( const LLD_MSP_IdTy id, tU8 amount);
extern tU8                            LLD_MSP_GetTxElemNumb2PH              ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxStartMode2PH             ( const LLD_MSP_IdTy id, LLD_MSP_2PHStartModeTy mode);
extern LLD_MSP_2PHStartModeTy         LLD_MSP_GetTxStartMode2PH             ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTx2PH                      ( const LLD_MSP_IdTy id, LLD_MSP_PhaseModeTy mode);
extern LLD_MSP_PhaseModeTy            LLD_MSP_GetTx2PH                      ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetTxHalfWordSwap             ( const LLD_MSP_IdTy id, LLD_MSP_HalfWordSwapTy swap);
extern LLD_MSP_HalfWordSwapTy         LLD_MSP_GetTxHalfWordSwap             ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxElemLength1PH            ( const LLD_MSP_IdTy id, LLD_MSP_Bits4ElemTy length);
extern LLD_MSP_Bits4ElemTy            LLD_MSP_GetRxElemLength1PH            ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxElemNumb1PH              ( const LLD_MSP_IdTy id, tU8 amount);
extern tU8                            LLD_MSP_GetRxElemNumb1PH              ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxDataType                 ( const LLD_MSP_IdTy id, LLD_MSP_DataTypeTy type);
extern LLD_MSP_DataTypeTy             LLD_MSP_GetRxDataType                 ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxEndianForm               ( const LLD_MSP_IdTy id, LLD_MSP_EndianFormTy type);
extern LLD_MSP_EndianFormTy           LLD_MSP_GetRxEndianForm               ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxDataDelay                ( const LLD_MSP_IdTy id, LLD_MSP_DataDelayTy delay);
extern LLD_MSP_DataDelayTy            LLD_MSP_GetRxDataDelay                ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxUnexpFrameSyncMode       ( const LLD_MSP_IdTy id, LLD_MSP_UnexpFrameSyncModeTy mode);
extern LLD_MSP_UnexpFrameSyncModeTy   LLD_MSP_GetRxUnexpFrameSyncMode       ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxElemLength2PH            ( const LLD_MSP_IdTy id, LLD_MSP_Bits4ElemTy length);
extern LLD_MSP_Bits4ElemTy            LLD_MSP_GetRxElemLength2PH            ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxElemNumb2PH              ( const LLD_MSP_IdTy id, tU8 amount);
extern tU8                            LLD_MSP_GetRxElemNumb2PH              ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxStartMode2PH             ( const LLD_MSP_IdTy id, LLD_MSP_2PHStartModeTy mode);
extern LLD_MSP_2PHStartModeTy         LLD_MSP_GetRxStartMode2PH             ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRx2PH                      ( const LLD_MSP_IdTy id, LLD_MSP_PhaseModeTy mode);
extern LLD_MSP_PhaseModeTy            LLD_MSP_GetRx2PH                      ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxHalfWordSwap             ( const LLD_MSP_IdTy id, LLD_MSP_HalfWordSwapTy swap);
extern LLD_MSP_HalfWordSwapTy         LLD_MSP_GetRxHalfWordSwap             ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_ConfigSampleRateGen           ( const LLD_MSP_IdTy id, tU32 ClockFreq, tU32 FrameFreq, tU32 DataClock,tU8 width);
extern tVoid                          LLD_MSP_GetSampleFreq                 ( const LLD_MSP_IdTy id, tU32 ClockFreq, tU32 *SampleFreq, tU32 *period);
extern tVoid                          LLD_MSP_SetActiveFrameWidth           ( const LLD_MSP_IdTy id, tU8 width);
extern tU8                            LLD_MSP_GetFrameWidth                 ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetFramePeriod                ( const LLD_MSP_IdTy id, tU16 period);
extern tU16                           LLD_MSP_GetFramePeriod                ( const LLD_MSP_IdTy id);
extern tBool                          LLD_MSP_GetFlagStatus                 ( const LLD_MSP_IdTy id, LLD_MSP_StatusFlagTy flag);
extern tVoid                          LLD_MSP_SetTxDmaMode                  ( const LLD_MSP_IdTy id, LLD_MSP_DmaModeTy mode);
extern LLD_MSP_DmaModeTy              LLD_MSP_GetTxDmaMode                  ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxDmaMode                  ( const LLD_MSP_IdTy id, LLD_MSP_DmaModeTy mode);
extern LLD_MSP_DmaModeTy              LLD_MSP_GetRxDmaMode                  ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_IRQEnable                     ( const LLD_MSP_IdTy id, LLD_MSP_IRQSrcTy irq_mask);
extern tVoid                          LLD_MSP_IRQDisable                    ( const LLD_MSP_IdTy id, LLD_MSP_IRQSrcTy irq_mask);
extern tU16                           LLD_MSP_GetIRQConfig                  ( const LLD_MSP_IdTy id);
extern tU16                           LLD_MSP_GetRawIRQStatus               ( const LLD_MSP_IdTy id);
extern tU16                           LLD_MSP_GetIRQStatus                  ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_IRQClear                      ( const LLD_MSP_IdTy id, LLD_MSP_IRQSrcTy irq_mask);
extern tVoid                          LLD_MSP_EnRxMultiCH                   ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_DisRxMultiCH                  ( const LLD_MSP_IdTy id);
extern LLD_MSP_MCHSubFrameTy          LLD_MSP_GetRxMultiCHSubFrame          ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetMultiCHCompareMD           ( const LLD_MSP_IdTy id, LLD_MSP_MCHComMDTy mode);
extern LLD_MSP_MCHComMDTy             LLD_MSP_GetMultiCHCompareMD           ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EnTxMultiCH                   ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_DisTxMultiCH                  ( const LLD_MSP_IdTy id);
extern LLD_MSP_MCHSubFrameTy          LLD_MSP_GetTxMultiCHSubFrame          ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_SetRxMultiCompareReg          ( const LLD_MSP_IdTy id, tU32 mask);
extern tU32                           LLD_MSP_GetRxMultiCompareReg          ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EnRxPINMultiCHCompare         ( const LLD_MSP_IdTy id, tU8 Channel);
extern tVoid                          LLD_MSP_DisRxPINMultiCHCompare        ( const LLD_MSP_IdTy id, tU8 Channel);
extern tVoid                          LLD_MSP_SetRxMultiCompareMask         ( const LLD_MSP_IdTy id, tU32 mask);
extern tU32                           LLD_MSP_GetRxMultiCompareMask         ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EnRxPINMultiCHCompareMask     ( const LLD_MSP_IdTy id, tU8 Channel);
extern tVoid                          LLD_MSP_DisRxPINMultiCHCompareMask    ( const LLD_MSP_IdTy id, tU8 Channel);
extern tVoid                          LLD_MSP_SetTxMultiCHEnable            ( const LLD_MSP_IdTy id, tU8 bank,tU32 reg);
extern tU32                           LLD_MSP_GetTxMultiCHEnable            ( const LLD_MSP_IdTy id, tU8 bank);
extern tVoid                          LLD_MSP_EnPINTxMultiCH                ( const LLD_MSP_IdTy id, tU8 Channel);
extern tVoid                          LLD_MSP_DisPINTxMultiCH               ( const LLD_MSP_IdTy id, tU8 Channel);
extern tVoid                          LLD_MSP_SetRxMultiCHEnable            ( const LLD_MSP_IdTy id, tU8 bank,tU32 reg);
extern tU32                           LLD_MSP_GetRxMultiCHEnable            ( const LLD_MSP_IdTy id, tU8 bank);
extern tVoid                          LLD_MSP_EnPINRxMultiCH                ( const LLD_MSP_IdTy id, tU8 Channel);
extern tVoid                          LLD_MSP_DisPINRxMultiCH               ( const LLD_MSP_IdTy id, tU8 Channel);
extern tVoid                          LLD_MSP_WaitTxComplete                ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_WaitRxComplete                ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_WaitRxNotEmpty                ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_WaitTxEmpty                   ( const LLD_MSP_IdTy id);

extern tVoid                          LLD_MSP_EmptyRxFifo                   ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EmptyTxFifo                   ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_DisableTxRx                   ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_DisableTx                     ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_DisableRx                     ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EnableTxRx                    ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EnableTx                      ( const LLD_MSP_IdTy id);
extern tVoid                          LLD_MSP_EnableRx                      ( const LLD_MSP_IdTy id);

#ifdef __cplusplus
}   /* Allow C++ to use this header */
#endif  /* __cplusplus              */

#endif  /* _LLD_MSP_H_               */

// End of file

