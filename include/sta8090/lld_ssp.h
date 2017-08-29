//!
//!   \file     lld_sqi_ctrl.h
//!   \brief    <i><b>RTT low level driver header file</b></i>
//!   \author   Maristella Frazzetto
///   \author   (original version) Emanuela Zaccaria
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef LLD_SSP_H
#define LLD_SSP_H

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

// SSP Interrupt Mask Set/Clear Register
// SSP Interrupt Mask Set/Clear Register (SSP_IMSC)
#define LLD_SSP_IMSC_MASK_RORIM      0x01U  // Receive Overrun Interrupt mask
#define LLD_SSP_IMSC_MASK_RTIM      0x02U  // Receive timeout Interrupt mask
#define LLD_SSP_IMSC_MASK_RXIM      0x04U  // Receive FIFO Interrupt mask
#define LLD_SSP_IMSC_MASK_TXIM      0x08U  // Transmit FIFO Interrupt mask

// SSP Raw Interrupt Status Register (SSP_RIS)
#define LLD_SSP_RIS_MASK_RORRIS      0x01U  // Receive Overrun Raw Interrupt status
#define LLD_SSP_RIS_MASK_RTRIS      0x02U  // Receive Timeout Raw Interrupt status
#define LLD_SSP_RIS_MASK_RXRIS      0x04U  // Receive FIFO Raw Interrupt status
#define LLD_SSP_RIS_MASK_TXRIS      0x08U  // Transmit FIFO Raw Interrupt status

// SSP Masked Interrupt Status Register (SSP_MIS)
#define LLD_SSP_MIS_MASK_RORMIS      0x01U  // Receive Overrun Masked Interrupt status
#define LLD_SSP_MIS_MASK_RTMIS      0x02U  // Receive Timeout Masked Interrupt status
#define LLD_SSP_MIS_MASK_RXMIS      0x04U  // Receive FIFO Masked Interrupt status
#define LLD_SSP_MIS_MASK_TXMIS      0x08U  // Transmit FIFO Masked Interrupt status

// SSP Interrupt Clear Register (SSP_ICR)
#define LLD_SSP_ICR_MASK_RORIC      0x01U  // Receive Overrun Raw Clear Interrupt bit
#define LLD_SSP_ICR_MASK_RTIC      0x02U  // Receive Timeout Clear Interrupt bit

// SSP Data Register
// Transmit or Receive Data
#define LLD_SSP_DR_MASK_DATA    0xFFFFFFFFU

// SSP Status Register
#define LLD_SSP_SR_MASK_TFE 0x01U  // Transmit FIFO empty
#define LLD_SSP_SR_MASK_TNF 0x02U  // Transmit FIFO not full
#define LLD_SSP_SR_MASK_RNE 0x04U  // Receive FIFO not empty
#define LLD_SSP_SR_MASK_RFF 0x08U  // Receive FIFO full
#define LLD_SSP_SR_MASK_BSY 0x10U  // Busy Flag

// SSP DMA Control Register
//#define LLD_SSP_DMACR_MASK_RXDMAE 0x01  // Receive DMA Enable bit
//#define LLD_SSP_DMACR_MASK_TXDMAE 0x02  // Transmit DMA Enable bit

#define MIN_CPSDVR 0x02U
#define MAX_CPSDVR 0xFEU
#define MIN_SCR 0x00U
#define MAX_SCR 0xFFU
#define LLD_SSP_ENABLE_ALL_INTERRUPT  0x0FU
#define LLD_SSP_DISABLE_ALL_INTERRUPT  0x00U
#define LLD_SSP_LAST8BITS 0x000000FFU
#define LLD_SSP_ALLZERO   0x00000000U

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef void * LLD_SSP_IdTy;

// Enum to check which interrupt is asserted
typedef enum
{
  LLD_SSP_IRQ_SRC_TRANSMIT        = 0x08U, //  Asserted when the number of elements in Tx
                                          //  FIFO is less than the programmed Watermark level
  LLD_SSP_IRQ_SRC_RECEIVE         = 0x04U, //  Asserted when the number of elements in Rx
                                          //  FIFO is more than the programmed Watermark level
  LLD_SSP_IRQ_SRC_RECEIVE_TIMEOUT = 0x02U, //  Asserted when Rx FIFO is not empty & no
                                          //  further data is received over a 32 bit period
  LLD_SSP_IRQ_SRC_RECEIVE_OVERRUN = 0x01U  //  Receive FIFO is already full & an additional
                                          //  frame is received
} LLD_SSP_IRQSrcIdTy;

typedef tU32 LLD_SSP_IRQSrcTy;

// Interface Type
typedef enum
{
  LLD_SSP_INTERFACE_MOTOROLA_SPI,         // Motorola Interface
  LLD_SSP_INTERFACE_TI_SYNC_SERIAL,       // Texas Instrument Synchronous Serial interface
  LLD_SSP_INTERFACE_NATIONAL_MICROWIRE,   // National Semiconductor Microwire interface
  LLD_SSP_INTERFACE_UNIDIRECTIONAL        // Unidirectional interface (STn8810&STn8815 only)
} LLD_SSP_InterfaceTy;

// master or slave configuration
typedef enum
{
  LLD_SSP_MASTER,               //  MuPoC SSP is master (provides the clock)
  LLD_SSP_SLAVE                 //  MuPoC SSP is slave (receives the clock)
} LLD_SSP_HierarchyTy;

// Clock parameters
typedef struct
{
  tU8 cpsdvsr;                  //  value from 2 to 254 (even only!)
  tU8 scr;                      //  value from 0 to 255
} LLD_SSP_ClockParamsTy;

// Endianness
typedef enum
{
  LLD_SSP_RX_MSB_TX_MSB,        // receive: MSBit first & transmit: MSBit first
  LLD_SSP_RX_MSB_TX_LSB,        // receive: MSBit first & transmit: LSBit first
  LLD_SSP_RX_LSB_TX_MSB,        // receive: LSBit first & transmit: MSBit first
  LLD_SSP_RX_LSB_TX_LSB         // receive: LSBit first & transmit: LSBit first
} LLD_SSP_EndianessTy;

// Number of bits in one data element
typedef enum
{
  LLD_SSP_DATA_BITS_4 = 0x03U,
  LLD_SSP_DATA_BITS_5,
  LLD_SSP_DATA_BITS_6,
  LLD_SSP_DATA_BITS_7,
  LLD_SSP_DATA_BITS_8,
  LLD_SSP_DATA_BITS_9,
  LLD_SSP_DATA_BITS_10,
  LLD_SSP_DATA_BITS_11,
  LLD_SSP_DATA_BITS_12,
  LLD_SSP_DATA_BITS_13,
  LLD_SSP_DATA_BITS_14,
  LLD_SSP_DATA_BITS_15,
  LLD_SSP_DATA_BITS_16,
  LLD_SSP_DATA_BITS_17,
  LLD_SSP_DATA_BITS_18,
  LLD_SSP_DATA_BITS_19,
  LLD_SSP_DATA_BITS_20,
  LLD_SSP_DATA_BITS_21,
  LLD_SSP_DATA_BITS_22,
  LLD_SSP_DATA_BITS_23,
  LLD_SSP_DATA_BITS_24,
  LLD_SSP_DATA_BITS_25,
  LLD_SSP_DATA_BITS_26,
  LLD_SSP_DATA_BITS_27,
  LLD_SSP_DATA_BITS_28,
  LLD_SSP_DATA_BITS_29,
  LLD_SSP_DATA_BITS_30,
  LLD_SSP_DATA_BITS_31,
  LLD_SSP_DATA_BITS_32
} LLD_SSP_DataSizeTy;

// Receive FIFO watermark level which triggers IT
typedef enum
{
  LLD_SSP_RX_1_OR_MORE_ELEM,        // IT fires when 1 or more elements in RX FIFO
  LLD_SSP_RX_4_OR_MORE_ELEM,        // IT fires when 4 or more elements in RX FIFO
  LLD_SSP_RX_8_OR_MORE_ELEM,        // IT fires when 8 or more elements in RX FIFO
  LLD_SSP_RX_16_OR_MORE_ELEM        // IT fires when 16 or more elements in RX FIFO
} LLD_SSP_RxLevelTriggerTy;

// Transmit FIFO watermark level which triggers IT
typedef enum
{
  LLD_SSP_TX_1_OR_MORE_EMPTY_LOC,      // IT fires when 1 or more empty locations in TX FIFO
  LLD_SSP_TX_4_OR_MORE_EMPTY_LOC,      // IT fires when 4 or more empty locations in TX FIFO
  LLD_SSP_TX_8_OR_MORE_EMPTY_LOC,      // IT fires when 8 or more empty locations in TX FIFO
  LLD_SSP_TX_16_OR_MORE_EMPTY_LOC      // IT fires when 16 or more empty locations in TX FIFO
} LLD_SSP_TxLevelTriggerTy;


// clock phase (Motorola SPI interface only)
typedef enum
{
  LLD_SSP_CLK_FALLING_EDGE,        // Receive data on falling edge.
  LLD_SSP_CLK_RISING_EDGE          // Receive data on rising edge.
} LLD_SSP_ClkPhaseTy;

// clock polarity (Motorola SPI interface only)
typedef enum
{
  LLD_SSP_CLK_POL_IDLE_LOW,        //  Low inactive level
  LLD_SSP_CLK_POL_IDLE_HIGH        //  High inactive level
} LLD_SSP_ClkPolarityTy;

// Command size in microwire format
typedef enum
{
  LLD_SSP_BITS_4 = 0x03U,
  LLD_SSP_BITS_5,
  LLD_SSP_BITS_6,
  LLD_SSP_BITS_7,
  LLD_SSP_BITS_8,
  LLD_SSP_BITS_9,
  LLD_SSP_BITS_10,
  LLD_SSP_BITS_11,
  LLD_SSP_BITS_12,
  LLD_SSP_BITS_13,
  LLD_SSP_BITS_14,
  LLD_SSP_BITS_15,
  LLD_SSP_BITS_16,
  LLD_SSP_BITS_17,
  LLD_SSP_BITS_18,
  LLD_SSP_BITS_19,
  LLD_SSP_BITS_20,
  LLD_SSP_BITS_21,
  LLD_SSP_BITS_22,
  LLD_SSP_BITS_23,
  LLD_SSP_BITS_24,
  LLD_SSP_BITS_25,
  LLD_SSP_BITS_26,
  LLD_SSP_BITS_27,
  LLD_SSP_BITS_28,
  LLD_SSP_BITS_29,
  LLD_SSP_BITS_30,
  LLD_SSP_BITS_31,
  LLD_SSP_BITS_32
} LLD_SSP_MicrowireCtrlTy;

// wait state
typedef enum
{
  LLD_SSP_MICROWIRE_WAIT_ZERO,      //  No wait state inserted after last command bit
  LLD_SSP_MICROWIRE_WAIT_ONE        //  One wait state inserted after last command bit
} LLD_SSP_MicrowireWaitStatelTy;

// Full/Half Duplex
typedef enum
{
  LLD_SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,   //  SSPTXD becomes bi-directional, SSPRXD not used
  LLD_SSP_MICROWIRE_CHANNEL_HALF_DUPLEX    //  SSPTXD is an output, SSPRXD is an input.
} LLD_SSP_DuplexTy;

// SSP FIFO status
typedef enum
{
  LLD_SSP_TRANSMIT_FIFO_EMPTY    = 0x01U,
  LLD_SSP_TRANSMIT_FIFO_NOT_FULL = 0x02U,
  LLD_SSP_RECEIVE_FIFO_NOT_EMPTY = 0x04U,
  LLD_SSP_RECEIVE_FIFO_FULL      = 0x08U,
  LLD_SSP_BUSY                   = 0x10U
} LLD_SSP_FifoStatusTy;

// Configuration parameters
typedef struct
{
  LLD_SSP_InterfaceTy           iface;              /**< Interface type                               */
  LLD_SSP_HierarchyTy           hierarchy;          /**< sets whether interface is master or slave    */
  tBool                         slave_tx_disable;   /**< SSPTXD is disconnected (in slave mode only)  */
  LLD_SSP_ClockParamsTy         clk_freq;           /**< Freq. of SSP interface (when master)         */
  LLD_SSP_EndianessTy           endian;             /**< sets whether MSBit or LSBit is first         */
  LLD_SSP_DataSizeTy            data_size;          /**< size of data elements (4 to 32 bits)         */
  LLD_SSP_RxLevelTriggerTy      rx_lev_trig;        /**< Rx FIFO watermark level (for IT & DMA mode)  */
  LLD_SSP_TxLevelTriggerTy      tx_lev_trig;        /**< Tx FIFO watermark level (for IT & DMA mode)  */
  LLD_SSP_ClkPhaseTy            clk_phase;          /**< Motorola SPI interface Clock phase           */
  LLD_SSP_ClkPolarityTy         clk_pol;            /**< Motorola SPI interface Clock polarity        */
  LLD_SSP_MicrowireCtrlTy       ctrl_len;           /**< Microwire interface: Control length          */
  LLD_SSP_MicrowireWaitStatelTy wait_state;         /**< Microwire interface: Wait state              */
  LLD_SSP_DuplexTy              duplex;             /**< Microwire interface: Full/Half duplex        */
  tBool                         loopback;           /**< loop back mode                               */
} LLD_SSP_ConfigTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern LLD_ErrorTy        LLD_SSP_Init                    ( const LLD_SSP_IdTy);
extern tVoid              LLD_SSP_Enable                  ( const LLD_SSP_IdTy);
extern tVoid              LLD_SSP_Disable                 ( const LLD_SSP_IdTy);
extern tVoid              LLD_SSP_SetLoopBackMode         ( const LLD_SSP_IdTy, tBool);
extern LLD_ErrorTy        LLD_SSP_Reset                   ( const LLD_SSP_IdTy);
extern LLD_ErrorTy        LLD_SSP_ResolveClockFrequency   ( tU32, tU32, tU32 *, LLD_SSP_ClockParamsTy *);
extern LLD_ErrorTy        LLD_SSP_SetConfiguration        ( const LLD_SSP_IdTy, LLD_SSP_ConfigTy *);
extern LLD_ErrorTy        LLD_SSP_SetData                 ( const LLD_SSP_IdTy, tU32);
extern LLD_ErrorTy        LLD_SSP_GetData                 ( const LLD_SSP_IdTy, tU32 *);
extern LLD_ErrorTy        LLD_SSP_GetFIFOStatus           ( const LLD_SSP_IdTy, tU32 *);
extern LLD_ErrorTy        LLD_SSP_FIFOFlush               ( const LLD_SSP_IdTy);
extern LLD_SSP_IRQSrcTy   LLD_SSP_GetIRQSrc               ( const LLD_SSP_IdTy);
extern tU32               LLD_SSP_DataRegAddress          ( const LLD_SSP_IdTy);
extern tVoid              LLD_SSP_EnableIRQSrc            ( const LLD_SSP_IdTy, LLD_SSP_IRQSrcTy);
extern tVoid              LLD_SSP_DisableIRQSrc           ( const LLD_SSP_IdTy, LLD_SSP_IRQSrcTy);
extern LLD_SSP_IRQSrcTy   LLD_SSP_GetIRQConfig            ( const LLD_SSP_IdTy id);
extern tVoid              LLD_SSP_ClearIRQSrc             ( const LLD_SSP_IdTy, LLD_SSP_IRQSrcTy);
extern tBool              LLD_SSP_IsPendingIRQSrc         ( const LLD_SSP_IdTy, LLD_SSP_IRQSrcIdTy);
extern tBool              LLD_SSP_IsMaster                ( const LLD_SSP_IdTy);
extern tBool              LLD_SSP_IsBusy                  ( const LLD_SSP_IdTy);
extern tBool              LLD_SSP_IsRxFifoFull            ( const LLD_SSP_IdTy);
extern tBool              LLD_SSP_IsRxFifoNotEmpty        ( const LLD_SSP_IdTy);
extern tBool              LLD_SSP_IsTxFifoNotFull         ( const LLD_SSP_IdTy);
extern tBool              LLD_SSP_IsTxFifoEmpty           ( const LLD_SSP_IdTy);
extern tVoid              LLD_SSP_SetDataFrameSize        ( const LLD_SSP_IdTy, LLD_SSP_DataSizeTy);
extern tVoid              LLD_SSP_SetClock                ( const LLD_SSP_IdTy id, LLD_SSP_ConfigTy *p_config);
extern tVoid              LLD_SSP_ResetClock              ( const LLD_SSP_IdTy id);

#ifdef __cplusplus
} // allow C++ to use these headers
#endif  // __cplusplus
#endif // LLD_SSP_H

// End of file - ssp.h
