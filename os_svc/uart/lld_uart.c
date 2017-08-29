//!
//!   \file       lld_uart.c
//!   \copyright (c) STMicroelectronics
//!   \brief      <i><b>UART Low Level Driver source file</b></i>
//!   \author     L. Cotignano
//!   \version    1.0
//!   \date       2007.11.09
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup LLD
//!   \{
//!   \addtogroup UART
//!   \{
//!   \addtogroup UART_functions
//!   \{
//! \page lld_uart_page UART

#include "lld_uart.h"
#include "lld_uart_p.h"

#ifdef __cplusplus
extern "C" {
#endif

//! \brief Macro to access to UART registers
//#define UARTid(x)               ((UartMap*)x)


//----------------------------------------------------------------------
// Function implementation (scope: module-local)
//----------------------------------------------------------------------

/********************************************//**
 * \brief   Set the UART registers to zero.
 *
 * \param   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_ResetReg( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */

#if defined( LLD_UART_VER_5_1_2_0)
  pUARTMap->UARTLCRH.REG      = 0x0000;
#elif defined( LLD_UART_VER_5_1_10_0)
  pUARTMap->UARTLCRH_RX.REG   = 0x0000;
  pUARTMap->UARTLCRH_TX.REG   = 0x0000;
#endif
  pUARTMap->UARTIMSC.REG      = 0x0000;
  pUARTMap->UARTCR.REG        = 0x0000;
  pUARTMap->UARTRSR.REG       = 0x0000;
  pUARTMap->UARTIFLS.REG      = 0x0000;
  //*(tVPU32)&pUARTMap->UARTDMACR    = 0x0000;
  //*(tVPU32)&pUARTMap->UARTFCR      = 0x0000;
  //*(tVPU32)&pUARTMap->UARTXON1    = 0x0000;
  //*(tVPU32)&pUARTMap->UARTXON2   = 0x0000;
  //*(tVPU32)&pUARTMap->UARTXOFF1   = 0x0000;
  //*(tVPU32)&pUARTMap->UARTXOFF2   = 0x0000;
  LLD_UART_DisableSwFlowControl(id);
}

/********************************************//**
 * \brief   Clear UART Tx FIFO.
 *
 * \details Empty the TX FIFO and clear TX interrupts
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_TxReset( const LLD_UART_IdTy id)
{
  //! ---
  //! <B>Algorithm</B>

  tU16 enable;
  UartMap *pUARTMap = (UartMap *)id;           /*lint !e9079 !e9087 */

  enable = pUARTMap->UARTCR.BIT.UARTEN;        //!  - Read current UART enable status

  pUARTMap->UARTCR.BIT.UARTEN = SET;

  while (!pUARTMap->UARTFR.BIT.TXFE);          //!  - Wait until all bytes into TX FIFO are transmitted

  pUARTMap->UARTICR.BIT.TXIC = SET;            //!  - Clear Tx interrupts

  pUARTMap->UARTCR.BIT.UARTEN = enable;        //!  - Restore enable status
}

/********************************************//**
 * \brief   Clear UART Rx FIFO.
 *
 * \details Empty the Rx FIFO
 *          and clear RX and timeout interrupts.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_RxReset( const LLD_UART_IdTy id)
{
  //! ---
  //! <B>Algorithm</B>

  volatile tU16 temp;
  tU16 enable;
  UartMap *pUARTMap = (UartMap *)id;            /*lint !e9079 !e9087 */

  enable = pUARTMap->UARTCR.BIT.UARTEN;         //!  - Read current UART enable status

  pUARTMap->UARTCR.BIT.UARTEN = SET;

  while (!pUARTMap->UARTFR.BIT.RXFE)
  {
    temp = pUARTMap->UARTDR.BIT.DATA;            //!  - Read all data from RX FIFO to flush it
  }

  pUARTMap->UARTICR.BIT.RTIC = SET;              //!  - Clear Timeout pending interrupts
  pUARTMap->UARTICR.BIT.RXIC = SET;              //!  - Clear Rx pending interrupts

  pUARTMap->UARTCR.BIT.UARTEN = enable;          //!  - Restore UART enable status
}

/********************************************//**
 * \brief   Enable RX and TX FIFOs.
 *
 * \details Enable RX and TX FIFOs and flush them.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_FifoEnable( const LLD_UART_IdTy id)
{
  //! ---
  //! <B>Algorithm</B>

  tU16 enable;
  UartMap *pUARTMap = (UartMap *)id;      /*lint !e9079 !e9087 */

  enable = pUARTMap->UARTCR.BIT.UARTEN;   //!  - Read current UART enable status

  pUARTMap->UARTCR.BIT.UARTEN = CLEAR;    //!  - Disable temporarily UART

#if defined( LLD_UART_VER_5_1_2_0)
  pUARTMap->UARTLCRH.FEN = SET;           //!  - Enable FIFOs
#elif defined( LLD_UART_VER_5_1_10_0)
  pUARTMap->UARTLCRH_RX.BIT.FEN = SET;    //!  - Enable FIFOs RX
  pUARTMap->UARTLCRH_TX.BIT.FEN = SET;    //!  - Enable FIFOs TX
#endif

  LLD_UART_RxReset(id);                     //!  - After enabling FIFOs flush them
  LLD_UART_TxReset(id);

  pUARTMap->UARTCR.BIT.UARTEN = enable;   //!  - Restore UART enable status
}

/********************************************//**
 * \brief   Disable RX and TX FIFOs.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_FifoDisable( const LLD_UART_IdTy id)
{
  //! ---
  //! <B>Algorithm</B>

  tU16 enable;
  volatile tU8 data;
  UartMap *pUARTMap = (UartMap *)id;        /*lint !e9079 !e9087 */

  enable = pUARTMap->UARTCR.BIT.UARTEN;     //!  - Read current UART enable status

#if defined( LLD_UART_VER_5_1_2_0)
  pUARTMap->UARTLCRH.FEN = CLEAR;           //!  - Disable FIFO
#elif defined( LLD_UART_VER_5_1_10_0)
  pUARTMap->UARTLCRH_RX.BIT.FEN = CLEAR;    //!  - Disable FIFO RX
  pUARTMap->UARTLCRH_TX.BIT.FEN = CLEAR;    //!  - Disable FIFO TX
#endif

  pUARTMap->UARTCR.BIT.UARTEN = SET;        //!  - Enable UART

  data = pUARTMap->UARTDR.BIT.DATA;         //!  - After disabling FIFO (1 byte long) flush it

  while (pUARTMap->UARTFR.BIT.BUSY);        //!  - Wait until all bytes into TX FIFO are transmitted

  pUARTMap->UARTCR.BIT.UARTEN = enable;     //!  - Restore UART enable status
}

/********************************************//**
 * \brief   Set UART baud rate.
 *
 * \details Set UART baud rate according to
 *          input parameters.
 *
 * \param[in]   id UART id
 * \param[in]   sys_bus_frequency system bus frequency
 * \param[in]   baudrate Baud rate
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_SetBaudRate( const LLD_UART_IdTy id, tU32 sys_bus_frequency, LLD_UART_BaudRateTy baudrate)
{
  //! ---
  //! <B>Algorithm</B>

  tU32 BaudIntDiv, BaudFracDiv, BaudRateDiv, OvsFact;
  tU32 Rounding = ((tUInt)1 << (11-1));  /* At low frequency, we don't need this "0.5" correction */
  tU16 enable;
#if defined( LLD_UART_VER_5_1_2_0)
  tU32 UARTLCRH_temp;
#elif defined( LLD_UART_VER_5_1_10_0)
  tU32 UARTLCRH_TX_temp, UARTLCRH_RX_temp;
#endif
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */


  while (pUARTMap->UARTFR.BIT.BUSY);     //!  - Wait until all bytes into TX FIFO are transmitted

  // sys_bus_frequency below 1.8432MHz requires OVS to be set to avoid DIV register set to 0 (1 843 200 / 16 / 115200 = 1.0).
  // Here, the threshold is set to 3.5MHz to gives a better margin between sampling frequency and system frequency
  if((baudrate > 3000000U) || (sys_bus_frequency < 1843200U))
  {
    if(sys_bus_frequency < 1843200U)
    {
      Rounding = 0U;
    }

    OvsFact = SET;
    baudrate = (LLD_UART_BaudRateTy)(((tUInt)baudrate)/2);
  }
  else
  {
    OvsFact = CLEAR;
  }

  enable = pUARTMap->UARTCR.BIT.UARTEN;          //!  - Read current UART enable status

  BaudRateDiv = ((sys_bus_frequency/100) << 11) / ((baudrate/100)*16);

  BaudIntDiv = BaudRateDiv >> 11;                  //!  - Calculate integer baud rate register value

  BaudFracDiv = ((BaudRateDiv & 0x07FF) << 6) + Rounding;  //!  - Calculate fractional baud rate register value
  if((BaudFracDiv >> 11) > 0x3F)
  {
    BaudIntDiv++;
  }
  BaudFracDiv = (BaudFracDiv >> 11) & 0x3F;

#if defined( LLD_UART_VER_5_1_2_0)
  UARTLCRH_temp = *(tVPU32)&pUARTMap->UARTLCRH;  //!  - Save UARTLCRH register status
#elif defined( LLD_UART_VER_5_1_10_0)
  UARTLCRH_RX_temp = *(tVPU32)&pUARTMap->UARTLCRH_RX;  //!  - Save UARTLCRH_RX register status
  UARTLCRH_TX_temp = *(tVPU32)&pUARTMap->UARTLCRH_TX;  //!  - Save UARTLCRH_TX register status
#endif

  pUARTMap->UARTCR.BIT.UARTEN = CLEAR;          //!  - Disable UART

  pUARTMap->UARTCR.BIT.OVSFACT = OvsFact;

  pUARTMap->UARTIBRD = (tU16)BaudIntDiv;        //!  - Set integer part of baudrate

  pUARTMap->UARTFBRD = (tU8)BaudFracDiv;        //!  - Calculate fractional baud rate register value

#if defined( LLD_UART_VER_5_1_2_0)
  *(tVPU32)&pUARTMap->UARTLCRH = UARTLCRH_temp;  //!  - Write to UARTLCRH register to update UARTIBRD and UARTFBRD contents
#elif defined( LLD_UART_VER_5_1_10_0)
  *(tVPU32)&pUARTMap->UARTLCRH_RX = UARTLCRH_RX_temp;  //!  - Write to UARTLCRH_RX register to update UARTIBRD and UARTFBRD contents
  *(tVPU32)&pUARTMap->UARTLCRH_TX = UARTLCRH_TX_temp;  //!  - Write to UARTLCRH_TX register to update UARTIBRD and UARTFBRD contents
#endif

  pUARTMap->UARTCR.BIT.UARTEN = enable;          //!  - Restore UART enable status
}

/********************************************//**
 * \brief   Check if UART baud rate can be reached
//               at this bus frequency.
 *
 * \param[in]   bus_freq frequency of PCLK bus
 * \param[in]   baudrate UART baudrate
 * \return      returns TRUE if expected baudrate can be achieved
 *              else otherwise.
 *
 ***********************************************/
tBool LLD_UART_CheckBaudRateReachability(tU32 bus_freq, tU32 baudrate)
{
   if (bus_freq >= (8 * baudrate))
   {
      // If this condition is true, then BaudIntDiv register value is > 0.
      return true;
   }
   else
   {
      // BaudIntDiv register value will be set to 0 which is forbidden.
      return false;
   }
}

/********************************************//**
 * \brief   Set data length in bits.
 *
 * \param[in]   id UART id
 * \param[in]   dataLen data length (in bits)
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_SetDataLen( const LLD_UART_IdTy id, LLD_UART_DataLenTy dataLen)
{
  //! ---
  //! <B>Algorithm</B>

  tU16 enable;
  UartMap *pUARTMap = (UartMap *)id;        /*lint !e9079 !e9087 */

  enable = pUARTMap->UARTCR.BIT.UARTEN;     //!  - Read current UART enable status

  pUARTMap->UARTCR.BIT.UARTEN = SET;

  while (pUARTMap->UARTFR.BIT.BUSY);     //!  - Wait until all bytes into TX FIFO are transmitted

  pUARTMap->UARTCR.BIT.UARTEN = CLEAR;    //!  - Stop UART before set register

#if defined( LLD_UART_VER_5_1_2_0)
  pUARTMap->UARTLCRH.WLEN = dataLen;   //!  - Set word length
#elif defined( LLD_UART_VER_5_1_10_0)
  pUARTMap->UARTLCRH_RX.BIT.WLEN = dataLen;   //!  - Set word length
  pUARTMap->UARTLCRH_TX.BIT.WLEN = dataLen;   //!  - Set word length
#endif

  pUARTMap->UARTCR.BIT.UARTEN = enable;   //!  - Restore UART enable status
}

/********************************************//**
 * \brief   Return FIFO depth.
 *
 * \param[in]   iflsel Bytes trigger level
 * \return  tU8
 * \retval  Return FIFO depth according
 *          to trigger level.
 *
 ***********************************************/
static tU8 LLD_UART_ConvertFifoTriggerLevel( tU32 iflsel)
{
  //! ---
  //! <B>Algorithm</B>
  //!  <BR> Returns the FIFO depth according to trigger level

  tU8 fifo_depth = 0;

  if( iflsel == LLD_UART_1_BYTES_TRIGGER_LEVEL)
  {
    fifo_depth = 1;
  }
  else if( iflsel == LLD_UART_2_BYTES_TRIGGER_LEVEL)
  {
    fifo_depth = 2;
  }
  else if( iflsel == LLD_UART_4_BYTES_TRIGGER_LEVEL)
  {
    fifo_depth = 4;
  }
  else if( iflsel == LLD_UART_8_BYTES_TRIGGER_LEVEL)
  {
    fifo_depth = 8;
  }
  else if( iflsel == LLD_UART_16_BYTES_TRIGGER_LEVEL)
  {
    fifo_depth = 16;
  }
  else if( iflsel == LLD_UART_32_BYTES_TRIGGER_LEVEL)
  {
    fifo_depth = 32;
  }
  else if( iflsel == LLD_UART_48_BYTES_TRIGGER_LEVEL)
  {
    fifo_depth = 48;
  }

  return fifo_depth;
}

/********************************************//**
 * \brief   Get depth of RX FIFO in bytes.
 *
 * \param[in]   id UART id
 * \return  tU8
 * \retval  Return RX FIFO depth value.
 *
 ***********************************************/
tU8 LLD_UART_GetRxFifoTriggerLevel( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return LLD_UART_ConvertFifoTriggerLevel( pUARTMap->UARTIFLS.BIT.RXIFLSEL);
}

/********************************************//**
 * \brief   Get depth of TX FIFO in bytes.
 *
 * \param[in]   id UART id
 * \return  tU8
 * \retval  Return TX FIFO depth value.
 *
 ***********************************************/
tU8 LLD_UART_GetTxFifoTriggerLevel( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return LLD_UART_ConvertFifoTriggerLevel( pUARTMap->UARTIFLS.BIT.TXIFLSEL);
}

/********************************************//**
 * \brief   Set depth of RX FIFO.
 *
 * \param[in]   id UART id
 * \param[in]   fifoLevel Rx FIFO trigger level
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_SetRxFifoTriggerLevel( const LLD_UART_IdTy id, LLD_UART_FifoLevelTy fifoLevel)
{
  //! ---
  //! <B>Algorithm</B>
  UartMap *pUARTMap = (UartMap *)id;            /*lint !e9079 !e9087 */

  tU16 enable;

  enable = pUARTMap->UARTCR.BIT.UARTEN;         //!  - Read current UART enable status

  pUARTMap->UARTCR.BIT.UARTEN = SET;            //!  - Enable UART

  while (pUARTMap->UARTFR.BIT.BUSY);            //!  - Wait until all bytes into TX FIFO are transmitted

  pUARTMap->UARTCR.BIT.UARTEN = CLEAR;          //!  - Stop UART before set register

  pUARTMap->UARTIFLS.BIT.RXIFLSEL = fifoLevel;  //!  - Set Rx FIFO trigger level

  pUARTMap->UARTCR.BIT.UARTEN = enable;         //!  - Restore UART enable status
}

/********************************************//**
 * \brief   Set depth of TX FIFO.
 *
 * \param[in]   id UART id
 * \param[in]   fifoLevel TX FIFO trigger level
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_SetTxFifoTriggerLevel( const LLD_UART_IdTy id, LLD_UART_FifoLevelTy fifoLevel)
{
  //! ---
  //! <B>Algorithm</B>

  tU16 enable;
  UartMap *pUARTMap = (UartMap *)id;            /*lint !e9079 !e9087 */

  enable = pUARTMap->UARTCR.BIT.UARTEN;         //!  - Read current UART enable status

  pUARTMap->UARTCR.BIT.UARTEN = SET;            //!  - Enable UART

  while (pUARTMap->UARTFR.BIT.BUSY);            //!  - Wait until all bytes into TX FIFO are transmitted

  pUARTMap->UARTCR.BIT.UARTEN = CLEAR;          //!  - Stop UART before set register

  pUARTMap->UARTIFLS.BIT.TXIFLSEL = fifoLevel;  //!  - Set Tx FIFO trigger level

  pUARTMap->UARTCR.BIT.UARTEN = enable;         //!  - Restore UART enable status
}

/********************************************//**
 * \brief   Enable UART RX.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_RxEnable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.RXE = SET;
}

/********************************************//**
 * \brief   Disable UART RX.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_RxDisable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.RXE = CLEAR;
}

/********************************************//**
 * \brief   Enable UART TX.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_TxEnable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.TXE = SET;
}

/********************************************//**
 * \brief   Disable UART TX.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_TxDisable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.TXE = CLEAR;
}

/********************************************//**
 * \brief   Enable UART macrocell.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_Enable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.UARTEN = SET;
}

/********************************************//**
 * \brief   Disable UART macrocell.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_Disable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.UARTEN = CLEAR;
}

/********************************************//**
 * \brief   Set Parity bit on UARTLCRH registers.
 *
 * \param[in]   id UART id
 * \param[in]   parity parity bit
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_SetParity( const LLD_UART_IdTy id, LLD_UART_ParityTy parity)
{
  //! ---
  //! <B>Algorithm</B>

  tU16 enable;
  tU32 pen = 0, eps = 0, sps = 0;
  UartMap *pUARTMap = (UartMap *)id;            /*lint !e9079 !e9087 */

  enable = pUARTMap->UARTCR.BIT.UARTEN;         //!  - Read current UART enable status

  pUARTMap->UARTCR.BIT.UARTEN = SET;            //!  - Enable UART
  while (pUARTMap->UARTFR.BIT.BUSY);            //!  - Wait until all bytes into TX FIFO are transmitted
  pUARTMap->UARTCR.BIT.UARTEN = CLEAR;          //!  - Disable UART

  //!  - Set PEN, EPS and SPS bit values according to parity input parameter.
  // Nothing done in case of LLD_UART_NO_PARITY

  if (parity == LLD_UART_ODD_PARITY)
  {
    pen = 1;
    eps = 0;
    sps = 0;
  }
  else if (parity == LLD_UART_EVEN_PARITY)
  {
    pen = 1;
    eps = 1;
    sps = 0;
  }
  else if (parity == LLD_UART_SPS_ZERO_PARITY)
  {
    pen = 1;
    eps = 1;
    sps = 1;
  }
  else if (parity == LLD_UART_SPS_ONE_PARITY)
  {
    pen = 1;
    eps = 0;
    sps = 1;
  }

#if defined( LLD_UART_VER_5_1_2_0)
  pUARTMap->UARTLCRH.PEN = pen;
  pUARTMap->UARTLCRH.EPS = eps;
  pUARTMap->UARTLCRH.SPS = sps;
#elif defined( LLD_UART_VER_5_1_10_0)
  pUARTMap->UARTLCRH_RX.BIT.PEN = pen;
  pUARTMap->UARTLCRH_RX.BIT.EPS = eps;
  pUARTMap->UARTLCRH_RX.BIT.SPS = sps;

  pUARTMap->UARTLCRH_TX.BIT.PEN = pen;
  pUARTMap->UARTLCRH_TX.BIT.EPS = eps;
  pUARTMap->UARTLCRH_TX.BIT.SPS = sps;
#endif

  pUARTMap->UARTCR.BIT.UARTEN = enable;              //!  - Restore UART enable status
}


/********************************************//**
 * \brief   Set stop bits on UARTLCRH register.
 *
 * \param[in]   id UART id
 * \param[in]   stopBits stop bits (1 or 2 stop bits)
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_SetStopBits( const LLD_UART_IdTy id, LLD_UART_StopBitsTy stopBits)
{
  //! ---
  //! <B>Algorithm</B>

  tU16 enable;
  UartMap *pUARTMap = (UartMap *)id;          /*lint !e9079 !e9087 */

  enable = pUARTMap->UARTCR.BIT.UARTEN;       //!  - Read current UART enable status

  pUARTMap->UARTCR.BIT.UARTEN = SET;          //!  - Enable UART

  while (pUARTMap->UARTFR.BIT.BUSY);          //!  - Wait until all bytes into TX FIFO are transmitted

  pUARTMap->UARTCR.BIT.UARTEN = CLEAR;        //!  - Disable UART

#if defined( LLD_UART_VER_5_1_2_0)
  pUARTMap->UARTLCRH.STP2 = stopBits;         //!  - Set 1 or 2 stop bits
#elif defined( LLD_UART_VER_5_1_10_0)
  pUARTMap->UARTLCRH_RX.BIT.STP2 = stopBits;      //!  - Set 1 or 2 stop bits for Rx
  pUARTMap->UARTLCRH_TX.BIT.STP2 = stopBits;      //!  - Set 1 or 2 stop bits for Tx
#endif

  pUARTMap->UARTCR.BIT.UARTEN = enable;      //!  - Restore UART enable status
}

/********************************************//**
 * \brief   Enable selected interrupt types.
 *
 * \param[in]   id UART id
 * \param[in]   intr interrupt numbers to set
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_InterruptEnable( const LLD_UART_IdTy id, LLD_UART_IRQSrcTy intr)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  *(tVPU32)&pUARTMap->UARTIMSC |= intr;
}

/********************************************//**
 * \brief   Disable selected interrupt types.
 *
 * \param[in]   id UART id
 * \param[in]   intr interrupt numbers to clear
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_ISR_UART LLD_UART_InterruptDisable( const LLD_UART_IdTy id, LLD_UART_IRQSrcTy intr)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  *(tVPU32)&pUARTMap->UARTIMSC &= ~intr;
}

/********************************************//**
 * \brief   Get UART interrupt mask.
 *
 * \param[in]   id UART id
 * \return  LLD_UART_IRQSrcTy
 * \retval Returns the current masked status value
 *         of corresponding interrupts.
 *
 ***********************************************/
LLD_UART_IRQSrcTy LLD_UART_GetInterruptMask( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (LLD_UART_IRQSrcTy)(*(tVPU32)&pUARTMap->UARTIMSC);
}

/********************************************//**
 * \brief   Indicate if TX FIFO is empty or not.
 *
 * \details Return TRUE if TX FIFO is empty, FALSE otherwise.
 *
 * \param[in]   id UART id
 * \return  tBool
 *
 ***********************************************/
tBool LLD_UART_IsTxFifoEmpty( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (pUARTMap->UARTFR.BIT.TXFE);
}

/********************************************//**
 * \brief   Indicate if TX FIFO is full or not.
 *
 * \param[in]   id UART id
 * \return tBool
 * \retval TRUE if TX FIFO is full
 * \retval FALSE otherwise
 *
 ***********************************************/
tBool LLD_ISR_UART LLD_UART_IsTxFifoFull( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (pUARTMap->UARTFR.BIT.TXFF);
}

/********************************************//**
 * \brief   Indicate if RX FIFO is empty or not.
 *
 * \param[in]   id UART id
 * \return  tBool
 * \retval TRUE if RX FIFO is empty
 * \retval FALSE otherwise
 *
 ***********************************************/
tBool LLD_ISR_UART LLD_UART_IsRxFifoEmpty( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (pUARTMap->UARTFR.BIT.RXFE);
}

/********************************************//**
 * \brief   Indicate if RX FIFO is full or not.
 *
 * \details Return TRUE if RX FIFO is full, FALSE otherwise.
 *
 * \param[in]   id UART id
 * \return  tBool
 * \retval TRUE if RX FIFO is full
 * \retval FALSE otherwise
 *
 ***********************************************/
tBool LLD_UART_IsRxFifoFull( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (pUARTMap->UARTFR.BIT.RXFF);
}

/********************************************//**
 * \brief   Indicate if UART peripheral is busy or not.
 *
 * \param[in]   id UART id
 * \return  tBool
 * \retval TRUE when busy transmitting data
 * \retval FALSE otherwise
 *
 ***********************************************/
tBool LLD_UART_IsBusy( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (pUARTMap->UARTFR.BIT.BUSY);
}

/********************************************//**
 * \brief   Write data to Tx Hw FIFO.
 *
 * \param[in]   id UART id
 * \param[in]   data data to write on Tx FIFO
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_ISR_UART LLD_UART_WriteTxFifo( const LLD_UART_IdTy id, tU8 data)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  *(tU8*)&pUARTMap->UARTDR = data;
}

/********************************************//**
 * \brief   Read RX FIFO data.
 *
 * \param[in]   id UART id
 * \return  tU8
 * \retval Returns the received data character
 *
 ***********************************************/
tU8 LLD_ISR_UART LLD_UART_ReadRxFifo( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (tU8)pUARTMap->UARTDR.BIT.DATA;
}

/********************************************//**
 * \brief   Return interrupt status register.
 *
 * \param[in]   id UART id
 * \return  LLD_UART_IRQSrcTy
 * \retval Returns the current masked status value
 *         of corresponding interrupts.
 *
 ***********************************************/
LLD_UART_IRQSrcTy LLD_ISR_UART LLD_UART_GetInterruptStatus( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (LLD_UART_IRQSrcTy)(*(tVPU32)&pUARTMap->UARTMIS);
}

/********************************************//**
 * \brief   Return raw interrupt status register.
 *
 * \param[in]   id UART id
 * \return  LLD_UART_IRQSrcTy
 * \retval Returns the current raw status value
 *         of corresponding interrupts.
 *
 ***********************************************/
LLD_UART_IRQSrcTy LLD_UART_GetRawInterruptStatus( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (LLD_UART_IRQSrcTy)(*(tVPU32)&pUARTMap->UARTRIS);
}

/********************************************//**
 * \brief   Check if the given interrupts have been raised.
 *
 * \param[in]   id UART id
 * \param   irq_mask interrupts mask
 * \return  tBool
 * \retval Returns the current masked status value
 *         of corresponding interrupts.
 *
 ***********************************************/
tBool LLD_UART_IsInterruptRaised( const LLD_UART_IdTy id, LLD_UART_IRQSrcTy irq_mask)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  /*   if(*(tVPU32)&pUARTMap->UARTMIS & irq_mask)
  return FALSE;
  else return TRUE;*/
  return((*(tVPU32)&pUARTMap->UARTMIS) & irq_mask);
}

/********************************************//**
 * \brief   Return UART flag register.
 *
 * \param[in]   id UART id
 * \return  tU16
 * \retval Return current UART flag register status.
 *
 ***********************************************/
tU16 LLD_UART_GetFlagRegister( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (*(tVPU32)&pUARTMap->UARTFR);
}

/********************************************//**
 * \brief   Return IBRD register.
 *
 * \param[in]   id UART id
 * \return  tU16
 * \retval Return the Integer Baud Rate Register.
 *
 ***********************************************/
tU16 LLD_UART_GetIBRDRegister( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return (*(tVPU32)&pUARTMap->UARTIBRD);
}

/********************************************//**
 * \brief   Clear selected interrupts.
 *
 * \param[in]   id UART id
 * \param[in]   irq_mask interrupts mask
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_ISR_UART LLD_UART_ClearInterrupt( const LLD_UART_IdTy id, LLD_UART_IRQSrcTy irq_mask)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  *(tVPU32)&pUARTMap->UARTICR = irq_mask;
}

/********************************************//**
 * \brief   Get UART Data Register address.
 *
 * \param[in]   id UART id
 * \return  tU32
 * \retval Return the UART Data Register
 *         for a given UART id.
 *
 ***********************************************/
tU32 LLD_UART_DataRegAddress( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  return *((tU32*)(&pUARTMap->UARTDR));
}

/********************************************//**
 * \brief   Enable DMA usage by UART peripheral.
 *
 * \param[in]   id UART id
 * \param[in]   type DMA type
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_DMAEnable( const LLD_UART_IdTy id, LLD_UART_DMATy type)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  *(tVPU32)&pUARTMap->UARTDMACR |= type;
}

/********************************************//**
 * \brief   Disable DMA usage by UART peripheral.
 *
 * \param[in]   id UART id
 * \param[in]   type DMA type
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_DMADisable( const LLD_UART_IdTy id, LLD_UART_DMATy type)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  *(tVPU32)&pUARTMap->UARTDMACR &= ~type;
}

/********************************************//**
 * \brief   Init Uart peripheral.
 *
 * \param[in]   id UART id
 * \param[in]   sys_bus_frequency System bus frequency
 * \param[in]   baudrate Baud rate
 * \param[in]   stopBits Stop bits
 * \param[in]   dataLen Data length
 * \param[in]   parity Parity bit
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_Config(  LLD_UART_IdTy id,
                        tU32 sys_bus_frequency,
                        LLD_UART_BaudRateTy baudrate,
                        LLD_UART_StopBitsTy stopBits,
                        LLD_UART_DataLenTy dataLen,
                        LLD_UART_ParityTy parity
                     )
{
  //! ---
  //! <B>Algorithm</B>
  UartMap *pUARTMap = (UartMap *)id;                    /*lint !e9079 !e9087 */

  *(tVPU32)&pUARTMap->UARTIMSC    = 0x0000;              //!  - Disable all interrupts
  *(tVPU32)&pUARTMap->UARTICR     = 0x07FF;              //!  - Reset interrupt pending bits

  LLD_UART_SetBaudRate(id, sys_bus_frequency, baudrate);  //!  - Set baudrate
  LLD_UART_SetStopBits(id, stopBits);                     //!  - Set stopbits
  LLD_UART_SetDataLen(id, dataLen);                       //!  - Set data length
  LLD_UART_SetParity(id, parity);                         //!  - Set parity
}

/********************************************//**
 * \brief   Enable UART IRDA feature.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_IRDAEnable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.SIREN = TRUE;
}

/********************************************//**
 * \brief   Disable UART IRDA feature.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_IRDADisable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.SIREN = FALSE;
}

/********************************************//**
 * \brief   Clear and mask the Rx interrupt
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 * \note    Under LLD_ISR region (ITCM)
 *
 ************************************************/
tVoid LLD_UART_INT_RxEnter( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  // Mask Rx interrupt
  pUARTMap->UARTIMSC.BIT.RXIM = CLEAR;

  // Clear Rx interrupt pending bit
  pUARTMap->UARTICR.BIT.RXIC = SET;
}

/********************************************//**
 * \brief   Clear and mask the Tx interrupt
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 * \note    Under LLD_ISR region (ITCM)
 *
 ************************************************/
tVoid LLD_UART_INT_TxEnter( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  // Mask Tx interrupt
  pUARTMap->UARTIMSC.BIT.TXIM = CLEAR;

  // Clear Tx interrupt pending bit
  pUARTMap->UARTICR.BIT.TXIC = SET;
}

/********************************************//**
 * \brief   Clear and mask the Timeout interrupt
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 * \note    Under LLD_ISR region (ITCM)
 *
 ************************************************/
tVoid LLD_UART_INT_TimeoutEnter( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  // Mask Timeout interrupt
  pUARTMap->UARTIMSC.BIT.RTIM = CLEAR;

  // Clear TimeOut interrupt pending bit
  pUARTMap->UARTICR.BIT.RTIC = SET;
}

/********************************************//**
 * \brief   Enable UART HW Flow control.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_EnableHwFlowControl( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.CTSEN = SET;
  pUARTMap->UARTCR.BIT.RTSEN = SET;
}

/********************************************//**
 * \brief   Disable UART HW Flow control.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_DisableHwFlowControl( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.CTSEN = CLEAR;
  pUARTMap->UARTCR.BIT.RTSEN = CLEAR;
}

/********************************************//**
 * \brief   Disable loop-back mode.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_DisableLoopBack( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.LBE = CLEAR;
}

/********************************************//**
 * \brief   Enable loop-back mode.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_EnableLoopBack( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.LBE = SET;
}

/********************************************//**
 * \brief   Clear the UART Error/Status Register.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_Clear_ErrorStatusReg( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTRSR.REG = CLEAR;
}

/********************************************//**
 * \brief   Enable Software Flow Control.
 *
 * \param[in]   id UART id
 * \param[in]   txmode Sw Transmit Flow Control mode
 * \param[in]   rxmode Sw Receive Flow Control mode
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_EnableSwFlowControl( const LLD_UART_IdTy id, LLD_UART_SwFlowCtrlModeTy txmode, LLD_UART_SwFlowCtrlModeTy rxmode)
{
  //! ---
  //! <B>Algorithm</B>
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */

  pUARTMap->UARTXFCR.BIT.XONANY = CLEAR; //!  - Specify that incoming character must match XON programmed value to be valid XON

  pUARTMap->UARTXFCR.BIT.SPECHR = CLEAR; //!  - Disable special character detection

  pUARTMap->UARTXFCR.BIT.SFRMOD = rxmode; //!  - Configure Sw Receive Flow Control mode according to input parameter \a rxmode

  pUARTMap->UARTXFCR.BIT.SFTMOD = txmode; //!  - Configure Sw Transmit Flow Control mode according to input parameter \a txmode

  pUARTMap->UARTXFCR.BIT.SFEN = SET;      //!  - Enable Sw Flow Control
}

/********************************************//**
 * \brief   Disable Software Flow Control
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_DisableSwFlowControl( const LLD_UART_IdTy id)
{
  //! ---
  //! <B>Algorithm</B>
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */

  pUARTMap->UARTXFCR.BIT.SFRMOD = LLD_UART_SWFLOWCTRLMODE_DISABLED;   //!  - Disable Sw Receive Flow Control

  pUARTMap->UARTXFCR.BIT.SFTMOD = LLD_UART_SWFLOWCTRLMODE_DISABLED;   //!  - Disable Sw Transmit Flow Control

  pUARTMap->UARTXFCR.BIT.SFEN = CLEAR;      //!  - Disable Sw Flow Control
}

/********************************************//**
 * \brief   Modify data in Xon1, Xon2, Xoff1 or Xoff2 register
 *
 * \param[in]   id UART id
 * \param[in]   type specify whether the request concerns
 *              Xon1, Xon2, Xoff1 or Xoff2
 * \param[in]   data data to write (1 byte)
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_SetXonXoffValue( const LLD_UART_IdTy id, LLD_UART_XonXOffTy type, tU32 data)
{
  //! ---
  //! <B>Algorithm</B>

  tU32 data_mask = 0xFF;
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */

  //! - According to \a type, erase the existing Xon1/Xon2/Xoff1 or Xoff2 data value and update it with \a data.

  if (type == LLD_UART_XONXOFF_XON1)
  {
    pUARTMap->UARTXON1.REG &= ~data_mask;
    pUARTMap->UARTXON1.REG |= (data & data_mask);
  }
  else if (type == LLD_UART_XONXOFF_XON2)
  {
    pUARTMap->UARTXON2.REG &= ~data_mask;
    pUARTMap->UARTXON2.REG |= (data & data_mask);
  }
  else if (type == LLD_UART_XONXOFF_XOFF1)
  {
    pUARTMap->UARTXOFF1.REG &= ~data_mask;
    pUARTMap->UARTXOFF1.REG |= (data & data_mask);
  }
  else if (type == LLD_UART_XONXOFF_XOFF2)
  {
    pUARTMap->UARTXOFF2.REG &= ~data_mask;
    pUARTMap->UARTXOFF2.REG |= (data & data_mask);
  }
}

/********************************************//**
 * \brief   Enable Test mode.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_TestModeEnable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTTESTEN = SET;
}

/********************************************//**
 * \brief   Disable Test mode.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_TestModeDisable( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTTESTEN = CLEAR;
}

/********************************************//**
 * \brief   Set Test mode type.
 *
 * \param[in]   id UART id
 * \param[in]   value Test mode type
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_TestModeSet( const LLD_UART_IdTy id, tU32 value)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTTESTVAL = value;
}


/********************************************//**
 * \brief   Set RTS Signal to 1
 *          RTS Flow Control must have 
 *          been disabled before.
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_RTS_set( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.RTS = SET;
}

/********************************************//**
 * \brief   Set RTS Signal to 0
 *
 * \param[in]   id UART id
 * \return  tVoid
 *
 ***********************************************/
tVoid LLD_UART_RTS_clear( const LLD_UART_IdTy id)
{
  UartMap *pUARTMap = (UartMap *)id;     /*lint !e9079 !e9087 */
  pUARTMap->UARTCR.BIT.RTS = CLEAR;

}

#ifdef __cplusplus
}
#endif

//!   \}
//!   \}
//!   \}
// End of file
