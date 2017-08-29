/*****************************************************************************
   FILE:          lld_c2c_p.h
   PROJECT:       ARM7 subsystem peripherals mapping
   SW PACKAGE:    ARM9 low level driver
------------------------------------------------------------------------------
   DESCRIPTION:   This is the header that describes locations for all hardware
                  peripheral of the ARM7 subsystem of Cartesio
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Developers:
      FB:   Fulvio Boggia
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   07.07.23  |  FB  | Original version
*****************************************************************************/

#ifndef LLD_UART_P_H
#define LLD_UART_P_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "defines.h"
#include "typedefs.h"
#include "macros.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//------------------------------------------------------------------------
// Universal asynchronous Receiver and Transmitter (UART)
//------------------------------------------------------------------------
typedef volatile struct
{
  union
  {
    struct                  // addr offset = 0x00
    {
      tU32 DATA   : 8;
      tU32 FE     : 1;
      tU32 PE     : 1;
      tU32 BE     : 1;
      tU32 OE     : 1;
      tU32 res    : 4;
    } BIT;
    tU32 REG;
  } UARTDR;

  union                     // addr offset = 0x04
  {
    struct
    {
      tU32 FE     : 1;
      tU32 PE     : 1;
      tU32 BE     : 1;
      tU32 OE     : 1;
      tU32 res    : 28;
    } BIT;
    tU32 REG;
  } UARTRSR; // and UARTECR

  gap32(1);

  tU32 UARTTIMEOUT;         // addr offset = 0x0c

  gap32(2);

  union
  {
    struct                  // addr offset = 0x18
    {
      tU32 CTS    : 1;
      tU32 DSR    : 1;
      tU32 DCD    : 1;
      tU32 BUSY   : 1;
      tU32 RXFE   : 1;
      tU32 TXFF   : 1;
      tU32 RXFF   : 1;
      tU32 TXFE   : 1;
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 RI     : 1;
      tU32 DCTS   : 1;
      tU32 DDSR   : 1;
      tU32 DDCD   : 1;
      tU32 TERI   : 1;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 res0   : 1;
      tU32 DCTS   : 1;
      tU32 res1   : 3;
#endif
      tU32 RTXDIS : 1;
      tU32 resx   : 18;
    } BIT;
    tU32 REG;
  } UARTFR;

#if defined( LLD_UART_VER_5_1_2_0)
  gap32(1);
#elif defined( LLD_UART_VER_5_1_10_0)
  union
  {
    struct                  // addr offset = 0x1C
    {
      tU32 BRK  : 1;
      tU32 PEN  : 1;
      tU32 EPS  : 1;
      tU32 STP2 : 1;
      tU32 FEN  : 1;
      tU32 WLEN : 2;
      tU32 SPS  : 1;
      tU32 res  : 24;
    } BIT;
    tU32 REG;
  } UARTLCRH_RX;
#endif

  //tU32  UARTILPR;         // addr offset = 0x20
  gap32(1);

  tU32  UARTIBRD;           // addr offset = 0x24

  tU32  UARTFBRD;           // addr offset = 0x28

  union
  {
    struct                  // addr offset = 0x2C
    {
      tU32 BRK    : 1;
      tU32 PEN    : 1;
      tU32 EPS    : 1;
      tU32 STP2   : 1;
      tU32 FEN    : 1;
      tU32 WLEN   : 2;
      tU32 SPS    : 1;
      tU32 res    : 24;
    } BIT;
    tU32 REG;
#if defined( LLD_UART_VER_5_1_2_0)
  } UARTLCRH;
#elif defined( LLD_UART_VER_5_1_10_0)
  } UARTLCRH_TX;
#endif

  union
  {
    struct                  // addr offset = 0x30
    {
      tU32 UARTEN : 1;
      tU32 SIREN  : 1;
      tU32 SIRLP  : 1;
      tU32 OVSFACT: 1;
      tU32 res0   : 3;
      tU32 LBE    : 1;
      tU32 TXE    : 1;
      tU32 RXE    : 1;
      tU32 DTR    : 1;
      tU32 RTS    : 1;
      tU32 res1   : 2;
      tU32 RTSEN  : 1;
      tU32 CTSEN  : 1;
      tU32 res2   : 16;
    } BIT;
    tU32 REG;
  } UARTCR;

  union
  {
    struct                  // addr offset = 0x34
    {
      tU32 TXIFLSEL:3;
      tU32 RXIFLSEL:3;
      tU32 reserved:26;
    } BIT;
    tU32 REG;
  }UARTIFLS;

  union
  {
    struct                  // addr offset = 0x38
    {
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 RIMIM  : 1;
      tU32 CTSMIM : 1;
      tU32 DCDMIM : 1;
      tU32 DSRMIM : 1;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 res0   : 1;
      tU32 CTSMIM : 1;
      tU32 res1   : 2;
#endif
      tU32 RXIM   : 1;
      tU32 TXIM   : 1;
      tU32 RTIM   : 1;
      tU32 FEIM   : 1;
      tU32 PEIM   : 1;
      tU32 BEIM   : 1;
      tU32 OEIM   : 1;
      tU32 XOFFIM : 1;
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 res2   : 4;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 TXFEIM : 1;
      tU32 res3   : 3;
#endif
      tU32 res4   : 16;
    } BIT;
    tU32 REG;
  }UARTIMSC;

  union
  {
    struct                  // addr offset = 0x3C
    {
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 RIRMIS   : 1;
      tU32 CTSRMIS  : 1;
      tU32 DCDRMIS  : 1;
      tU32 DSRRMIS  : 1;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 res0     : 1;
      tU32 CTSRMIS  : 1;
      tU32 res1     : 2;
#endif
      tU32 RXRIS    : 1;
      tU32 TXRIS    : 1;
      tU32 RTRIS    : 1;
      tU32 FERIS    : 1;
      tU32 PERIS    : 1;
      tU32 BERIS    : 1;
      tU32 OERIS    : 1;
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 res2     : 5;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 XOFFRIS  : 1;
      tU32 TXFERIS  : 1;
      tU32 res3     : 3;
#endif
    } BIT;
    tU32 REG;
  }UARTRIS;

  union
  {
    struct                  // addr offset = 0x40
    {
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 RIMMIS   : 1;
      tU32 CTSMMIS  : 1;
      tU32 DCDMMIS  : 1;
      tU32 DSRMMIS  : 1;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 res0     : 1;
      tU32 CTSMMIS  : 1;
      tU32 res1     : 2;
#endif
      tU32 RXMIS    : 1;
      tU32 TXMIS    : 1;
      tU32 RTMIS    : 1;
      tU32 FEMIS    : 1;
      tU32 PEMIS    : 1;
      tU32 BEMIS    : 1;
      tU32 OEMIS    : 1;
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 res2     : 5;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 XOFFMIS  : 1;
      tU32 TXFEMIS  : 1;
      tU32 res3     : 3;
#endif
    } BIT;
    tU32 REG;
  }UARTMIS;

  union
  {
    struct                  // addr offset = 0x44
    {
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 RIMIC  : 1;
      tU32 CTSMIC : 1;
      tU32 DCDMIC : 1;
      tU32 DSRMIC : 1;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 res0   : 1;
      tU32 CTSMIC : 1;
      tU32 res1   : 2;
#endif
      tU32 RXIC   : 1;
      tU32 TXIC   : 1;
      tU32 RTIC   : 1;
      tU32 FEIC   : 1;
      tU32 PEIC   : 1;
      tU32 BEIC   : 1;
      tU32 OEIC   : 1;
#if defined( LLD_UART_VER_5_1_2_0)
      tU32 res2   : 5;
#elif defined( LLD_UART_VER_5_1_10_0)
      tU32 XOFFIC : 1;
      tU32 TXFEIC : 1;
      tU32 res3   : 3;
#endif
    } BIT;
    tU32 REG;
  }UARTICR;

  union
  {
    struct                  // addr offset = 0x48
    {
      tU32 RXDMAE   : 1;
      tU32 TXDMAE   : 1;
      tU32 DMAONERR : 1;
      tU32 res      : 29;
    } BIT;
    tU32 REG;
  }UARTDMACR;

  gap32(1);               // addr offset = 0x4C

  union
  {
    struct                // addr offset = 0x50
    {
      tU32 SFEN   : 1;
      tU32 SFRMOD : 2;
      tU32 SFTMOD : 2;
      tU32 XONANY : 1;
      tU32 SPECHR : 1;
      tU32 res    : 25;
    } BIT;
    tU32 REG;
  }UARTXFCR;

  union
  {
    struct                // addr offset = 0x54
    {
      tU32 XON1   : 8;
      tU32 res    : 24;
    } BIT;
    tU32 REG;
  }UARTXON1;

  union
  {
    struct                // addr offset = 0x58
    {
      tU32 XON2   : 8;
      tU32 res    : 24;
    } BIT;
    tU32 REG;
  }UARTXON2;

  union
  {
    struct                // addr offset = 0x5C
    {
      tU32 XOFF1   : 8;
      tU32 res    : 24;
    } BIT;
    tU32 REG;
  }UARTXOFF1;

  union
  {
    struct                // addr offset = 0x60
    {
      tU32 XOFF2   : 8;
      tU32 res    : 24;
    } BIT;
    tU32 REG;
  }UARTXOFF2;

  gap32(7);               // addr offset = 0x64

  tU32 UARTTESTEN;          // addr offset = 0x80

  gap32(1);                 // addr offset = 0x84

  tU32 UARTTESTVAL;         // addr offset = 0x88

  gap32(29);                // addr offset = 0x8C

  union
  {
    struct                  // addr offset = 0x100
    {
      tU32 ACFGEN   : 1;
      tU32 RESTART  : 1;
      tU32 UPDATEN  : 1;
      tU32 res     : 29;
    } BIT;
    tU32 REG;
  }UARTABCR;

  union
  {
    struct                  // addr offset = 0x104
    {
      tU32 BYTE2S   : 3;
      tU32 BYTE1S   : 5;
      tU32 VALFMT   : 1;
      tU32 CMDERR   : 2;
      tU32 BAUDERR  : 1;
      tU32 ACDONE   : 1;
      tU32 res     : 19;
    } BIT;
    tU32 REG;
  }UARTABSR;

  union
  {
    struct                  // addr offset = 0x108
    {
      tU32 res1   : 1;
      tU32 PEN    : 1;
      tU32 EPS    : 1;
      tU32 STP2   : 1;
      tU32 FEN    : 1;
      tU32 WLEN   : 2;
      tU32 res    : 25;
    } BIT;
    tU32 REG;
  }UARTABFMT;

  gap32(17);                // addr offset = 0x10C

  union
  {
    struct                  // addr offset = 0x150
    {
      tU32 AUTOBAUDBDR  : 16;
      tU32 res          : 16;
    } BIT;
    tU32 REG;
  }UARTABDR;

  union
  {
    struct                  // addr offset = 0x154
    {
      tU32 AUTOBAUDBDFR  : 6;
      tU32 res           : 26;
    } BIT;
    tU32 REG;
  }UARTABDFR;

  union
  {
    struct                  // addr offset = 0x158
    {
      tU32 AUTOBAUDBMR   : 20;
      tU32 res           : 12;
    } BIT;
    tU32 REG;
  }UARTABMR;

  union
  {
    struct                  // addr offset = 0x15C
    {
      tU32 ABERRIM   : 1;
      tU32 ABDONEIM  : 1;
      tU32 res      : 30;
    } BIT;
    tU32 REG;
  }UARTABIMSC;

  union
  {
    struct                  // addr offset = 0x160
    {
      tU32 ABERRIS   : 1;
      tU32 ABDONERIS : 1;
      tU32 res       : 30;
    } BIT;
    tU32 REG;
  }UARTABRIS;

  union
  {
    struct                  // addr offset = 0x164
    {
      tU32 ABERRMIS  : 1;
      tU32 ABDONEMIS : 1;
      tU32 res      : 30;
    } BIT;
    tU32 REG;
  }UARTABMIS;

  union
  {
    struct                  // addr offset = 0x168
    {
      tU32 ABERRIC   : 1;
      tU32 ABDONEIC  : 1;
      tU32 res      : 30;
    } BIT;
    tU32 REG;
  }UARTABICR;

  gap32(923);               // addr offset = 0x16C

  union
  {
    struct                  // addr offset = 0xFD8
    {
      tU32 Y            : 4;
      tU32 X            : 4;
      tU32 H            : 2;
      tU32 PRODUCT_ID   : 8;
      tU32 res         : 14;
    } BIT;
    tU32 REG;
  }UARTID_PRODUCT_H_XY;

  union
  {
    struct                  // addr offset = 0xFDC
    {
      tU32 PROVIDER_ID  : 14;
      tU32 res          : 18;
    } BIT;
    tU32 REG;
  }UARTID_PROVIDER;

  union
  {
    struct                  // addr offset = 0xFE0
    {
      tU32 PartNumber0  : 8;
      tU32 reserved     : 24;
    } BIT;
    tU32 REG;
  }UARTPeriphID0;

  union
  {
    struct                  // addr offset = 0xFE4
    {
      tU32 PartNumber1  : 4;
      tU32 Designer0    : 4;
      tU32 reserved     : 8;
    } BIT;
    tU32 REG;
  }UARTPeriphID1;

  union
  {
    struct                  // addr offset = 0xFE8
    {
      tU32 Designer1    : 4;
      tU32 Revision     : 4;
      tU32 reserved     : 8;
    } BIT;
    tU32 REG;
  }UARTPeriphID2;

  union
  {
    struct                  // addr offset = 0xFEC
    {
      tU32 Configuration  : 8;
      tU32 reserved       : 8;
    } BIT;
    tU32 REG;
  }UARTPeriphID3;

  union
  {
    struct                  // addr offset = 0xFF0
    {
      tU32 UARTPCellID0 : 8;
      tU32 reserved     : 8;
    } BIT;
    tU32 REG;
  }UARTPCellID0;

  union
  {
    struct                  // addr offset = 0xFF4
    {
      tU32 UARTPCellID1 : 8;
      tU32 reserved     : 8;
    } BIT;
    tU32 REG;
  }UARTPCellID1;

  union
  {
    struct                  // addr offset = 0xFF8
    {
      tU32 UARTPCellID2 : 8;
      tU32 reserved     : 8;
    } BIT;
    tU32 REG;
  }UARTPCellID2;

  union
  {
    struct                  // addr offset = 0xFFC
    {
      tU32 UARTPCellID3 : 8;
      tU32 reserved     : 8;
    } BIT;
    tU32 REG;
  }UARTPCellID3;
}UartMap;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* LLD_UART_P_H */
