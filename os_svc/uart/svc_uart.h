/*****************************************************************************
   FILE:          <filename>
   PROJECT:       <project name>
   SW PACKAGE:    <software package name>
------------------------------------------------------------------------------
   DESCRIPTION:   <...>
------------------------------------------------------------------------------
   COPYRIGHT:     (c) <year> STMicroelectronics, (<group>) <site>
------------------------------------------------------------------------------
   Developers:
      AI:   Author Initials
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   yy.mm.dd  |  AI  | Original version
*****************************************************************************/
//!
//!   \file       svc_uart.h
//!   \copyright (c) STMicroelectronics
//!   \brief      <i><b>OS Services for LLD UART source file</b></i>
//!   \author
//!   \version    1.0
//!   \date       2007.11.09

#ifndef SVC_UART_H
#define SVC_UART_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"

#include "lld_uart.h"

#include "gpOS.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   svc_uart_init                   ( gpOS_partition_t *partition, tU32 bus_id);
extern void           svc_uart_refresh                ( tU32 bus_speed);

extern gpOS_error_t   svc_uart_open_port              ( tUInt uart_id, gpOS_interrupt_priority_t irq_pri, LLD_UART_BaudRateTy baud_rate, tU16 fifo_tx_size, tU16 fifo_rx_size, tU32 priority);
extern gpOS_error_t   svc_uart_close_port             ( tUInt uart_id);
extern gpOS_error_t   svc_uart_change_baud_rate       ( const tU32 uart_id, const LLD_UART_BaudRateTy baud_rate);
extern tU32           svc_uart_read                   ( tUInt uart_id, tU8 *out_buf, tU32 max_chars, gpOS_clock_t *timeout);
extern tU32           svc_uart_write                  ( tUInt uart_id, tU8 *, tU32, gpOS_clock_t *);
extern void           svc_uart_lock                   ( tUInt uart_id);
extern void           svc_uart_release                ( tUInt uart_id);
extern boolean_t      svc_uart_is_rx_empty            ( tUInt uart_id);
extern boolean_t      svc_uart_is_tx_full             ( tUInt uart_id);
extern void           svc_uart_enable_swflowcontrol   ( tUInt uart_id, LLD_UART_SwFlowCtrlModeTy txmode, LLD_UART_SwFlowCtrlModeTy rxmode, tU32 xon1, tU32 xoff1, tU32 xon2, tU32 xoff2);
extern void           svc_uart_disable_swflowcontrol  ( tUInt uart_id);
extern boolean_t      svc_uart_is_tx_empty            ( const tUInt uart_id);
extern tVoid          svc_uart_reset_rx               ( const tUInt uart_id);
extern tVoid          svc_uart_set_RTS                (const tUInt uart_id);
extern tVoid          svc_uart_clear_RTS              (const tUInt uart_id);
extern tVoid          svc_uart_enable_HW_Flow_Control (const tUInt uart_id);
extern tVoid          svc_uart_disable_HW_Flow_Control(const tUInt uart_id);

#endif  /* SVC_UART_H */

/* End of file */
