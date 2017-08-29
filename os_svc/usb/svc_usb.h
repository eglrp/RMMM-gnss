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

#ifndef SVC_USB_H
#define SVC_USB_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"

#include "lld_usb.h"

#include "gpOS.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef struct svc_usb_com_handler_s svc_usb_com_handler_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t       svc_usb_init         ( gpOS_partition_t *partition, tU32 bus_id);

extern gpOS_error_t       svc_usb_open_vcom    ( tUInt usb_id, gpOS_interrupt_priority_t irq_pri, tU16 fifo_tx_size, tU16 fifo_rx_size, tU32 priority);
extern gpOS_error_t       svc_usb_close_vcom   ( tUInt usb_id);
extern tU32               svc_usb_read         ( tUInt usb_id, tU8 *out_buf, tU32 max_chars, gpOS_clock_t *timeout);
extern tU32               svc_usb_write        ( tUInt usb_id, tU8 *, tU32, gpOS_clock_t *);

#endif  /* __SVC_USB_H__ */

/* End of file */
