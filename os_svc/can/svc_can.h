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

#ifndef SVC_CAN_H
#define SVC_CAN_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"

#include "lld_can.h"

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

extern gpOS_error_t   svc_can_init              ( gpOS_partition_t *partition, tU32 bus_id);
extern gpOS_error_t   svc_can_open_port         ( tUInt can_id, gpOS_interrupt_priority_t irq_pri, LLD_CAN_BaudRateTy baud_rate, LLD_CAN_IdType id_type, tBool SOM, tU32 max_msg_buffered);
extern gpOS_error_t   svc_can_receive           ( tUInt can_id, tU32 *can_obj, tU32 *msg_id, tU64 *out_buf, gpOS_clock_t *timeout);
extern gpOS_error_t   svc_can_send              ( tUInt can_id, tU32 *can_obj, tU32 *msg_id, tU64 *out_buf, tU32 dlc, LLD_CAN_IdType id_type);
extern tVoid          svc_can_setup_rx_objects  ( tUInt can_id, tU32 object, tU32 message_id, tU32 mask, tBool SOM);
#endif  /* SVC_CAN_H */

/* End of file */
