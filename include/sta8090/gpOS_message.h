/**
 * @file    message.h
 * @brief   OS messages definitions and macros.
 *
 * @addtogroup OS
 */

/*****************************************************************************
   includes
*****************************************************************************/

#ifndef gpOS_MESSAGE_H
#define gpOS_MESSAGE_H

#include "gpOS_types.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct gpOS_message_queue_s gpOS_message_queue_t;   /**< OS message type */

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_message_queue_t*  gpOS_message_create_queue    ( tSize msg_body_size, tUInt num_of_msgs);
extern gpOS_message_queue_t*  gpOS_message_create_queue_p  ( gpOS_partition_t *custom_part, tSize msg_body_size, tUInt num_of_msgs);
extern gpOS_error_t           gpOS_message_delete_queue    ( gpOS_message_queue_t *msg_queue);

extern void *       gpOS_message_claim            ( gpOS_message_queue_t *msg_queue);
extern void *       gpOS_message_claim_timeout    ( gpOS_message_queue_t *msg_queue, const gpOS_clock_t *timeout);
extern void         gpOS_message_send             ( gpOS_message_queue_t *msg_queue, void *message);
extern void *       gpOS_message_receive          ( gpOS_message_queue_t *msg_queue);
extern void *       gpOS_message_receive_timeout  ( gpOS_message_queue_t *msg_queue, const gpOS_clock_t *timeout);
extern void         gpOS_message_release          ( gpOS_message_queue_t *msg_queue, void *message);

#endif /* gpOS_MESSAGE_H */
