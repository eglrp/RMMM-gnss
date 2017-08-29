/**
 * @file    os20togpOS_message.h
 * @brief   OS20 messages functions definitions.
 *
 * @addtogroup OS20
 */

#ifndef OS20TOGPOS_MESSAGE_H
#define OS20TOGPOS_MESSAGE_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "os20togpOS_messagei.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
#define message_create_queue(msg_body_size, num_of_msgs)                   gpOS_message_create_queue(msg_body_size, num_of_msgs)
#define message_create_queue_p(custom_part, msg_body_size, num_of_msgs)    gpOS_message_create_queue_p(custom_part, msg_body_size, num_of_msgs)

#define message_delete_queue(msg_queue)                                    gpOS_message_delete_queue(msg_queue)

#define message_claim(msg_queue)                                           gpOS_message_claim(msg_queue)
#define message_claim_timeout(msg_queue, timeout)                          gpOS_message_claim_timeout(msg_queue, timeout)
#define message_send(msg_queue, message)                                   gpOS_message_send(msg_queue, message)
#define message_receive(msg_queue)                                         gpOS_message_receive(msg_queue)
#define message_receive_timeout(msg_queue, timeout)                        gpOS_message_receive_timeout(msg_queue, timeout)
#define message_release(msg_queue, message)                                gpOS_message_release(msg_queue, message)

#endif /* OS20TOGPOS_MESSAGE_H */
