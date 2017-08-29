/**
 * @file    OStoFR_message.h
 * @brief   OS message definitions and macros.
 *
 * @addtogroup gpOS_WRAPPER
 */

#ifndef OSTOFR_MESSAGE_H
#define OSTOFR_MESSAGE_H

// Fallback for generic OS
#ifndef gpOS_MESSAGE_H
#define gpOS_MESSAGE_H
#endif

/*****************************************************************************
   includes
*****************************************************************************/

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "FR_memory.h"

/*****************************************************************************
   defines and macros
*****************************************************************************/

#define gpOS_message_create_queue( msg_body_size, num_of_msgs) \
        fis_message_create_queue_p(__FILE__, __LINE__, msg_body_size, num_of_msgs)
#define gpOS_message_create_queue_p(custom_part, msg_body_size, num_of_msgs) \
        fis_message_create_queue_p(__FILE__, __LINE__, custom_part, msg_body_size, num_of_msgs)

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

extern gpOS_message_queue_t*  fis_message_create_queue    ( tChar *file, tUInt line, tSize msg_body_size, tUInt num_of_msgs);
extern gpOS_message_queue_t*  fis_message_create_queue_p  ( tChar *file, tUInt line, gpOS_partition_t *custom_part, tSize msg_body_size, tUInt num_of_msgs);
extern gpOS_error_t           gpOS_message_delete_queue   ( gpOS_message_queue_t *msg_queue);

extern void *   gpOS_message_claim                ( gpOS_message_queue_t *msg_queue);
extern void *   gpOS_message_claim_timeout        ( gpOS_message_queue_t *msg_queue, const gpOS_clock_t *timeout);

extern void     gpOS_message_send                 ( gpOS_message_queue_t *msg_queue, void *message);
extern void *   gpOS_message_receive              ( gpOS_message_queue_t *msg_queue);
extern void *   gpOS_message_receive_timeout      ( gpOS_message_queue_t *msg_queue, const gpOS_clock_t *timeout);
extern void *   gpOS_message_receive_HiResTimeout ( gpOS_message_queue_t *msg_queue, const gpOS_clock_t *timeout);

extern void     gpOS_message_release              ( gpOS_message_queue_t *msg_queue, void *message);

#endif /* OSTOFR_MESSAGE_H */
