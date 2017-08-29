/**
 * @file    FRtoOS_messagei.h
 * @brief   OS wrapper messages internal definitions and macros.
 *
 * @addtogroup gpOS_WRAPPER
 */

#ifndef FRTOOS_MESSAGE_I_H
#define FRTOOS_MESSAGE_I_H

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

#define MESSAGE_IDENTIFIER_MASK_BITS 2
#define MESSAGE_IDENTIFIER_MASK (0xFFFFFFFF >> (32-MESSAGE_IDENTIFIER_MASK_BITS) )
#define REGULAR_MSG_IDENTIFIER 0x0
#define TIME_OUT_IDENTIFIER    0x1
#define UNUSED_IDENTIFIER_2    0x2
#define UNUSED_IDENTIFIER_3    0x3

/*****************************************************************************
   typedefs and structures
*****************************************************************************/

typedef struct FRtoOS_free_msg_block_s FRtoOS_free_msg_block_t;

struct FRtoOS_free_msg_block_s {
  FRtoOS_free_msg_block_t * next_free;
};

struct gpOS_message_queue_s {
  QueueHandle_t             fRtos_queue;
  SemaphoreHandle_t         cnt_sem;
  FRtoOS_free_msg_block_t * free_blocks;   /**< pointer to first free message memory */
  gpOS_partition_t *        part;  /* Partition used for allocation */
  tU32                      TimeOutMessage;
};

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif /* FRTOOS_MESSAGE_I_H */

