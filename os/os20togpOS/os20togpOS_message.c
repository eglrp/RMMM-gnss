/**
 * @file    os20togpOS_message.c
 * @brief   OS20 message to genericOS message wrapper.
 *
 * @addtogroup OS20 to genericOS wrapper
 */

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "os20togpOS_messagei.h"
#include "os20togpOS_memoryi.h"
#include "os20togpOS_timei.h"
#include "os20togpOS_types.h"

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
message_queue_t* message_create_queue( tSize body_size, tUInt num_of_msgs)
{
	return gpOS_message_create_queue(body_size, num_of_msgs);
}

message_queue_t* message_create_queue_p( partition_t *partition, tSize body_size, tUInt num_of_msgs)
{
	return gpOS_message_create_queue_p(partition, body_size, num_of_msgs);
}

os20_error_t message_delete_queue( message_queue_t* msg_queue)
{
	return gpOS_message_delete_queue(msg_queue);
}

void* message_claim( message_queue_t *msg_queue)
{
	return gpOS_message_claim(msg_queue);
}

void* message_claim_timeout( message_queue_t *msg_queue, const os20_clock_t *timeout)
{
  const os20_clock_t *os_timeout = timeout;

  if( os_timeout == &_ST_TimeoutImmediate)
  {
    os_timeout = gpOS_TIMEOUT_IMMEDIATE;
  }
  else if( os_timeout == &_ST_TimeoutInfinity)
  {
    os_timeout = gpOS_TIMEOUT_INFINITY;
  }

  return gpOS_message_claim_timeout(msg_queue, os_timeout);
}

void message_send( message_queue_t *msg_queue, void* message)
{
	gpOS_message_send(msg_queue, message);
}

void* message_receive( message_queue_t *msg_queue)
{
	return gpOS_message_receive(msg_queue);
}

void* message_receive_timeout( message_queue_t *msg_queue, const os20_clock_t *timeout)
{
  const os20_clock_t *os_timeout = timeout;

  if( os_timeout == &_ST_TimeoutImmediate)
  {
    os_timeout = gpOS_TIMEOUT_IMMEDIATE;
  }
  else if( os_timeout == &_ST_TimeoutInfinity)
  {
    os_timeout = gpOS_TIMEOUT_INFINITY;
  }

	return gpOS_message_receive_timeout(msg_queue, os_timeout);
}

void message_release( message_queue_t *msg_queue, void *message)
{
	gpOS_message_release(msg_queue, message);
}

