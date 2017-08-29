/*****************************************************************************
   FILE:          message_handling_test1.c
   PROJECT:       STA8090 GNSS application
   SW PACKAGE:    STA8090 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Message Handling
                  (OS20+ operative system specification, Chapter 10)
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics,
------------------------------------------------------------------------------
   Developers:
      AO:   Aldo Occhipinti
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
 ------------+------+------------------------------------------------------
             |      |
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "gnss_debug.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define TX_WS_SIZE     1024
#define RX_WS_SIZE     1024
#define NUM_MSG_IN_QUEUE  1
#define NUM_MSG           9
#define BUFFER_SIZE       5
/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
typedef struct tx_manager_s
{

  gpOS_task_t *          task;
  gpOS_message_queue_t * msg_queue;

} tx_manager_t;

typedef struct rx_manager_s
{

  gpOS_task_t *          task;
  gpOS_message_queue_t * msg_queue;

} rx_manager_t;

typedef struct test_message_s
{
  unsigned int id;
  unsigned char data[ BUFFER_SIZE];
  boolean_t flag;

} test_message_t;

typedef struct messages_test1_manager_s
{
  gpOS_partition_t *       part;              /**< partition used for dynamic allocation */

  gpOS_task_t *            RxTask;            /**< RxTask pointer */
  gpOS_task_t *            TxTask;            /**< TxTask pointer */

  gpOS_message_queue_t *   msg_queue;

} messages_test1_manager_t;

static messages_test1_manager_t *messages_test1_manager = NULL;

int tx_task_priority = 9;
int rx_task_priority = 10;
boolean_t messages_test1_exit_flag = FALSE;
unsigned char payload[ NUM_MSG][BUFFER_SIZE] = { 'a', 'e', 'i', 'o', 'u',
                                                 'u', 'a', 'e', 'i', 'o',
                                                 'o', 'u', 'a', 'e', 'i',
                                                 'i', 'o', 'u', 'a', 'e',
                                                 'e', 'i', 'o', 'u', 'a',
                                                 'i', 'o', 'u', 'a', 'e',
                                                 'o', 'u', 'a', 'e', 'i',
                                                 'u', 'a', 'e', 'i', 'o',
                                                 'a', 'e', 'i', 'o', 'u'
                                               };

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static gpOS_task_exit_status_t rx_process       ( void *);
static gpOS_task_exit_status_t tx_process       ( void *);

static gpOS_task_exit_status_t tx_process( void *p)
{
  test_message_t *test_msg;
  int i;

  gpOS_task_delay( 20000000);

  while( messages_test1_exit_flag == FALSE)
  {

    for (i=0; i< NUM_MSG; i++)
    {
  test_msg = (test_message_t *)gpOS_message_claim_timeout( messages_test1_manager->msg_queue, gpOS_TIMEOUT_INFINITY);

  test_msg->flag   = TRUE;
    test_msg->id     = i;

    test_msg->data[0] = payload[i][0];
    test_msg->data[1] = payload[i][1];
    test_msg->data[2] = payload[i][2];
    test_msg->data[3] = payload[i][3];
    test_msg->data[4] = payload[i][4];

    gpOS_message_send( messages_test1_manager->msg_queue, test_msg);
    GPS_DEBUG_MSG(("[Messages] [1] [TX] [%d] [%c] [%c] [%c] [%c] [%c]\r\n", test_msg->id,
                                                                        test_msg->data[0],
                                                                        test_msg->data[1],
                                                                        test_msg->data[2],
                                                                        test_msg->data[3],
                                                                        test_msg->data[4]));

    gpOS_task_delay( 2000000);
    }

  test_msg = (test_message_t *)gpOS_message_claim_timeout( messages_test1_manager->msg_queue, gpOS_TIMEOUT_INFINITY);
  test_msg->flag   = FALSE;
  gpOS_message_send( messages_test1_manager->msg_queue, test_msg);
  messages_test1_exit_flag = TRUE;

  }

  return gpOS_SUCCESS;

}

static gpOS_task_exit_status_t rx_process( void *p)
{
  test_message_t *msg;

  while( TRUE)
  {

    msg = gpOS_message_receive( messages_test1_manager->msg_queue);

    gpOS_task_delay( 20000000);

    if ( msg->flag == TRUE)
     {
      GPS_DEBUG_MSG(("[Messages] [1] [RX] [%d] [%c] [%c] [%c] [%c] [%c]\r\n", msg->id,
                                                                                msg->data[0],
                                                                              msg->data[1],
                                                                            msg->data[2],
                                                                                msg->data[3],
                                                                            msg->data[4]));
     }


    gpOS_message_release( messages_test1_manager->msg_queue, msg);

    gpOS_task_delay( 2000000);

  }

  //return gpOS_SUCCESS;
}


gpOS_error_t message_handling_test1_run( gpOS_partition_t *part)
{

  GPS_DEBUG_MSG(("\r\n[Messages] [1] [RX/TX] [ID] [DATA0] [DATA1] [DATA2] [DATA3] [DATA4]\r\n\r\n"));

  messages_test1_manager->msg_queue = gpOS_message_create_queue_p( part, sizeof( test_message_t), NUM_MSG_IN_QUEUE);

  messages_test1_manager->RxTask = gpOS_task_create_p( part, tx_process, NULL, TX_WS_SIZE, tx_task_priority, "Message T1 P1 TX", gpOS_TASK_FLAGS_ACTIVE);
  messages_test1_manager->RxTask = gpOS_task_create_p( part, rx_process, NULL, RX_WS_SIZE, rx_task_priority, "Message T1 P2 RX", gpOS_TASK_FLAGS_ACTIVE);

  if(
      ( messages_test1_manager->RxTask    == NULL) ||
      ( messages_test1_manager->TxTask    == NULL) ||
      ( messages_test1_manager->msg_queue == NULL)
    )
      {

        gpOS_task_delete( messages_test1_manager->TxTask);
        gpOS_task_delete( messages_test1_manager->RxTask);
        gpOS_message_delete_queue( messages_test1_manager->msg_queue);

        GPS_DEBUG_MSG(("\nOS20TEST_NO_ERROR rec task_create() failed\n"));

        return gpOS_FAILURE;
      }

  return gpOS_SUCCESS;
}
/*}}}  */
