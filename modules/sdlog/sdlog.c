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

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"
#include "sdlog.h"

#include <stdio.h>
#include "lld_gpio.h"

#include "svc_pwr.h"
#include "svc_sdi.h"
#include "ff.h"
#include "platform.h"
#include "clibs.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SDLOG_TASK_WORKSPACE        1024
#define SDLOG_MESSAGE_SIZE          sizeof( sdlog_message_t)

#define SDLOG_FILEBUFFER_SIZE       (1024+512)
#define SDLOG_MINSIZETOWRITE        512U

#define SDLOG_MAX_FILENAME          13U
#define SDLOG_MAX_FILES             100U

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief sdlog file handler
 ***********************************************/
struct sdlog_file_s
{
  FIL               ff_file_hdlr;         /**< FATFS file handler */

  gpOS_semaphore_t *  access_sem;           /**< access semaphore */
  gpOS_semaphore_t *  not_full_sem;         /**< semaphore for  */
  boolean_t         not_full_waiting;     /**<  */

  tU8 *             buffer;               /**< pointer to file buffer */
  tU16              in_cnt;               /**< current input index */
  tU16              out_cnt;              /**< current output index */
  tU16              len;                  /**< current buffer len */
};

/********************************************//**
 * \brief sdlog handler
 ***********************************************/
typedef struct sdlog_handler_s
{
  gpOS_partition_t *      part;              /**< partition used for dynamic allocation */

  gpOS_task_t *           task;              /**< task pointer */
  gpOS_message_queue_t *  msg_queue;         /**< message queue for task */
  gpOS_semaphore_t *      access_sem;        /**< access semaphore */

  FATFS *               fatfs;             /**< FATFS filesystem handler */
  gpOS_wakelockid_t          wakelock_id;       /**< Power handler */
} sdlog_handler_t;

/********************************************//**
 * \brief sdlog message types
 ***********************************************/
typedef enum sdlog_msg_type_e
{
  SDLOG_MSG_WRITE
} sdlog_msg_type_t;

/********************************************//**
 * \brief sdlog write message data
 ***********************************************/
typedef struct sdlog_msg_write_s
{
  sdlog_file_t *file_hdlr;
} sdlog_msg_write_t;

/********************************************//**
 * \brief sdlog message
 ***********************************************/
typedef struct sdlog_message_s
{
  sdlog_msg_type_t type;
  union
  {
    sdlog_msg_write_t msg_write_data;
  } data;
} sdlog_message_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static sdlog_handler_t *sdlog_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static gpOS_task_exit_status_t sdlog_process       ( void *);
static void               sdlog_issue_write   ( sdlog_file_t *file_hdlr);

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Send a write message to sdlog process
 *
 * \param file_hdlr Ponter to file handler
 * \return void
 *
 ***********************************************/
static void sdlog_issue_write( sdlog_file_t *file_hdlr)
{
  sdlog_message_t *msg;

  if( sdlog_handler != NULL)
  {
    msg = gpOS_message_claim( sdlog_handler->msg_queue);

    msg->type = SDLOG_MSG_WRITE;
    msg->data.msg_write_data.file_hdlr = file_hdlr;

    gpOS_message_send( sdlog_handler->msg_queue, msg);
  }
}

/********************************************//**
 * \brief Process for file writing
 *
 * \param dummy NULL
 * \return gpOS_task_exit_status_t
 *
 ***********************************************/
static gpOS_task_exit_status_t sdlog_process( void *dummy)
{
  boolean_t exit_flag = FALSE;

  while( sdlog_handler == NULL)
  {
    gpOS_task_delay( gpOS_timer_ticks_per_sec());
  }

  while( exit_flag == FALSE)
  {
    sdlog_message_t *msg;
    sdlog_file_t *file_hdlr;

    msg = gpOS_message_receive( sdlog_handler->msg_queue);

    file_hdlr = msg->data.msg_write_data.file_hdlr;

    switch( msg->type)
    {
      case SDLOG_MSG_WRITE:
        if( file_hdlr->len > SDLOG_MINSIZETOWRITE)
        {
          tUInt written_chars;
          FRESULT write_result;

          gpOS_semaphore_wait( sdlog_handler->access_sem);

          write_result = f_write( &file_hdlr->ff_file_hdlr, &file_hdlr->buffer[file_hdlr->out_cnt],
                                  SDLOG_MINSIZETOWRITE, (UINT *)&written_chars);

          if(( write_result == FR_OK) && ( written_chars == SDLOG_MINSIZETOWRITE))
          {
            f_sync( &file_hdlr->ff_file_hdlr);

            file_hdlr->out_cnt += (tU16)SDLOG_MINSIZETOWRITE;

            gpOS_semaphore_wait( file_hdlr->access_sem);
            file_hdlr->len -= SDLOG_MINSIZETOWRITE;
            gpOS_semaphore_signal( file_hdlr->access_sem);

            if( file_hdlr->out_cnt == (tU16)SDLOG_FILEBUFFER_SIZE)
            {
              file_hdlr->out_cnt = 0;
            }

            if( file_hdlr->not_full_waiting != FALSE)
            {
              file_hdlr->not_full_waiting = FALSE;
              gpOS_semaphore_signal( file_hdlr->not_full_sem);
            }

            //LLD_GPIO_TogglePinState((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN1);
          }

          gpOS_semaphore_signal( sdlog_handler->access_sem);
        }
        break;

      default:
        break;
    }

    gpOS_message_release( sdlog_handler->msg_queue, msg);
  }

  // should never reach this
  return -1;
}

/********************************************//**
 * \brief   Free all allocated memory
 *
 * \return  void
 *
 ***********************************************/
static void sdlog_fail( void)
{
  gpOS_message_delete_queue( sdlog_handler->msg_queue);
  gpOS_semaphore_delete( sdlog_handler->access_sem);
  gpOS_task_delete( sdlog_handler->task);
  gpOS_memory_deallocate_p( sdlog_handler->part, sdlog_handler->fatfs);
  gpOS_memory_deallocate_p( sdlog_handler->part, sdlog_handler);
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Initialize sdlog module
 *
 * \param part Pointer to user partition
 * \param port SD port to initialize
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t sdlog_init( gpOS_partition_t *part, tUInt port)
{
  FIL dummy_file;
  gpOS_error_t error = gpOS_SUCCESS;

  if( sdlog_handler == NULL)
  {
    sdlog_handler = gpOS_memory_allocate_p( part, sizeof( sdlog_handler_t));

    /**< Allocate space for sdlog handler */
    if( sdlog_handler == NULL)
    {
      error = gpOS_FAILURE;
    }

    if( error != gpOS_FAILURE)
    {
      sdlog_handler->access_sem = NULL;
      sdlog_handler->msg_queue  = NULL;
      sdlog_handler->task       = NULL;

      /**< Allocate memory for FATFS structure */
      sdlog_handler->part = part;
      sdlog_handler->fatfs = gpOS_memory_allocate_p( part, sizeof( FATFS));

      if( sdlog_handler->fatfs == NULL)
      {
        sdlog_fail();
        error = gpOS_FAILURE;
      }

      if( error != gpOS_FAILURE)
      {
        /**< Init SDI service */
        svc_sdi_init( part, PLATFORM_BUSCLK_ID_MCLK);

        /**< Initialize FAT module */
        if( f_mount(0, sdlog_handler->fatfs) != FR_OK)
        {
          sdlog_fail();
          error = gpOS_FAILURE;
        }

        if( error != gpOS_FAILURE)
        {
          if( f_open( &dummy_file, "dummy.txt", (FA_WRITE | FA_CREATE_NEW)) != FR_OK)
          {
            sdlog_fail();
            error = gpOS_FAILURE;
          }

          if( error != gpOS_FAILURE)
          {
            f_close( &dummy_file);

            f_unlink( "dummy.txt");

            sdlog_handler->task = gpOS_task_create_p( part, sdlog_process, NULL, SDLOG_TASK_WORKSPACE, 8, "SDLog Task", gpOS_TASK_FLAGS_ACTIVE);
            sdlog_handler->access_sem = gpOS_semaphore_create_p( SEM_FIFO, part, 1);
            sdlog_handler->msg_queue = gpOS_message_create_queue_p( part, SDLOG_MESSAGE_SIZE, 3);

            if(
                (sdlog_handler->access_sem == NULL) ||
                (sdlog_handler->msg_queue == NULL)  ||
                (sdlog_handler->task == NULL)
            )
            {
              sdlog_fail();
              error = gpOS_FAILURE;
            }
          }
        }
      }
    }

    if( error != gpOS_FAILURE)
    {
      gpOS_wakelock_register(&sdlog_handler->wakelock_id);
    }
  }

  return error;
}

/********************************************//**
 * \brief Open a file for sdlog module
 *
 * \param folder_name Folder name
 * \param file_name File name
 * \return Pointer to sdlog file handler
 *
 ***********************************************/
sdlog_file_t *sdlog_open_file( tChar *folder_name, tChar *file_name)
{
  FRESULT fat_res;
  sdlog_file_t *file_hdlr;
  tChar final_file_name[SDLOG_MAX_FILENAME];
  tUInt cnt;

  if( sdlog_handler == NULL)
  {
    return NULL;
  }

  /**< Allocate memory for sdlog file handler */
  file_hdlr = gpOS_memory_allocate_p( sdlog_handler->part, sizeof( sdlog_file_t));

  if( file_hdlr == NULL)
  {
    return NULL;
  }

  file_hdlr->access_sem = gpOS_semaphore_create_p( SEM_FIFO, sdlog_handler->part, 0);
  file_hdlr->not_full_sem = gpOS_semaphore_create_p( SEM_FIFO, sdlog_handler->part, 0);
  file_hdlr->buffer = gpOS_memory_allocate_p( sdlog_handler->part, SDLOG_FILEBUFFER_SIZE);

  if(
      (file_hdlr->access_sem == NULL) ||
      (file_hdlr->not_full_sem == NULL) ||
      (file_hdlr->buffer == NULL)
    )
  {
    gpOS_memory_deallocate_p( sdlog_handler->part, file_hdlr->buffer);
    gpOS_semaphore_delete( file_hdlr->not_full_sem);
    gpOS_semaphore_delete( file_hdlr->access_sem);
    gpOS_memory_deallocate_p( sdlog_handler->part, file_hdlr);
    return NULL;
  }

  /**< Create folder for file and enter it */
  gpOS_semaphore_wait( sdlog_handler->access_sem);

  fat_res = f_mkdir( folder_name );

  if( (fat_res != FR_OK) && (fat_res != FR_EXIST))
  {
    gpOS_memory_deallocate_p( sdlog_handler->part, file_hdlr->buffer);
    gpOS_semaphore_delete( file_hdlr->not_full_sem);
    gpOS_semaphore_delete( file_hdlr->access_sem);
    gpOS_memory_deallocate_p( sdlog_handler->part, file_hdlr);

    gpOS_semaphore_signal( sdlog_handler->access_sem);
    return NULL;
  }

  fat_res = f_chdir( folder_name );

  /**< Look for a new file & open it if no existing file */
  for( cnt = 0; cnt < SDLOG_MAX_FILES ; cnt++ )
  {
    FILINFO FileInfo;

    sprintf( final_file_name, "%03d_%s.txt", cnt, file_name);

    // In case of standby concatened into same log file
    if( svc_pwr_StartupMode() != SVC_PWR_STARTUP_POWER_ON )
    {
      // Look for until file do not exist
      if( f_stat(final_file_name,&FileInfo) != FR_OK )
      {
        // open last file
        if( cnt == 0 )
        {
          fat_res = f_open( &file_hdlr->ff_file_hdlr, final_file_name, (FA_WRITE | FA_CREATE_NEW));
        }
        else
        {
          // load previous file found
          sprintf( final_file_name, "%03d_%s.txt", cnt - 1U, file_name);
          fat_res = f_open( &file_hdlr->ff_file_hdlr, final_file_name, FA_WRITE );

          if (fat_res == FR_OK)
          {
            fat_res = f_lseek( &file_hdlr->ff_file_hdlr, FileInfo.fsize);
          }
        }

        if (fat_res != FR_OK)
        {
          cnt = SDLOG_MAX_FILES;
          break;
        }
      }
      else
      {
        // still search last
        fat_res = FR_NO_FILE;
      }
    }
    // else avoid existing file
    else
    {
      fat_res = f_open( &file_hdlr->ff_file_hdlr, final_file_name, (FA_WRITE | FA_CREATE_NEW));
    }

    if (fat_res == FR_OK)
    {
      break;
    }
  }

  if( cnt == SDLOG_MAX_FILES)
  {
    gpOS_memory_deallocate_p( sdlog_handler->part, file_hdlr->buffer);
    gpOS_semaphore_delete( file_hdlr->not_full_sem);
    gpOS_semaphore_delete( file_hdlr->access_sem);
    gpOS_memory_deallocate_p( sdlog_handler->part, file_hdlr);

    gpOS_semaphore_signal( sdlog_handler->access_sem);
    return NULL;
  }

  file_hdlr->in_cnt           = 0;
  file_hdlr->out_cnt          = 0;
  file_hdlr->len              = 0;
  file_hdlr->not_full_waiting = FALSE;

  gpOS_semaphore_signal( sdlog_handler->access_sem);

  gpOS_semaphore_signal( file_hdlr->access_sem);

  return file_hdlr;
}

/********************************************//**
 * \brief Write a buffer into a file
 *
 * \param hdlr_ptr Pointer to file handler
 * \param buffer Pointer to buffer to transmit
 * \param len Length of buffer to transmit
 * \return Written bytes
 *
 ***********************************************/
tU32 sdlog_write( sdlog_file_t *hdlr_ptr, tChar *buffer, tU32 len)
{
  tU8 *buf_ptr;
  tU32 buf_len, in_cnt;
  tU32 written_chars = 0;

  if( hdlr_ptr == NULL)
  {
    return written_chars;
  }

  gpOS_semaphore_wait( hdlr_ptr->access_sem);

  gpOS_wakelock_acquire(sdlog_handler->wakelock_id);

  buf_ptr = &hdlr_ptr->buffer[hdlr_ptr->in_cnt];
  buf_len = hdlr_ptr->len;
  in_cnt  = hdlr_ptr->in_cnt;

  while( written_chars < len)
  {
    if( buf_len < SDLOG_FILEBUFFER_SIZE)
    {
      *buf_ptr++ = *buffer++;
      written_chars++, buf_len++, in_cnt++;

      if( in_cnt == (tU32)SDLOG_FILEBUFFER_SIZE)
      {
        in_cnt = 0;
        buf_ptr = hdlr_ptr->buffer;
      }
    }
    else
    {
      hdlr_ptr->len = buf_len;

      hdlr_ptr->not_full_waiting = TRUE;
      gpOS_semaphore_signal( hdlr_ptr->access_sem);

      sdlog_issue_write( hdlr_ptr);

      gpOS_semaphore_wait( hdlr_ptr->not_full_sem);
      gpOS_semaphore_wait( hdlr_ptr->access_sem);

      buf_len = hdlr_ptr->len;
    }
  }

  hdlr_ptr->len = buf_len;
  hdlr_ptr->in_cnt = in_cnt;

  gpOS_semaphore_signal( hdlr_ptr->access_sem);

  sdlog_issue_write( hdlr_ptr);

  gpOS_wakelock_release(sdlog_handler->wakelock_id, gpOS_TIMEOUT_INFINITY);

  return written_chars;
}
/* End of file */
