/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Implements a SPI oves SSP protocol.
 *
 ******************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "typedefs.h"

#include "lld_sqi_ctrl.h"

#if defined( NVM_SQI_CACHED )
#include "lld_arm946.h"
#endif

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_sqi.h"
#include "string.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_SQI_HANDLER_SIZE        sizeof( svc_sqi_handler_t)
#define SVC_SQI_REGION_SIZE         sizeof( svc_sqi_region_t)

#define SVC_SQI_TASKERASE_WS        512

#define SVC_SQI_MSG_SIZE            sizeof( svc_sqi_message_t)
#define SVC_SQI_MSG_NUMBER          2

#define SVC_SQI_MIN_WRITE_TIME      (5 * gpOS_timer_ticks_per_msec())
#define SVC_SQI_LATENCY_TIME        (20 * gpOS_timer_ticks_per_usec())

#define SVC_SQI_TASKERASE_PRIORITY  3

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

struct svc_sqi_region_s;

typedef struct svc_sqi_region_s
{
  struct svc_sqi_region_s  *next;

  tUInt                     start_address;
  tUInt                     size;

  LLD_SQI_CfgTy             cfg;

  gpOS_semaphore_t            *write_access_sem;
  gpOS_semaphore_t            *erase_access_sem;
  boolean_t                 erasing;
} svc_sqi_region_t;

typedef struct svc_sqi_handler_s
{
  gpOS_partition_t *          partition;
  gpOS_task_t *               task;
  gpOS_message_queue_t *      msg_queue;

  gpOS_clock_t                time_to_suspend;

  svc_sqi_region_t *        head_region;
} svc_sqi_handler_t;

typedef enum svc_sqi_msg_type_e
{
  SVC_SQI_MSG_ERASE      = 0,
  SVC_SQI_MSG_SUB_ERASE  = 1
} svc_sqi_msg_type_t;

typedef struct svc_sqi_msg_erase_s
{
  svc_sqi_region_t *       region;
  tUInt                     block_start;
  tUInt                     block_number;
} svc_sqi_msg_erase_t;

typedef struct sdlog_message_s
{
  svc_sqi_msg_type_t type;
  union
  {
    svc_sqi_msg_erase_t msg_write_data;
  } data;
} svc_sqi_message_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static svc_sqi_handler_t *svc_sqi_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param start_addr tUInt
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
static gpOS_error_t svc_sqi_check_region_conflicts( tUInt start_addr, tUInt size)
{
  tUInt end_addr = start_addr + size - 1;
  svc_sqi_region_t *curr_region;

  /**< Check if that region overlaps another. */
  curr_region = svc_sqi_handler->head_region;

  while( curr_region)
  {
    tUInt curr_region_start  = curr_region->start_address;
    tUInt curr_region_end    = curr_region->start_address + curr_region->size - 1;

    if(
        (( start_addr >= curr_region_start) && ( start_addr <= curr_region_end))  ||
        (( end_addr   >= curr_region_start) && ( end_addr   <= curr_region_end))  ||
        (( start_addr <= curr_region_start) && ( end_addr   >= curr_region_end))
      )
    {
      return gpOS_FAILURE;
    }
    curr_region = curr_region->next;
  }
  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param start_addr tUInt
 * \param size tUInt
 * \return svc_sqi_region_t*
 *
 ***********************************************/
static svc_sqi_region_t *svc_sqi_lookup_region( tUInt start_addr, tUInt size)
{
  tUInt end_addr = start_addr + size - 1;
  svc_sqi_region_t *curr_region;

  /**< Check if that region overlaps another. */
  curr_region = svc_sqi_handler->head_region;

  while( curr_region)
  {
    tUInt curr_region_start  = curr_region->start_address;
    tUInt curr_region_end    = curr_region->start_address + curr_region->size - 1;

    if( ( start_addr >= curr_region_start) && ( end_addr <= curr_region_end))
    {
      return curr_region;
    }
    curr_region = curr_region->next;
  }
  return NULL;
}

/********************************************//**
 * \brief
 *
 * \param dummy gpOS_task_param_t
 * \return gpOS_task_exit_status_t
 *
 ***********************************************/
static FLASH_MODIFY gpOS_task_exit_status_t svc_sqi_erase_process( gpOS_task_param_t dummy)
{
  tU32 block_start;
  LLD_SQI_CfgTy *sqi_cfg;
  volatile LLD_SQI_StatusTy sqi_status;
  svc_sqi_region_t *region;

  while( 1)
  {
    svc_sqi_message_t *msg;
    tU32 idx;

    msg = gpOS_message_receive( svc_sqi_handler->msg_queue);

    region      = msg->data.msg_write_data.region;
    sqi_cfg     = &region->cfg;
    block_start = msg->data.msg_write_data.block_start;

    for( idx = 0; idx < msg->data.msg_write_data.block_number; idx++)
    {
      tU32 block_curr = block_start + idx;
      const gpOS_clock_t write_time = SVC_SQI_MIN_WRITE_TIME;

      gpOS_task_lock();

      // Start erase
      if( msg->type == SVC_SQI_MSG_ERASE)
      {
        LLD_SQI_SectorErase( block_curr);
      }
      else if( msg->type == SVC_SQI_MSG_SUB_ERASE)
      {
        LLD_SQI_SubSectorErase( block_curr);
      }
      sqi_status = LLD_SQI_GetStatus( sqi_cfg->type);

      // Wait for the end of the erase
      while( sqi_status == LLD_SQI_BUSY)
      {
        // Wait for SVC_SQI_MIN_WRITE_TIME
        svc_sqi_handler->time_to_suspend = gpOS_time_plus( gpOS_time_now(), write_time);
        while( gpOS_time_after( svc_sqi_handler->time_to_suspend, gpOS_time_now()))
        {}

        sqi_status = LLD_SQI_GetStatus( sqi_cfg->type);

        // Check if flash erase is still in progress
        if( sqi_status == LLD_SQI_BUSY)
        {
          // Suspend in progress erase
          LLD_SQI_ProgramEraseSuspend( sqi_cfg->type);

          // Wait for flash is suspended
          do
          {
            sqi_status = LLD_SQI_GetStatus( sqi_cfg->type);
          }
          while( sqi_status == LLD_SQI_BUSY);

          gpOS_task_unlock();

          gpOS_task_lock();

          // Resume in progress erase
          LLD_SQI_ProgramEraseResume( sqi_cfg->type);
          sqi_status = LLD_SQI_GetStatus( sqi_cfg->type);
        }
      }
      gpOS_task_unlock();
    }

    gpOS_message_release( svc_sqi_handler->msg_queue, msg);

    #if defined( NVM_SQI_CACHED )
    gpOS_kernel_user_system_call( (gpOS_syscall_func_t)LLD_ARM946_FlushDCache, NULL);
    #endif

    region->erasing = FALSE;

    gpOS_semaphore_signal(region->write_access_sem);
  }
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param partition gpOS_partition_t*
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_init( gpOS_partition_t *partition)
{
  if( svc_sqi_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_sqi_handler = gpOS_memory_allocate_p( partition, SVC_SQI_HANDLER_SIZE);

  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  svc_sqi_handler->msg_queue = gpOS_message_create_queue_p( partition, SVC_SQI_MSG_SIZE, SVC_SQI_MSG_NUMBER);
  svc_sqi_handler->task = gpOS_task_create_p( partition, svc_sqi_erase_process, NULL, SVC_SQI_TASKERASE_WS, SVC_SQI_TASKERASE_PRIORITY, "SVC_SQI_SRVE", gpOS_TASK_FLAGS_ACTIVE);

  if( (svc_sqi_handler->msg_queue == NULL) || (svc_sqi_handler->task == NULL))
  {
    gpOS_task_delete( svc_sqi_handler->task);
    gpOS_message_delete_queue( svc_sqi_handler->msg_queue);
  }

  svc_sqi_handler->partition    = partition;
  svc_sqi_handler->head_region  = NULL;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param start_addr tUInt
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_create_region( tUInt start_addr, tUInt size, tU8 spm_conf)
{
  svc_sqi_region_t *curr_region;

  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Check if that region overlaps another. */
  if( svc_sqi_check_region_conflicts( start_addr, size) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;
  }

  /**< Allocate needed memory */
  curr_region = gpOS_memory_allocate_p( svc_sqi_handler->partition, SVC_SQI_REGION_SIZE);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  curr_region->write_access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_sqi_handler->partition, 0);
  curr_region->erase_access_sem  = gpOS_semaphore_create_p( SEM_FIFO, svc_sqi_handler->partition, 0);

  if( (curr_region->write_access_sem == NULL) || (curr_region->erase_access_sem == NULL))
  {
    gpOS_semaphore_delete( curr_region->write_access_sem);
    gpOS_semaphore_delete( curr_region->erase_access_sem);
    gpOS_memory_deallocate_p( svc_sqi_handler->partition, curr_region);
  }

  /**< Configure NOR config structure */
  LLD_SQI_Configure( &curr_region->cfg);
  LLD_SQI_InitStatusRegisters( curr_region->cfg.type, spm_conf);
  LLD_SQI_EnableQuadIO( curr_region->cfg.type);

  /**< Update region parameters */
  curr_region->start_address    = start_addr;
  curr_region->size             = size;
  curr_region->next             = NULL;
  curr_region->erasing          = FALSE;

  /**< Enqueue new region in handler */
  if( svc_sqi_handler->head_region == NULL)
  {
    svc_sqi_handler->head_region = curr_region;
  }
  else
  {
    svc_sqi_region_t *next_region;

    next_region = svc_sqi_handler->head_region;

    while( next_region->next != NULL)
    {
      next_region = next_region->next;
    }

    next_region->next = curr_region;
  }

  /**< Free access to region */
  gpOS_semaphore_signal( curr_region->write_access_sem);
  gpOS_semaphore_signal( curr_region->erase_access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param addr void*
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
FLASH_MODIFY gpOS_error_t svc_sqi_reset_region( void *start_addr)
{
  LLD_SQI_CfgTy *sqi_cfg;
  volatile LLD_SQI_StatusTy sqi_status;
  svc_sqi_region_t *curr_region;

  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Look for address in regions */
  curr_region = svc_sqi_lookup_region( (tUInt)start_addr, LLD_SQI_SECTOR_SIZE);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  gpOS_semaphore_wait( curr_region->write_access_sem);

  sqi_cfg = &curr_region->cfg;

  gpOS_interrupt_lock();

  sqi_status = LLD_SQI_GetStatus( sqi_cfg->type);

  if( sqi_status == LLD_SQI_BUSY)
  {
    while( sqi_status == LLD_SQI_BUSY)
    {
      sqi_status = LLD_SQI_GetStatus( sqi_cfg->type);
    }
  }
  else if (( sqi_status == LLD_SQI_SUSPENDED) || ( sqi_status == LLD_SQI_ERROR))
  {
    LLD_SQI_ProgramEraseResume( sqi_cfg->type);

    do
    {
      sqi_status = LLD_SQI_GetStatus( sqi_cfg->type);
    }
    while( sqi_status == LLD_SQI_BUSY);
  }

  gpOS_interrupt_unlock();

  curr_region->erasing = FALSE;

  gpOS_semaphore_signal( curr_region->write_access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param dest_addr void*
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_unlock( void *dest_addr, tUInt size)
{
  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param dest_addr void*
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_lock( void *dest_addr, tUInt size)
{
  return gpOS_FAILURE;
}

/********************************************//**
 * \brief
 *
 * \param spm tU8
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_set_sw_protection( tU8 spm)
{
  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  if( svc_sqi_handler->head_region == NULL)
  {
    return gpOS_FAILURE;
  }

  gpOS_semaphore_wait( svc_sqi_handler->head_region->write_access_sem);
  gpOS_semaphore_wait( svc_sqi_handler->head_region->erase_access_sem);

  gpOS_task_lock();

  LLD_SQI_SetSwProtectionMode( svc_sqi_handler->head_region->cfg.type, spm);

  gpOS_task_unlock();

  gpOS_semaphore_signal( svc_sqi_handler->head_region->write_access_sem);
  gpOS_semaphore_signal( svc_sqi_handler->head_region->erase_access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param spm tU8*
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_get_sw_protection( tU8 *spm)
{
  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  if( svc_sqi_handler->head_region == NULL)
  {
    return gpOS_FAILURE;
  }

  *spm = LLD_SQI_GetSwProtectionMode( svc_sqi_handler->head_region->cfg.type);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param dest_addr void*
 * \param src_addr void*
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_read( void *dest_addr, void *src_addr, tUInt size)
{
  return gpOS_FAILURE;
}

/********************************************//**
 * \brief
 *
 * \param dest_addr void*
 * \param src_addr void*
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_write( void *dest_ptr, void *src_ptr, tUInt size)
{
  svc_sqi_region_t *curr_region;
  gpOS_error_t os_error = gpOS_SUCCESS;
  tU32 src_addr = (tU32)src_ptr;
  tU32 dest_addr = (tU32)dest_ptr;
  tU32 pos, sqi_step_size;

  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Look for address in regions */
  curr_region = svc_sqi_lookup_region( (tUInt)dest_addr, size);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  sqi_step_size = curr_region->cfg.step_size;

  gpOS_semaphore_wait( curr_region->write_access_sem);

  for( pos = 0; pos < size; pos += sqi_step_size )
  {
    /**< Start program phase */
    gpOS_interrupt_lock();

    if( LLD_SQI_Program( &curr_region->cfg, dest_addr, (tVoid *)src_addr) == LLD_ERROR)
    {
      gpOS_interrupt_unlock();
      os_error = gpOS_FAILURE;
      break;
    }
    else
    {
      #if defined( NVM_SQI_CACHED )
      gpOS_kernel_user_system_call( (gpOS_syscall_func_t)LLD_ARM946_FlushDCacheAddr, (gpOS_syscall_param_t)dest_addr);
//      if( sqi_step_size == LLD_SQI_SIZE_BYTE)
//      {
//        *((tVPU8)dest_addr) = *((tVPU8)src_addr);
//      }
//      else if( sqi_step_size == LLD_SQI_SIZE_HALF)
//      {
//        *((tVPU16)dest_addr) = *((tVPU16)src_addr);
//      }
//      else if( sqi_step_size == LLD_SQI_SIZE_WORD)
//      {
//        *((tVPU32)dest_addr) = *((tVPU32)src_addr);
//      }
      #endif
      gpOS_interrupt_unlock();
    }

    dest_addr += sqi_step_size;
    src_addr += sqi_step_size;
  }
  gpOS_semaphore_signal( curr_region->write_access_sem);

  return( os_error);
}

/********************************************//**
 * \brief
 *
 * \param dest_addr void*
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_erase( void *dest_addr, tUInt size, boolean_t wait_for_completion)
{
  svc_sqi_region_t *curr_region;
  tU32 block_start, block_num;
  tU32 curr_size;
  svc_sqi_message_t *msg;

  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  if( size == 0)
  {
    return gpOS_SUCCESS;
  }

  /**< Look for address in regions */
  curr_region = svc_sqi_lookup_region( (tUInt)dest_addr, size);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Evaluate start block and number of blocks to erase */
  LLD_SQI_AddrToSectorNum( (tU32)dest_addr, &block_start);

  block_num = 0;

  for( curr_size = 0; curr_size < size; )
  {
    curr_size += LLD_SQI_SECTOR_SIZE;
    block_num++;
  }

  gpOS_semaphore_wait( curr_region->write_access_sem);

  gpOS_semaphore_wait( curr_region->erase_access_sem);

  curr_region->erasing = TRUE;

  msg = (svc_sqi_message_t *)gpOS_message_claim( svc_sqi_handler->msg_queue);

  msg->type = SVC_SQI_MSG_ERASE;
  msg->data.msg_write_data.region       = curr_region;
  msg->data.msg_write_data.block_start  = block_start;
  msg->data.msg_write_data.block_number = block_num;

  gpOS_message_send( svc_sqi_handler->msg_queue, msg);

  if( wait_for_completion == TRUE)
  {
    // Wait for erase completion
    while( curr_region->erasing == TRUE)
    {
      gpOS_task_delay( 10 * gpOS_timer_ticks_per_msec());
    }
  }

  gpOS_semaphore_signal( curr_region->erase_access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param dest_addr void*
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_subsector_erase( void *dest_addr, tUInt size, boolean_t wait_for_completion)
{
  svc_sqi_region_t *curr_region;
  tU32 block_start, block_num;
  tU32 curr_size;
  svc_sqi_message_t *msg;

  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  if( size == 0)
  {
    return gpOS_SUCCESS;
  }

  /**< Look for address in regions */
  curr_region = svc_sqi_lookup_region( (tUInt)dest_addr, size);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Evaluate start block and number of blocks to erase */
  LLD_SQI_AddrToSubSectorNum( (tU32)dest_addr, &block_start);

  block_num = 0;

  for( curr_size = 0; curr_size < size; )
  {
    curr_size += LLD_SQI_SUBSECTOR_SIZE;
    block_num++;
  }

  gpOS_semaphore_wait( curr_region->write_access_sem);

  gpOS_semaphore_wait( curr_region->erase_access_sem);

  curr_region->erasing = TRUE;

  msg = (svc_sqi_message_t *)gpOS_message_claim( svc_sqi_handler->msg_queue);

  msg->type = SVC_SQI_MSG_SUB_ERASE;
  msg->data.msg_write_data.region       = curr_region;
  msg->data.msg_write_data.block_start  = block_start;
  msg->data.msg_write_data.block_number = block_num;

  gpOS_message_send( svc_sqi_handler->msg_queue, msg);

  if( wait_for_completion == TRUE)
  {
    // Wait for erase completion
    while( curr_region->erasing == TRUE)
    {
      gpOS_task_delay( 10 * gpOS_timer_ticks_per_msec());
    }
  }

  gpOS_semaphore_signal( curr_region->erase_access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param void
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_sqi_reset( void)
{
  if( svc_sqi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  LLD_SQI_Reset( svc_sqi_handler->head_region->cfg.type);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param void
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t FLASH_MODIFY svc_sqi_power_down( void)
{
  gpOS_error_t error = gpOS_SUCCESS;
  LLD_SQI_CfgTy *sqi_cfg;

  if( svc_sqi_handler == NULL)
  {
    error = gpOS_FAILURE;
  }
  else
  {
    sqi_cfg = &svc_sqi_handler->head_region->cfg;

    // Suspend in progress erase
    LLD_SQI_ProgramEraseSuspend( sqi_cfg->type);
  }

  return error;
}
