/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Implements NOR basic APIs through usage of FSMC driver.
 *
 ******************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "typedefs.h"

#include "lld_fsmc_ctrl.h"
#include "lld_fsmc_nor.h"

#ifdef LLD_TRC_FSMC_NOR
#include "lld_trace.h"
#endif

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_fsmc.h"
#include "string.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_FSMC_NOR_NULLBLOCKID     0xFFFFFFFFU

#define SVC_FSMC_HANDLER_SIZE        sizeof( svc_fsmc_nor_handler_t)
#define SVC_FSMC_REGION_SIZE         sizeof( svc_fsmc_nor_region_t)

#define SVC_FSMC_TASKERASE_WS        512

#define SVC_FSMC_MSG_SIZE            sizeof( svc_fsmc_message_t)
#define SVC_FSMC_MSG_NUMBER          2

#define SVC_FSMC_MIN_WRITE_TIME      (5 * gpOS_timer_ticks_per_msec())

#ifdef LLD_TRC_FSMC_NOR
#define SVC_FSMC_TRC_Add(x)          { LLD_TRC_Start(); LLD_TRC_Add( x); LLD_TRC_Stop(); }
#else
#define SVC_FSMC_TRC_Add(x)
#endif

#define SVC_FSMC_TASKERASE_PRIORITY  3

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

struct svc_fsmc_nor_region_s;

typedef struct svc_fsmc_nor_region_status_s
{
  LLD_FSMC_NorStatusTy    last_status;
  tU32                    address;
  tU32                    block;
  tU32                    data;
} svc_fsmc_nor_region_status_t;

typedef struct svc_fsmc_nor_region_s
{
  struct svc_fsmc_nor_region_s * next;

  tUInt                           start_address;
  tUInt                           size;

  LLD_FSMC_NorCfgTy               cfg;

  gpOS_semaphore_t *                   write_access_sem;
  gpOS_semaphore_t *                   erase_ended_sem;

  svc_fsmc_nor_region_status_t   prog_status;
  svc_fsmc_nor_region_status_t   erase_status;
} svc_fsmc_nor_region_t;

typedef struct svc_fsmc_nor_handler_s
{
  gpOS_partition_t *           partition;
  gpOS_task_t *                task;
  gpOS_message_queue_t *       msg_queue;

  svc_fsmc_nor_region_t *head_region;
} svc_fsmc_nor_handler_t;

typedef enum svc_fsmc_msg_type_e
{
  SVC_FSMC_MSG_ERASE
} svc_fsmc_msg_type_t;

typedef struct svc_fsmc_msg_erase_s
{
  svc_fsmc_nor_region_t *  region;
  tUInt                     block_start;
  tUInt                     block_number;
  boolean_t                 waiting_erase_end;
} svc_fsmc_msg_erase_t;

typedef struct sdlog_message_s
{
  svc_fsmc_msg_type_t type;
  union
  {
    svc_fsmc_msg_erase_t msg_erase_data;
  } data;
} svc_fsmc_message_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

svc_fsmc_nor_handler_t *svc_fsmc_nor_handler = NULL;

struct {
  gpOS_clock_t max_prog_delay;
  gpOS_clock_t max_erase_delay;
  gpOS_clock_t max_resume_delay;
  gpOS_clock_t max_suspend_delay;
} svc_fsmc_ilock_delays;

tU32 svc_fsmc_nor_status[10];

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static gpOS_error_t               svc_fsmc_check_region_conflicts  ( tUInt start_addr, tUInt size);
static svc_fsmc_nor_region_t *    svc_fsmc_lookup_region           ( tUInt start_addr, tUInt size);
static gpOS_error_t               svc_fsmc_nor_write_single        ( svc_fsmc_nor_region_t *curr_region, tU32 dest_addr, tVoid *src_addr);
static gpOS_task_exit_status_t    svc_fsmc_erase_process           ( gpOS_task_param_t dummy);

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
static gpOS_error_t svc_fsmc_check_region_conflicts( tUInt start_addr, tUInt size)
{
  tUInt end_addr = start_addr + size - 1;
  svc_fsmc_nor_region_t *curr_region;

  /**< Check if that region overlaps another. */
  curr_region = svc_fsmc_nor_handler->head_region;

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
 * \return svc_fsmc_nor_region_t*
 *
 ***********************************************/
static svc_fsmc_nor_region_t *svc_fsmc_lookup_region( tUInt start_addr, tUInt size)
{
  tUInt end_addr = start_addr + size - 1;
  svc_fsmc_nor_region_t *curr_region;

  /**< Check if that region overlaps another. */
  curr_region = svc_fsmc_nor_handler->head_region;

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
 * \param curr_flash_nor_cfg LLD_FSMC_NorCfgTy*
 * \param dest_addr tU32
 * \param src_addr tVoid*
 * \return gpOS_error_t
 *
 ***********************************************/
static FLASH_MODIFY gpOS_error_t svc_fsmc_nor_write_single( svc_fsmc_nor_region_t *curr_region, tU32 dest_addr, tVoid *src_addr)
{
  gpOS_error_t error = gpOS_SUCCESS;
  LLD_FSMC_NorStatusTy nor_status;
  tU32 block_num, block_pos;\
  LLD_FSMC_NorCfgTy *nor_cfg = &curr_region->cfg;

  LLD_FSMC_NorAddrToBlockPos( nor_cfg, dest_addr, &block_num, &block_pos);

  if( curr_region->erase_status.block != block_num)
  {
    gpOS_interrupt_lock();

    if( LLD_FSMC_NorProgram( nor_cfg, dest_addr, src_addr) == LLD_ERROR)
    {
      error = gpOS_FAILURE;
    }
    else
    {
      do
      {
        nor_status = LLD_FSMC_NorGetAddrStatus( nor_cfg, (void *)dest_addr, LLD_FSMC_NORSTATUS_PROGRAMMING);
      } while( (nor_status != LLD_FSMC_NORSTATUS_READ) && (nor_status != LLD_FSMC_NORSTATUS_ERROR));

      LLD_FSMC_NorResetAddrStatus( nor_cfg, (void *)dest_addr);
    }

    gpOS_interrupt_unlock();
  }

  return error;
}

/********************************************//**
 * \brief
 *
 * \param dummy gpOS_task_param_t
 * \return gpOS_task_exit_status_t
 *
 ***********************************************/
static FLASH_MODIFY gpOS_task_exit_status_t svc_fsmc_erase_process( gpOS_task_param_t dummy)
{
  boolean_t exit_flag = FALSE;
  tU32 block_start;
  LLD_FSMC_NorCfgTy *nor_cfg;
  svc_fsmc_nor_region_t *region;

  while( exit_flag == FALSE)
  {
    svc_fsmc_message_t *msg;
    tU32 idx;
    LLD_ErrorTy lld_error = LLD_NO_ERROR;

    msg = gpOS_message_receive( svc_fsmc_nor_handler->msg_queue);

    region      = msg->data.msg_erase_data.region;
    nor_cfg     = &region->cfg;
    block_start = msg->data.msg_erase_data.block_start;

    for( idx = 0; idx < (msg->data.msg_erase_data.block_number) && (lld_error == LLD_NO_ERROR); idx++)
    {
      boolean_t erase_end = FALSE;
      tU32 block_curr = block_start + idx;
      LLD_FSMC_NorStatusTy nor_status;

      gpOS_task_lock();

      //gpOS_semaphore_wait( region->write_access_sem);

      do
      {
        // Start erase
        gpOS_interrupt_lock();
        lld_error = LLD_FSMC_NorEraseBlock( nor_cfg, block_curr);
        nor_status = LLD_FSMC_NORSTATUS_BUSY;
        gpOS_interrupt_unlock();

        if( lld_error == LLD_NO_ERROR)
        {
          // Check that erase started
          do
          {
            gpOS_interrupt_lock();
            nor_status = LLD_FSMC_NorGetBlockStatus( nor_cfg, block_curr, LLD_FSMC_NORSTATUS_ERASING);
            gpOS_interrupt_unlock();
          } while( nor_status == LLD_FSMC_NORSTATUS_BUSY);
        }
      } while( (lld_error == LLD_NO_ERROR) && (nor_status == LLD_FSMC_NORSTATUS_READ));

      if( lld_error == LLD_NO_ERROR)
      {
        // Update erasing status
        region->erase_status.last_status  = LLD_FSMC_NORSTATUS_ERASING;
        region->erase_status.block        = block_curr;

        while( erase_end == FALSE)
        {
          gpOS_clock_t time_to_suspend;
          // wait minimum write time
          time_to_suspend = gpOS_time_plus( gpOS_time_now(), SVC_FSMC_MIN_WRITE_TIME);
          while( gpOS_time_after( time_to_suspend, gpOS_time_now()));

          gpOS_interrupt_lock();
          nor_status = LLD_FSMC_NorGetBlockStatus( nor_cfg, block_curr, LLD_FSMC_NORSTATUS_ERASING);
          gpOS_interrupt_unlock();

          if( nor_status == LLD_FSMC_NORSTATUS_ERASING)
          {
            gpOS_interrupt_lock();
            LLD_FSMC_NorEraseSuspend( nor_cfg, block_curr);
            gpOS_interrupt_unlock();

            do
            {
              gpOS_interrupt_lock();
              nor_status = LLD_FSMC_NorGetBlockStatus( nor_cfg, block_curr, LLD_FSMC_NORSTATUS_ERASING);
              gpOS_interrupt_unlock();
            } while( (nor_status != LLD_FSMC_NORSTATUS_SUSPENDED) && (nor_status != LLD_FSMC_NORSTATUS_READ));

            region->erase_status.last_status  = LLD_FSMC_NORSTATUS_SUSPENDED;
            gpOS_task_unlock();

            //gpOS_semaphore_signal( region->write_access_sem);

            /**< Schedule all pending tasks */

            //gpOS_semaphore_wait( region->write_access_sem);

            gpOS_task_lock();

            gpOS_interrupt_lock();
            LLD_FSMC_NorEraseResume( nor_cfg, block_curr);
            gpOS_interrupt_unlock();

            do
            {
              gpOS_interrupt_lock();
              nor_status = LLD_FSMC_NorGetBlockStatus( nor_cfg, block_curr, LLD_FSMC_NORSTATUS_ERASING);
              gpOS_interrupt_unlock();
            } while( (nor_status != LLD_FSMC_NORSTATUS_ERASING) && (nor_status != LLD_FSMC_NORSTATUS_READ));

            region->erase_status.last_status = nor_status;

            if( nor_status == LLD_FSMC_NORSTATUS_READ)
            {
              erase_end = TRUE;
            }
          }
          else
          {
            erase_end = TRUE;

            SVC_FSMC_TRC_Add( 0xadd00020U);
          }
        }

        SVC_FSMC_TRC_Add( 0xadd000F0U)
      }
      else
      {
        SVC_FSMC_TRC_Add( 0xdeaddeadU);
      }

      LLD_FSMC_NorResetBlockStatus( nor_cfg, block_curr);
      region->erase_status.last_status = LLD_FSMC_NORSTATUS_READ;
      region->erase_status.block = SVC_FSMC_NOR_NULLBLOCKID;

      gpOS_task_unlock();

      //gpOS_semaphore_signal( region->write_access_sem);
    }

    if( msg->data.msg_erase_data.waiting_erase_end == TRUE)
    {
      gpOS_semaphore_signal( region->erase_ended_sem);
    }

    gpOS_message_release( svc_fsmc_nor_handler->msg_queue, msg);

    gpOS_semaphore_signal(region->write_access_sem);
  }

  // should never reach this
  return -1;
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
gpOS_error_t svc_fsmc_nor_init( gpOS_partition_t *partition)
{
  if( svc_fsmc_nor_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_fsmc_nor_handler = gpOS_memory_allocate_p( partition, SVC_FSMC_HANDLER_SIZE);

  if( svc_fsmc_nor_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  svc_fsmc_nor_handler->msg_queue = gpOS_message_create_queue_p( partition, SVC_FSMC_MSG_SIZE, SVC_FSMC_MSG_NUMBER);
  svc_fsmc_nor_handler->task = gpOS_task_create_p( partition, svc_fsmc_erase_process, NULL, SVC_FSMC_TASKERASE_WS, SVC_FSMC_TASKERASE_PRIORITY, "SVC_FSMC_SRVE", gpOS_TASK_FLAGS_ACTIVE);

  if( (svc_fsmc_nor_handler->msg_queue == NULL) || (svc_fsmc_nor_handler->task == NULL))
  {
    gpOS_task_delete( svc_fsmc_nor_handler->task);
    gpOS_message_delete_queue( svc_fsmc_nor_handler->msg_queue);
  }

  svc_fsmc_nor_handler->partition    = partition;
  svc_fsmc_nor_handler->head_region  = NULL;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param bank LLD_FSMC_SRAMNORBankTy
 * \param start_addr tUInt
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_fsmc_nor_create_region( LLD_FSMC_SRAMNORBankTy bank, tUInt start_addr, tUInt size)
{
  svc_fsmc_nor_region_t *curr_region;

  if( svc_fsmc_nor_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Check if that region overlaps another. */
  if( svc_fsmc_check_region_conflicts( start_addr, size) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;
  }

  /**< Allocate needed memory */
  curr_region = gpOS_memory_allocate_p( svc_fsmc_nor_handler->partition, SVC_FSMC_REGION_SIZE);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  curr_region->write_access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_fsmc_nor_handler->partition, 0);
  curr_region->erase_ended_sem  = gpOS_semaphore_create_p( SEM_FIFO, svc_fsmc_nor_handler->partition, 0);

  if(
      (curr_region->write_access_sem == NULL) ||
      (curr_region->erase_ended_sem  == NULL)
    )
  {
    gpOS_semaphore_delete( curr_region->write_access_sem);
    gpOS_semaphore_delete( curr_region->erase_ended_sem);
    gpOS_memory_deallocate_p( svc_fsmc_nor_handler->partition, curr_region);
  }

  /**< Configure NOR config structure */
  curr_region->cfg.bank_no = bank;
  curr_region->cfg.bank_start_addr = (volatile tU32 *)start_addr;
  LLD_FSMC_NorConfigure( &curr_region->cfg);

  /**< Update region parameters */
  curr_region->start_address  = start_addr;
  curr_region->size           = size;
  curr_region->next           = NULL;

  /**< Enqueue new region in handler */
  if( svc_fsmc_nor_handler->head_region == NULL)
  {
    svc_fsmc_nor_handler->head_region = curr_region;
  }
  else
  {
    svc_fsmc_nor_region_t *next_region;

    next_region = svc_fsmc_nor_handler->head_region;

    while( next_region->next != NULL)
    {
      next_region = next_region->next;
    }

    next_region->next = curr_region;
  }

  curr_region->erase_status.last_status       = LLD_FSMC_NORSTATUS_READ;
  curr_region->erase_status.address           = 0;
  curr_region->erase_status.block             = SVC_FSMC_NOR_NULLBLOCKID;
  curr_region->erase_status.data              = 0;

  curr_region->prog_status.last_status        = LLD_FSMC_NORSTATUS_READ;
  curr_region->prog_status.address            = 0;
  curr_region->prog_status.block              = SVC_FSMC_NOR_NULLBLOCKID;
  curr_region->prog_status.data               = 0;

  /**< Free access to region */
  gpOS_semaphore_signal( curr_region->write_access_sem);

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
FLASH_MODIFY gpOS_error_t svc_fsmc_nor_reset_region( void *addr, tUInt size)
{
  svc_fsmc_nor_region_t *curr_region;
  tUInt nor_addr;
  tU32 block_num, block_pos;
  tU32 curr_size = 0;

  if( svc_fsmc_nor_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Look for address in regions */
  curr_region = svc_fsmc_lookup_region( (tUInt)addr, size);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  gpOS_semaphore_wait( curr_region->write_access_sem);

  nor_addr = (tUInt)addr - (tUInt)curr_region->cfg.bank_start_addr;

  /**< Cleanup region */
  if( LLD_FSMC_NorAddrToBlockPos( &curr_region->cfg, (tU32)nor_addr, &block_num, &block_pos) != LLD_ERROR)
  {
    for( curr_size = 0; curr_size < size; )
    {
      LLD_FSMC_NorStatusTy nor_status;

      gpOS_interrupt_lock();

      nor_status = LLD_FSMC_NorGetBlockStatus( &curr_region->cfg, block_num, LLD_FSMC_NORSTATUS_ERASING);

      if( nor_status == LLD_FSMC_NORSTATUS_ERASING)
      {
        while( (nor_status != LLD_FSMC_NORSTATUS_READ) && (nor_status != LLD_FSMC_NORSTATUS_ERROR))
        {
          nor_status = LLD_FSMC_NorGetBlockStatus( &curr_region->cfg, block_num, LLD_FSMC_NORSTATUS_ERASING);
        }
      }
      else
      {
        LLD_FSMC_NorEraseResume( &curr_region->cfg, block_num);

        while( (nor_status != LLD_FSMC_NORSTATUS_READ) && (nor_status != LLD_FSMC_NORSTATUS_ERROR))
        {
          nor_status = LLD_FSMC_NorGetBlockStatus( &curr_region->cfg, block_num, LLD_FSMC_NORSTATUS_ERASING);
        }
      }

      LLD_FSMC_NorResetBlockStatus( &curr_region->cfg, block_num);

      gpOS_interrupt_unlock();

      curr_size += LLD_FSMC_NorBlockSize( &curr_region->cfg, block_num);
      block_num++;
    }

    curr_region->erase_status.last_status       = LLD_FSMC_NORSTATUS_READ;
    curr_region->erase_status.address           = 0;
    curr_region->erase_status.block             = SVC_FSMC_NOR_NULLBLOCKID;
    curr_region->erase_status.data              = 0;
  }

  gpOS_semaphore_signal( curr_region->write_access_sem);

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
gpOS_error_t svc_fsmc_nor_unlock( void *addr, tUInt size)
{
  svc_fsmc_nor_region_t *curr_region;
  tUInt nor_addr;
  tU32 block_num, block_pos;
  tU32 curr_size = 0;

  if( svc_fsmc_nor_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Look for address in regions */
  curr_region = svc_fsmc_lookup_region( (tUInt)addr, size);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  gpOS_semaphore_wait( curr_region->write_access_sem);

  /**< Unlock region */
  nor_addr = (tUInt)addr - (tUInt)curr_region->cfg.bank_start_addr;

  if( LLD_FSMC_NorAddrToBlockPos( &curr_region->cfg, (tU32)nor_addr, &block_num, &block_pos) != LLD_ERROR)
  {
    for( curr_size = 0; curr_size < size; )
    {
      LLD_FSMC_NorUnlockBlock( &curr_region->cfg, block_num);

      curr_size += LLD_FSMC_NorBlockSize( &curr_region->cfg, block_num);

      block_num++;
    }
  }

  gpOS_semaphore_signal( curr_region->write_access_sem);

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
gpOS_error_t svc_fsmc_nor_lock( void *addr, tUInt size)
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
gpOS_error_t svc_fsmc_nor_write( void *dest_addr, void *src_addr, tUInt size)
{
  svc_fsmc_nor_region_t *curr_region;
  tU32 nor_addr;
  LLD_FSMC_NorCfgTy *curr_flash_nor_cfg;
  gpOS_error_t exit_status = gpOS_SUCCESS;

  if( size == 0)
  {
    return gpOS_SUCCESS;
  }

  if( svc_fsmc_nor_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Look for address in regions */
  curr_region = svc_fsmc_lookup_region( (tUInt)dest_addr, size);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  curr_flash_nor_cfg = &curr_region->cfg;

  gpOS_semaphore_wait( curr_region->write_access_sem);

  nor_addr = (tU32)((tUInt)dest_addr - (tUInt)curr_flash_nor_cfg->bank_start_addr);

  {
    tU32 pos = 0;
    tU32 src_ptr_int = (tU32)src_addr;
    tU16 nor_step_size = curr_flash_nor_cfg->params.step_size;
    tU16 data_to_write;
    tU32 addr_to_write;

    // check if destination address is wrongly aligned for 16 bits nor
    if( (nor_step_size == LLD_FSMC_NOR_SIZE_HALF) && (nor_addr & 0x1))
    {
      // Update word with proper data
      tU8 byte_to_write = *((tU8 *)src_ptr_int);
      addr_to_write = (tUInt)(nor_addr & ~0x1) + (tUInt)curr_flash_nor_cfg->bank_start_addr;
      data_to_write = *((tU16 *)addr_to_write);

      if( ~(data_to_write >> 8) & byte_to_write)
      {
        // byte cannot be written
        exit_status = gpOS_FAILURE;
      }
      else
      {
        data_to_write = (data_to_write & 0xff) | ((tU16)byte_to_write << 8);

        if( svc_fsmc_nor_write_single( curr_region, nor_addr, (tVoid *)&data_to_write) == gpOS_FAILURE)
        {
          exit_status = gpOS_FAILURE;
        }
        else
        {
          src_ptr_int++;
          nor_addr++;
          pos++;
        }
      }
    }

    if( exit_status == gpOS_SUCCESS)
    {
      tU32 addr_step = 1 << nor_step_size;

      //for( ; pos < size; pos += nor_step_size, src_ptr_int += nor_step_size)
      while( (pos + addr_step - 1) < size)
      {
        tU32 old_val = 0, new_val = 0;

        addr_to_write = (tUInt)nor_addr + (tUInt)curr_flash_nor_cfg->bank_start_addr;

        if( nor_step_size == LLD_FSMC_NOR_SIZE_BYTE)
        {
          old_val = *((tU8 *)addr_to_write);
          new_val = ((tU8 *)src_ptr_int)[0];
        }
        else if( nor_step_size == LLD_FSMC_NOR_SIZE_HALF)
        {
          old_val = *((tU16 *)addr_to_write);
          new_val = ((tU16)(((tU8 *)src_ptr_int)[1]) << 8) | (((tU8 *)src_ptr_int)[0]);
        }

        if( ~old_val & new_val )
        {
          exit_status = gpOS_FAILURE;
          break;
        }

        /**< Start program phase */
        if( svc_fsmc_nor_write_single( curr_region, nor_addr, (tVoid *)&new_val) == gpOS_FAILURE)
        {
          exit_status = gpOS_FAILURE;
        }

        nor_addr += addr_step;
        src_ptr_int += addr_step;
        pos += addr_step;
      }

      // check if more bytes must be written
      if( (nor_step_size == LLD_FSMC_NOR_SIZE_HALF) && (pos < size))
      {
        // Update word with proper data
        tU8 byte_to_write = *((tU8 *)src_ptr_int);
        addr_to_write = (tUInt)nor_addr  + (tUInt)curr_flash_nor_cfg->bank_start_addr;
        data_to_write = *((tU16 *)addr_to_write);

        if( ~(data_to_write & 0xff) & byte_to_write)
        {
          // byte cannot be written
          exit_status = gpOS_FAILURE;
        }
        else
        {
          data_to_write = (data_to_write & 0xff00) | byte_to_write;

          if( svc_fsmc_nor_write_single( curr_region, nor_addr, (tVoid *)&data_to_write) == gpOS_FAILURE)
          {
            exit_status = gpOS_FAILURE;
          }
        }
      }
    }
  }

  gpOS_semaphore_signal( curr_region->write_access_sem);

  return( exit_status);
}

/********************************************//**
 * \brief
 *
 * \param addr void*
 * \param size tUInt
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_fsmc_nor_erase( void *addr, tUInt size, boolean_t wait_for_completion)
{
  svc_fsmc_nor_region_t *curr_region;
  tU32 block_start, block_num, block_pos;
  tU32 curr_size;
  tU32 nor_addr;
  LLD_FSMC_NorCfgTy *curr_flash_nor_cfg;
  svc_fsmc_message_t *msg;

  if( svc_fsmc_nor_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  if( size == 0)
  {
    return gpOS_SUCCESS;
  }

  /**< Look for address in regions */
  curr_region = svc_fsmc_lookup_region( (tUInt)addr, size);

  if( curr_region == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Evaluate start block and number of blocks to erase */
  curr_flash_nor_cfg = &curr_region->cfg;

  nor_addr = (tUInt)addr - (tUInt)curr_flash_nor_cfg->bank_start_addr;

  if( LLD_FSMC_NorAddrToBlockPos( curr_flash_nor_cfg, nor_addr, &block_start, &block_pos) == LLD_ERROR)
  {
    return gpOS_FAILURE;
  }

  block_num = 0;
  for( curr_size = 0; curr_size < size; )
  {
    curr_size += LLD_FSMC_NorBlockSize( curr_flash_nor_cfg, block_start + block_num);
    block_num++;
  }

  gpOS_semaphore_wait( curr_region->write_access_sem);

  msg = (svc_fsmc_message_t *)gpOS_message_claim( svc_fsmc_nor_handler->msg_queue);

  msg->type = SVC_FSMC_MSG_ERASE;
  msg->data.msg_erase_data.region             = curr_region;
  msg->data.msg_erase_data.block_start        = block_start;
  msg->data.msg_erase_data.block_number       = block_num;
  msg->data.msg_erase_data.waiting_erase_end  = wait_for_completion;

  gpOS_message_send( svc_fsmc_nor_handler->msg_queue, msg);

  if( wait_for_completion == TRUE)
  {
    gpOS_semaphore_wait( curr_region->erase_ended_sem);
  }

  return gpOS_SUCCESS;
}
