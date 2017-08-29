/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Implements a SDI.
 *
 ******************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"

#include "lld_sdi.h"

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_sdi.h"
#include "svc_pwr.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_SDI_HANDLER_SIZE        sizeof( svc_sdi_handler_t)
#define SVC_SDI_PORT_HANDLER_SIZE   sizeof( svc_sdi_port_handler_t)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief SDI port handler
 ***********************************************/
typedef struct svc_sdi_port_handler_s svc_sdi_port_handler_t;

struct svc_sdi_port_handler_s
{
  svc_sdi_port_handler_t *  next;               /**< Next SDI port handler */

  tUInt                     port_id;            /**< ID of SDI port */
  gpOS_semaphore_t *        access_sem;         /**< access semaphore */

  LLD_SDI_SystemContextTy   system_context;     /**< SDI context handler */
  peripherallockid_t        read_peripherallock_id;  /**< Pwr handler  */
  peripherallockid_t        write_peripherallock_id;  /**< Pwr handler  */

};

/********************************************//**
 * \brief SDI handler
 ***********************************************/
typedef struct svc_sdi_handler_s
{
  svc_mcu_item_t             svc_mcu_item_handler;

  svc_sdi_port_handler_t *    port_head;        /**< Linked list of ports */
} svc_sdi_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static svc_sdi_handler_t *svc_sdi_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief   Get UART handler for specific ID
 *
 * \param   uart_id   ID of wanted UART handler
 * \return  svc_uart_port_handler_t *  pointer to UART handler, or NULL if not open
 *
 ***********************************************/
static svc_sdi_port_handler_t *svc_sdi_get_hdlr_ptr( tUInt sdi_port)
{
  svc_sdi_port_handler_t *port_hdlr_ptr;

  port_hdlr_ptr = svc_sdi_handler->port_head;

  while( port_hdlr_ptr != NULL)
  {
    if( port_hdlr_ptr->port_id == sdi_port)
    {
      return( port_hdlr_ptr);
    }
    else
    {
      port_hdlr_ptr = port_hdlr_ptr->next;
    }
  }

  return( NULL);
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Initialize SDI svc_mcu
 *
 * \param partition Partition used for memory allocation
 * \param bus_speed Bus speed
 * \return gpOS_SUCCESS if all goes ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_sdi_init( gpOS_partition_t *partition, tU32 bus_id )
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_sdi_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_sdi_handler = gpOS_memory_allocate_p( partition, SVC_SDI_HANDLER_SIZE);

  if( svc_sdi_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  // Install service on main handler and fill physical info
  if( svc_mcu_install( SVC_MCU_PER_ID_SDI, bus_id, &svc_sdi_handler->svc_mcu_item_handler) == gpOS_FAILURE)
  {
    gpOS_memory_deallocate_p( partition, svc_sdi_handler);
    return gpOS_FAILURE;
  }

  // Fill specific fields
  svc_sdi_handler->svc_mcu_item_handler.part   = partition;
  svc_sdi_handler->svc_mcu_item_handler.mem_used    = mem_at_start - gpOS_memory_getheapfree_p( partition);

  svc_sdi_handler->port_head   = NULL;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param bus_speed tU32
 * \return void
 *
 ***********************************************/
void svc_sdi_refresh( tU32 bus_speed )
{

}

/********************************************//**
 * \brief Open SDI port
 *
 * \param sdi_port SDI Port number
 * \param irq_pri Priority for SDI interrupt
 * \return gpOS_SUCCESS if all goes ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_sdi_open_port( tUInt sdi_port, gpOS_interrupt_priority_t irq_pri )
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_sdi_handler->svc_mcu_item_handler.part);
  svc_sdi_port_handler_t *last_port_hdlr_ptr, *port_hdlr_ptr;

  LLD_SDI_IdTy sdi_phy_id;
  LLD_SDI_ErrorTy error;

  if( (svc_sdi_handler == NULL) || (sdi_port >= svc_sdi_handler->svc_mcu_item_handler.phy_item->number))
  {
    return gpOS_FAILURE;
  }

  /**< Check if port was already open */
  last_port_hdlr_ptr = svc_sdi_handler->port_head;
  port_hdlr_ptr = svc_sdi_handler->port_head;

  while( port_hdlr_ptr != NULL)
  {
    if( port_hdlr_ptr->port_id == sdi_port)
    {
      return( gpOS_FAILURE);
    }
    else
    {
      last_port_hdlr_ptr = port_hdlr_ptr;
      port_hdlr_ptr = port_hdlr_ptr->next;
    }
  }

  sdi_phy_id = (LLD_SDI_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SDI, sdi_port);

  port_hdlr_ptr = gpOS_memory_allocate_p( svc_sdi_handler->svc_mcu_item_handler.part, SVC_SDI_PORT_HANDLER_SIZE);

  if( port_hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;
  }

  port_hdlr_ptr->access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_sdi_handler->svc_mcu_item_handler.part, 0);

  if( port_hdlr_ptr->access_sem == NULL)
  {
    gpOS_memory_deallocate_p( svc_sdi_handler->svc_mcu_item_handler.part, port_hdlr_ptr);
    return gpOS_FAILURE;
  }
  port_hdlr_ptr->port_id = sdi_port;

  svc_mcu_enable( SVC_MCU_PER_ID_SDI, sdi_port);

  // Select card
  LLD_SDI_SelectCard(sdi_phy_id, LLD_SDI_EXTERNAL_CARD);

  // Card power-on
  error = LLD_SDI_PowerON( sdi_phy_id, &port_hdlr_ptr->system_context );
  if( error)
  {
    gpOS_semaphore_delete( port_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_sdi_handler->svc_mcu_item_handler.part, port_hdlr_ptr);
    return gpOS_FAILURE;
  }
  // Card initialization
  error = LLD_SDI_InitializeCards( sdi_phy_id, &port_hdlr_ptr->system_context );
  if( error)
  {
    gpOS_semaphore_delete( port_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_sdi_handler->svc_mcu_item_handler.part, port_hdlr_ptr);
    return gpOS_FAILURE;
  }
  // Bus clock is 26 MHz
  error = LLD_SDI_SetClockFrequency( sdi_phy_id, 0);
  if( error)
  {
    gpOS_semaphore_delete( port_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_sdi_handler->svc_mcu_item_handler.part, port_hdlr_ptr);
    return gpOS_FAILURE;
  }
  // Bus width is 8 bits
  error = LLD_SDI_EnableWideBusOperation( sdi_phy_id, &port_hdlr_ptr->system_context, 1, LLD_SDI_4_BIT_WIDE );
  if( error)
  {
    gpOS_semaphore_delete( port_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_sdi_handler->svc_mcu_item_handler.part, port_hdlr_ptr);
    return gpOS_FAILURE;
  }

  if( last_port_hdlr_ptr == NULL)
  {
    svc_sdi_handler->port_head = port_hdlr_ptr;
  }
  else
  {
    last_port_hdlr_ptr = port_hdlr_ptr;
  }

  svc_sdi_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_sdi_handler->svc_mcu_item_handler.part);

  svc_pwr_peripherallock_register(&port_hdlr_ptr->read_peripherallock_id);
  svc_pwr_peripherallock_register(&port_hdlr_ptr->write_peripherallock_id);

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Read sectors from SDI port
 *
 * \param sdi_port SDI port
 * \param sector_start Start sector
 * \param sector_number Number of sectors to read
 * \param sector_size Size of each sector
 * \param buffer_ptr Pointer to buffer to fill
 * \return gpOS_SUCCESS if all goes ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_sdi_read( tUInt sdi_port, tUInt sector_start, tUInt sector_number, tUInt sector_size, tU8 *buffer_ptr)
{
  LLD_SDI_IdTy sdi_phy_id;
  svc_sdi_port_handler_t *port_hdlr_ptr;
  gpOS_error_t error = gpOS_SUCCESS;

  if( (svc_sdi_handler == NULL) || (sdi_port >=  svc_sdi_handler->svc_mcu_item_handler.phy_item->number))
  {
    return gpOS_FAILURE;
  }

  port_hdlr_ptr = svc_sdi_get_hdlr_ptr( sdi_port);

  if( port_hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;
  }

  sdi_phy_id = (LLD_SDI_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SDI, sdi_port);

  gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

  svc_pwr_peripherallock_acquire(port_hdlr_ptr->read_peripherallock_id);

  if( LLD_SDI_ReadBlocks( sdi_phy_id, &port_hdlr_ptr->system_context, 1, sector_start, (tU32 *)buffer_ptr, sector_size, sector_number ) != LLD_SDI_OK)
  {
  	error = gpOS_FAILURE;
  }

  svc_pwr_peripherallock_release(port_hdlr_ptr->read_peripherallock_id);

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  return error;
}

/********************************************//**
 * \brief Write sectors to SDI port
 *
 * \param sdi_port SDI port
 * \param sector_start Start sector
 * \param sector_number Number of sectors to write
 * \param sector_size Size of each sector
 * \param buffer_ptr Pointer to buffer to write
 * \return gpOS_SUCCESS if all goes ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_sdi_write( tUInt sdi_port, tUInt sector_start, tUInt sector_number, tUInt sector_size, tU8 *buffer_ptr)
{
  LLD_SDI_IdTy sdi_phy_id;
  svc_sdi_port_handler_t *port_hdlr_ptr;
  gpOS_error_t error = gpOS_SUCCESS;

  if( (svc_sdi_handler == NULL) || (sdi_port >= svc_sdi_handler->svc_mcu_item_handler.phy_item->number))
  {
    return gpOS_FAILURE;
  }

  port_hdlr_ptr = svc_sdi_get_hdlr_ptr( sdi_port);

  if( port_hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;
  }

  sdi_phy_id = (LLD_SDI_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SDI, sdi_port);

  gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

  svc_pwr_peripherallock_acquire(port_hdlr_ptr->write_peripherallock_id);

  if( LLD_SDI_WriteBlocks( sdi_phy_id, &port_hdlr_ptr->system_context, 1, sector_start, (tU32 *)buffer_ptr, sector_size, sector_number ) != LLD_SDI_OK)
  {
  	error = gpOS_FAILURE;
  }

  svc_pwr_peripherallock_release(port_hdlr_ptr->write_peripherallock_id);

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  return error;
}

/********************************************//**
 * \brief Check if SDI is busy
 *
 * \param sdi_port SDI port
 * \return TRUE if SDI is operating, else FALSE
 *
 ***********************************************/
boolean_t svc_sdi_is_port_busy( tUInt sdi_port)
{
  LLD_SDI_IdTy sdi_phy_id;
  svc_sdi_port_handler_t *port_hdlr_ptr;
  boolean_t busy = FALSE;

  if( (svc_sdi_handler == NULL) || (sdi_port >=  svc_sdi_handler->svc_mcu_item_handler.phy_item->number))
  {
    busy = TRUE;
  }
  else
  {

    port_hdlr_ptr = svc_sdi_get_hdlr_ptr( sdi_port);

    if( port_hdlr_ptr == NULL)
    {
      busy = FALSE;
    }
    else
    {
      sdi_phy_id        = (LLD_SDI_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SDI, sdi_port);

      if( LLD_SDI_GetTransferState( sdi_phy_id ) == LLD_SDI_TRANSFER_IN_PROGRESS)
      {
        busy = TRUE;
      }
    }

  }

  return busy;
}
