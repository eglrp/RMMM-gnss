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

#include "lld_i2c.h"

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_i2c.h"
#include "svc_pwr.h"
#include "string.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define   SVC_I2C_HANDLER_SIZE         sizeof( svc_i2c_handler_t)
#define   SVC_I2C_PORT_HANDLER_SIZE    sizeof( svc_i2c_port_handler_t)
#define   SVC_I2C_COM_HANDLER_SIZE     sizeof( svc_i2c_com_handler_t)

#define   SVC_I2C_TXIRQS               (LLD_I2C_IRQ_TRANSMIT_FIFO_NEARLY_EMPTY)
#define   SVC_I2C_RXIRQS               (LLD_I2C_IRQ_RECEIVE_FIFO_FULL | LLD_I2C_IRQ_RECEIVE_FIFO_NEARLY_FULL)

#define   SVC_I2C_SLVADDR_TICKS        40
#define   SVC_I2C_BAUDRATE_FACTOR_STD  1
#define   SVC_I2C_BAUDRATE_FACTOR_FAST 4
#define   SVC_I2C_BAUDRATE_FACTOR_HS   16
#define   SVC_I2C_MAX_OUT_FREQ         1600000


/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Handler for specific I2C peripheral
 ***********************************************/
typedef struct svc_i2c_com_hooks_s
{
  svc_i2c_hook_t                 pre_cb;         /**< callback to use before any R/w operation */
  tVoid *                         pre_cb_param;   /**< parameter fro pre_cb */

  svc_i2c_hook_t                 post_cb;        /**< callback to use after any R/w operation */
  tVoid *                         post_cb_param;  /**< parameter fro post_cb */
} svc_i2c_com_hooks_t;

/********************************************//**
 * \brief I2C COM handler
 ***********************************************/
struct svc_i2c_com_handler_s
{
  svc_i2c_com_handler_t *    next;                   /**< Pointer to next COM in the queue */

  tUInt                       port_id;                /**< I2C port associated to the COM */
  gpOS_semaphore_t *               access_sem;             /**< access semaphore to com */

  svc_i2c_com_hooks_t        hooks;                  /**< Pre/post callbacks for specified COM */

  LLD_I2C_configTy            config;                 /**< I2C configuration for COM */
  tUInt                       addr;                   /**< Address of connected peripheral */
};

typedef enum svc_i2c_operation_e
{
  SVC_I2C_OPERATION_READ,
  SVC_I2C_OPERATION_WRITE
} svc_i2c_operation_t;

/********************************************//**
 * \brief I2C Port handler
 ***********************************************/
typedef struct svc_i2c_port_handler_s svc_i2c_port_handler_t;

typedef struct svc_i2c_port_handler_s
{
  svc_i2c_port_handler_t *   next;                   /**< Next I2C port */
  tUInt                       port_id;                /**< Port ID */

  svc_i2c_com_handler_t *    curr_com_ptr;           /**< COM currently using the port */

  gpOS_semaphore_t *               access_sem;             /**< access semaphore to port */

  svc_i2c_operation_t        curr_op;                /**< current operation */
  gpOS_semaphore_t *               done_sem;               /**< semaphore to signal end of transfer */
  tU8 *                       buf_ptr;                /**< Pointer to TX/RX buffer */
  tU16                        buf_pos;                /**< Position in TX/RX buffer */
  tU16                        buf_len;                /**< Length of TX/RX request */

  tUInt                       slave_addr;             /**< Slave address of this port */
  peripherallockid_t          write_peripherallockid; /**< pwr status */
  peripherallockid_t          read_peripherallockid;  /**< pwr status */
} svc_i2c_port_handler_t;

/********************************************//**
 * \brief I2C svc_mcu handler
 ***********************************************/
typedef struct svc_i2c_handler_s
{
  svc_mcu_item_t             svc_mcu_item_handler;

  svc_i2c_com_handler_t *    com_head;               /**< Linked list of COMs */
  svc_i2c_port_handler_t *   port_head;              /**< Linked list of ports */
} svc_i2c_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/**< Standard I2C_configuration for I2C protocol */
static LLD_I2C_configTy svc_i2c_basecfg = {
  LLD_I2C_DGTLFILTER_OFF,
  #if defined( LLD_I2C_DMASUPPORT)
  LLD_I2C_DMASYNCLOGIC_OFF,
  #endif
  LLD_I2C_STANDARD_MODE,
  0,
  0,
  8,
  8,
  0,
  0,
  0,
  LLD_I2C_STARTPROC_DISABLED
};

static svc_i2c_handler_t *svc_i2c_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static svc_i2c_port_handler_t *  svc_i2c_get_hdlr_ptr   ( tUInt i2c_port);
static void                       svc_i2c_callback       ( svc_i2c_port_handler_t *);

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief   Get I2C handler for specific port
 *
 * \param   i2c_port  port of wanted I2C handler
 * \return  svc_i2c_port_handler_t *  pointer to I2C handler, or NULL if not open
 *
 ***********************************************/
static svc_i2c_port_handler_t *svc_i2c_get_hdlr_ptr( tUInt i2c_port)
{
  svc_i2c_port_handler_t *port_hdlr_ptr;

  port_hdlr_ptr = svc_i2c_handler->port_head;

  while( port_hdlr_ptr != NULL)
  {
    if( port_hdlr_ptr->port_id == i2c_port)
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

/********************************************//**
 * \brief I2C svc_mcu interrupt callback
 *
 * \param port_hdlr_ptr Port handler pointer
 * \return void
 *
 ***********************************************/

static LLD_ISR_I2C void svc_i2c_callback( svc_i2c_port_handler_t *port_hdlr_ptr)
{
  LLD_I2C_IdTy i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, port_hdlr_ptr->port_id);
  LLD_I2C_IrqSrcTy irq_status;

  irq_status = LLD_I2C_GetInterruptStatus( i2c_id, LLD_I2C_IRQ_ALL);

  if( irq_status & (tU32)LLD_I2C_IRQ_BUS_ERROR)
  {
    LLD_I2C_InterruptDisable( i2c_id, LLD_I2C_IRQ_ALL);
    LLD_I2C_ClearInterrupt( i2c_id, LLD_I2C_IRQ_ALL);
  }
  else if( LLD_I2C_GetOperatingMode( i2c_id) == LLD_I2C_BUSCTRLMODE_MASTER )
  {
    if( irq_status & LLD_I2C_IRQ_MASTER_TRANSACTION_DONE)
    {
      /**< This could be either a read or a write */
      LLD_I2C_InterruptDisable( i2c_id, LLD_I2C_IRQ_ALL);

      if( LLD_I2C_GetMasterMode( i2c_id) == LLD_I2C_MSTMODE_READ)
      {
        port_hdlr_ptr->buf_pos +=  LLD_I2C_ReadFifo( i2c_id, &port_hdlr_ptr->buf_ptr[port_hdlr_ptr->buf_pos], port_hdlr_ptr->buf_len - port_hdlr_ptr->buf_pos);
      }

      LLD_I2C_ClearInterrupt( i2c_id, LLD_I2C_IRQ_ALL_CLEARABLE);

      gpOS_semaphore_signal( port_hdlr_ptr->done_sem);
    }
    else if( port_hdlr_ptr->curr_op == SVC_I2C_OPERATION_WRITE)
    {
      if( irq_status & SVC_I2C_TXIRQS)
      {
        port_hdlr_ptr->buf_pos +=  LLD_I2C_WriteFifo( i2c_id, &port_hdlr_ptr->buf_ptr[port_hdlr_ptr->buf_pos], port_hdlr_ptr->buf_len - port_hdlr_ptr->buf_pos);

        if( port_hdlr_ptr->buf_pos == port_hdlr_ptr->buf_len)
        {
          LLD_I2C_InterruptDisable( i2c_id, SVC_I2C_TXIRQS);
        }

        LLD_I2C_ClearInterrupt( i2c_id, SVC_I2C_TXIRQS & LLD_I2C_IRQ_ALL_CLEARABLE);
      }
    }
    else if( port_hdlr_ptr->curr_op == SVC_I2C_OPERATION_READ)
    {
      if( irq_status & LLD_I2C_IRQ_TRANSMIT_FIFO_EMPTY)
      {
        LLD_I2C_InterruptDisable( i2c_id, LLD_I2C_IRQ_TRANSMIT_FIFO_EMPTY);
        gpOS_semaphore_signal( port_hdlr_ptr->done_sem);
      }
      if( irq_status & SVC_I2C_RXIRQS)
      {
        port_hdlr_ptr->buf_pos +=  LLD_I2C_ReadFifo( i2c_id, &port_hdlr_ptr->buf_ptr[port_hdlr_ptr->buf_pos], port_hdlr_ptr->buf_len - port_hdlr_ptr->buf_pos);

        if( port_hdlr_ptr->buf_pos == port_hdlr_ptr->buf_len)
        {
          LLD_I2C_InterruptDisable( i2c_id, SVC_I2C_RXIRQS);
          LLD_I2C_InterruptEnable( i2c_id, LLD_I2C_IRQ_MASTER_TRANSACTION_DONE);
        }

        LLD_I2C_ClearInterrupt( i2c_id, SVC_I2C_RXIRQS & LLD_I2C_IRQ_ALL_CLEARABLE);
      }
    }
  }
}

/********************************************//**
 * \brief Callback function called by bus service
 *
 * \param[in] cmd_id type of command requested
 * \param[in] param additional parameter for
 *                  the request
 *
 ***********************************************/
static gpOS_error_t svc_i2c_cmdcallback_exec( svc_mcu_cmd_id_t cmd_id, void *param)
{
  svc_i2c_com_handler_t *curr_com_port = NULL;
  svc_i2c_com_handler_t *com_hdlr_ptr = svc_i2c_handler->com_head;

  curr_com_port = svc_i2c_handler->port_head->curr_com_ptr;

  if (cmd_id == SVC_MCU_CMD_ID_CHANGE_SPEED)
  {
    /* I2C baudrate shall be adjusted to maintain same baudrate if possible or a value as close as possible
    from the current baudrate, using BRCNTx registers.*/

    if (com_hdlr_ptr == NULL)
    {
      // No I2C port open yet so update of the base configuration frequency
      svc_mcu_busclk_get( svc_i2c_handler->svc_mcu_item_handler.bus_id, &svc_i2c_basecfg.input_freq);
    }
    else
    {
      tU32 freq = 0;

      while( com_hdlr_ptr != NULL)
      {
        svc_mcu_busclk_get( svc_i2c_handler->svc_mcu_item_handler.bus_id, &freq);

        // Update base configuration (in case an init occurs)
        svc_i2c_basecfg.input_freq = freq;

        // Update current configuration for open ports
        com_hdlr_ptr->config.input_freq  = freq;

        com_hdlr_ptr = com_hdlr_ptr->next;
      }
      // Update also current com port in case there is a frequency switch while I2C is running
      while( curr_com_port != NULL)
      {
        LLD_I2C_IdTy i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, curr_com_port->port_id);

        if (LLD_I2C_GetOperatingMode(i2c_id) == LLD_I2C_BUSCTRLMODE_MASTER)
        {
          // Update current configuration for open ports
          curr_com_port->config.input_freq = freq;

          // Reconfigure current com port speed in case a write/read was on-going
          LLD_I2C_ResetReg( i2c_id);
          LLD_I2C_Config( i2c_id, &curr_com_port->config);
          LLD_I2C_SetOperatingMode( i2c_id, LLD_I2C_BUSCTRLMODE_MASTER);

          curr_com_port = curr_com_port->next;
        }
      }
    }
  }
  else if (cmd_id == SVC_MCU_CMD_ID_SUSPEND_TRANSFER)
  {
    while( com_hdlr_ptr != NULL)
    {
      LLD_I2C_IdTy i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, com_hdlr_ptr->port_id);

      // Suspend transfer if Master
      if (LLD_I2C_GetOperatingMode(i2c_id) == LLD_I2C_BUSCTRLMODE_MASTER)
      {
        gpOS_semaphore_wait( com_hdlr_ptr->access_sem);
      }
      com_hdlr_ptr = com_hdlr_ptr->next;
    }
  }
  else if (cmd_id == SVC_MCU_CMD_ID_RESTORE_TRANSFER)
  {
    // Restart all UART ports
    while( com_hdlr_ptr != NULL)
    {
      LLD_I2C_IdTy i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, com_hdlr_ptr->port_id);

      // Suspend transfer if Master
      if (LLD_I2C_GetOperatingMode(i2c_id) == LLD_I2C_BUSCTRLMODE_MASTER)
      {
        gpOS_semaphore_signal( com_hdlr_ptr->access_sem);
      }
      com_hdlr_ptr = com_hdlr_ptr->next;
    }

  }
  return gpOS_SUCCESS;
}


/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Initialize I2C svc_mcu
 *
 * \param partition Partition to use to allocate memory
 * \param bus_speed Bus speed
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_i2c_init( gpOS_partition_t *partition, tU32 bus_id)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_i2c_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_i2c_handler = gpOS_memory_allocate_p( partition, SVC_I2C_HANDLER_SIZE);

  if( svc_i2c_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  // Install service on main handler and fill physical info
  if( svc_mcu_install( SVC_MCU_PER_ID_I2C, bus_id, &svc_i2c_handler->svc_mcu_item_handler) == gpOS_FAILURE)
  {
    gpOS_memory_deallocate_p( partition, svc_i2c_handler);
    return gpOS_FAILURE;
  }

  // Fill specific fields
  svc_i2c_handler->svc_mcu_item_handler.part       = partition;
  svc_i2c_handler->svc_mcu_item_handler.mem_used    = mem_at_start - gpOS_memory_getheapfree_p( partition);

  svc_i2c_handler->com_head    = NULL;
  svc_i2c_handler->port_head   = NULL;
  svc_i2c_handler->svc_mcu_item_handler.cmdif = svc_i2c_cmdcallback_exec;

  // Configure default I2C configuration
  svc_mcu_busclk_get( svc_i2c_handler->svc_mcu_item_handler.bus_id, &svc_i2c_basecfg.input_freq);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Open I2C Port
 *
 * \param i2c_port Port number (hw specific)
 * \param irq_pri Interrupt priority of I2C svc_mcu
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_i2c_open_port( tUInt i2c_port, gpOS_interrupt_priority_t irq_pri, tUInt slave_addr, LLD_I2C_BusCtrlModeTy start_mode)
{
  gpOS_error_t error = gpOS_SUCCESS;
  tU32 mem_at_start;
  svc_i2c_port_handler_t *last_port_hdlr_ptr, *port_hdlr_ptr;

  LLD_I2C_IdTy i2c_phy_id;
  VicLineTy i2c_phy_irq_line;

  if( svc_i2c_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  i2c_phy_id        = (LLD_I2C_IdTy)svc_i2c_handler->svc_mcu_item_handler.phy_item->addr[i2c_port];
  i2c_phy_irq_line  = (VicLineTy)svc_i2c_handler->svc_mcu_item_handler.phy_item->irq_line[i2c_port];
  mem_at_start      = gpOS_memory_getheapfree_p( svc_i2c_handler->svc_mcu_item_handler.part);

  /**< Check if port was already open */
  last_port_hdlr_ptr = svc_i2c_handler->port_head;
  port_hdlr_ptr = svc_i2c_handler->port_head;

  while( port_hdlr_ptr != NULL)
  {
    if( port_hdlr_ptr->port_id == i2c_port)
    {
      return( gpOS_FAILURE);
    }
    else
    {
      last_port_hdlr_ptr = port_hdlr_ptr;
      port_hdlr_ptr = port_hdlr_ptr->next;
    }
  }

  port_hdlr_ptr = gpOS_memory_allocate_p( svc_i2c_handler->svc_mcu_item_handler.part, SVC_I2C_PORT_HANDLER_SIZE);

  if( port_hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;
  }

  port_hdlr_ptr->access_sem       = gpOS_semaphore_create_p( SEM_FIFO, svc_i2c_handler->svc_mcu_item_handler.part, 0);
  port_hdlr_ptr->done_sem         = gpOS_semaphore_create_p( SEM_FIFO, svc_i2c_handler->svc_mcu_item_handler.part, 0);

  if( (port_hdlr_ptr->access_sem == NULL) || (port_hdlr_ptr->done_sem == NULL))
  {
    gpOS_semaphore_delete( port_hdlr_ptr->done_sem);
    gpOS_semaphore_delete( port_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_i2c_handler->svc_mcu_item_handler.part, port_hdlr_ptr);
    return gpOS_FAILURE;
  }

  port_hdlr_ptr->port_id      = i2c_port;

  port_hdlr_ptr->curr_com_ptr = NULL;

  port_hdlr_ptr->buf_ptr   = NULL;
  port_hdlr_ptr->buf_pos   = 0;
  port_hdlr_ptr->buf_len   = 0;
  port_hdlr_ptr->next      = NULL;

  gpOS_interrupt_install( i2c_phy_irq_line, irq_pri, (gpOS_interrupt_callback_t)svc_i2c_callback, port_hdlr_ptr);
  gpOS_interrupt_enable( i2c_phy_irq_line);

  if( last_port_hdlr_ptr == NULL)
  {
    svc_i2c_handler->port_head = port_hdlr_ptr;
  }
  else
  {
    last_port_hdlr_ptr = port_hdlr_ptr;
  }

  port_hdlr_ptr->slave_addr = slave_addr;

  /**< Configure device */
  svc_mcu_enable( SVC_MCU_PER_ID_I2C, i2c_port);

  LLD_I2C_ResetReg(i2c_phy_id);

  if( start_mode == LLD_I2C_BUSCTRLMODE_SLAVE)
  {
    LLD_I2C_Config( i2c_phy_id, &svc_i2c_basecfg);
    LLD_I2C_SetSlaveAddr( i2c_phy_id, slave_addr);
    LLD_I2C_Enable( i2c_phy_id);
  }

  svc_i2c_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_i2c_handler->svc_mcu_item_handler.part);

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  if( svc_i2c_reset(i2c_port) == gpOS_FAILURE)
  {
    error = gpOS_FAILURE;
  }

  return error;
}

/*lint -save -e9059  : C comment contains C++ comment [MISRA 2012 Rule 3.1, required]*/
/**********************************************
 * \brief Create COM on I2C port
 *
 * \param i2c_port I2C port used      //param ssp_mode Interface type
 * \param out_clk Out clock frequency //param data_size Size of each data transmitted
 * \param speed_mode Baud Rate
 * \param address Slave Address (7 or 10 bits)
 * \return COM handler pointer
 *
 ***********************************************/
/*lint -restore  : C comment contains C++ comment [MISRA 2012 Rule 3.1, required]*/
svc_i2c_com_handler_t *svc_i2c_create_com_speed( tUInt i2c_port, LLD_I2C_SpeedModeTy speed_mode, tU32 out_freq, tU16 address)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_i2c_handler->svc_mcu_item_handler.part);
  svc_i2c_com_handler_t *com_hdlr_ptr;
  svc_i2c_com_handler_t **last_com_hdlr_ptr_ptr;
  svc_i2c_port_handler_t *port_hdlr_ptr;

  /**< Check if I2C svc_mcu was initialized */
  if( svc_i2c_handler == NULL)
  {
    return NULL;
  }

  /**< check if specified port is available on platform */
  if( i2c_port >= svc_i2c_handler->svc_mcu_item_handler.phy_item->number)
  {
    return NULL;
  }

  /**< check if handler for specified port was initialized */
  port_hdlr_ptr = svc_i2c_get_hdlr_ptr( i2c_port);

  if( port_hdlr_ptr == NULL)
  {
    return NULL;
  }

  gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

  /**< allocate memory for COM handler and exit if no space is available */
  com_hdlr_ptr = gpOS_memory_allocate_p( svc_i2c_handler->svc_mcu_item_handler.part, SVC_I2C_COM_HANDLER_SIZE);

  if( com_hdlr_ptr != NULL)
  {
    /**< create access semaphore and exit if no space is available */
    com_hdlr_ptr->access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_i2c_handler->svc_mcu_item_handler.part, 0);

    if( com_hdlr_ptr->access_sem != NULL)
    {
      /**< Enqueue new COM handler on COM queue */
      last_com_hdlr_ptr_ptr = &svc_i2c_handler->com_head;

      while( *last_com_hdlr_ptr_ptr != NULL)
      {
        last_com_hdlr_ptr_ptr = &(*last_com_hdlr_ptr_ptr)->next;
      }

      *last_com_hdlr_ptr_ptr = com_hdlr_ptr;

      com_hdlr_ptr->next    = NULL;

      /**< Configure COM parameters */
      memcpy( &com_hdlr_ptr->config, &svc_i2c_basecfg, sizeof( LLD_I2C_configTy));

      com_hdlr_ptr->port_id               = i2c_port;
      com_hdlr_ptr->config.speed_mode     = speed_mode;
      com_hdlr_ptr->config.out_freq       = out_freq;
      com_hdlr_ptr->config.slave_address  = (tU16)port_hdlr_ptr->slave_addr;
      com_hdlr_ptr->addr                  = address;

      svc_i2c_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_i2c_handler->svc_mcu_item_handler.part);

      gpOS_semaphore_signal( com_hdlr_ptr->access_sem);
    }
    else
    {
      gpOS_semaphore_delete( com_hdlr_ptr->access_sem);
      gpOS_memory_deallocate_p( svc_i2c_handler->svc_mcu_item_handler.part, com_hdlr_ptr);
    }
  }

  svc_pwr_peripherallock_register(&port_hdlr_ptr->write_peripherallockid);
  svc_pwr_peripherallock_register(&port_hdlr_ptr->read_peripherallockid);

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  return com_hdlr_ptr;
}

/*lint -save -e9059  : C comment contains C++ comment [MISRA 2012 Rule 3.1, required]*/
/**********************************************
 * \brief Create COM on I2C port
 *
 * \param i2c_port I2C port used      //param ssp_mode Interface type
 * \param out_clk Out clock frequency //param data_size Size of each data transmitted
 * \param speed_mode Baud Rate
 * \param address Slave Address (7 or 10 bits)
 * \return COM handler pointer
 *
 ***********************************************/
/*lint -restore  : C comment contains C++ comment [MISRA 2012 Rule 3.1, required]*/
svc_i2c_com_handler_t *svc_i2c_create_com( tUInt i2c_port, LLD_I2C_SpeedModeTy speed_mode, tU16 address)
{
  svc_i2c_com_handler_t *com_hdlr_ptr;
#if 0
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_i2c_handler->svc_mcu_item_handler.part);
  svc_i2c_com_handler_t *com_hdlr_ptr;
  svc_i2c_com_handler_t **last_com_hdlr_ptr_ptr;
  svc_i2c_port_handler_t *port_hdlr_ptr;

  /**< Check if I2C svc_mcu was initialized */
  if( svc_i2c_handler == NULL)
  {
    return NULL;
  }

  /**< check if specified port is available on platform */
  if( i2c_port >= svc_i2c_handler->svc_mcu_item_handler.phy_item->number)
  {
    return NULL;
  }

  /**< check if handler for specified port was initialized */
  port_hdlr_ptr = svc_i2c_get_hdlr_ptr( i2c_port);

  if( port_hdlr_ptr == NULL)
  {
    return NULL;
  }

  gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

  /**< allocate memory for COM handler and exit if no space is available */
  com_hdlr_ptr = gpOS_memory_allocate_p( svc_i2c_handler->svc_mcu_item_handler.part, SVC_I2C_COM_HANDLER_SIZE);

  if( com_hdlr_ptr != NULL)
  {
    /**< create access semaphore and exit if no space is available */
    com_hdlr_ptr->access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_i2c_handler->svc_mcu_item_handler.part, 0);

    if( com_hdlr_ptr->access_sem != NULL)
    {
      /**< Enqueue new COM handler on COM queue */
      last_com_hdlr_ptr_ptr = &svc_i2c_handler->com_head;

      while( *last_com_hdlr_ptr_ptr != NULL)
      {
        last_com_hdlr_ptr_ptr = &(*last_com_hdlr_ptr_ptr)->next;
      }

      *last_com_hdlr_ptr_ptr = com_hdlr_ptr;

      com_hdlr_ptr->next    = NULL;

      /**< Configure COM parameters */
      memcpy( &com_hdlr_ptr->config, &svc_i2c_basecfg, sizeof( LLD_I2C_configTy));

      com_hdlr_ptr->port_id               = i2c_port;
      com_hdlr_ptr->config.speed_mode     = speed_mode;
      com_hdlr_ptr->config.slave_address  = port_hdlr_ptr->slave_addr;
      com_hdlr_ptr->addr                  = address;

      svc_i2c_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_i2c_handler->svc_mcu_item_handler.part);

      gpOS_semaphore_signal( com_hdlr_ptr->access_sem);
    }
    else
    {
      gpOS_semaphore_delete( com_hdlr_ptr->access_sem);
      gpOS_memory_deallocate_p( svc_i2c_handler->svc_mcu_item_handler.part, com_hdlr_ptr);
    }
  }

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);
#endif

  com_hdlr_ptr = svc_i2c_create_com_speed( i2c_port, speed_mode, (tU32)speed_mode, address);
  return com_hdlr_ptr;
  //return (svc_i2c_com_handler_t*)svc_i2c_create_com_speed( i2c_port, speed_mode, (tU32)speed_mode, address);
}

/********************************************//**
 * \brief Write buffer on I2C COM
 *
 * \param i2c_com_hdlr    COM handler pointer
 * \param addr_start      peripheral start address
 * \param addr_bytes      number of bytes of start address
 * \param out_buf         pointer to buffer to write
 * \param len             bytes to write
 * \param bytes_in_a_row  bytes to send during a single write operation
 * \param timeout         timeout
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_i2c_write( svc_i2c_com_handler_t *i2c_com_hdlr, tU32 addr_start, tU32 addr_bytes, tU8 *out_buf, tU32 len, tU32 bytes_in_a_row, gpOS_clock_t *timeout)
{
  gpOS_error_t error = gpOS_SUCCESS;
  boolean_t per_found = FALSE;

  if( (len == 0) || ( i2c_com_hdlr == NULL))
  {
    return gpOS_FAILURE;
  }
  else
  {
    svc_i2c_port_handler_t *port_hdlr_ptr;
    LLD_I2C_IdTy i2c_id;
    tU32 written_bytes = 0;
    tU32 curr_bytes_to_write;

    /**< Access COM handler */
    gpOS_semaphore_wait( i2c_com_hdlr->access_sem);

    port_hdlr_ptr = svc_i2c_get_hdlr_ptr( i2c_com_hdlr->port_id);
    i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, port_hdlr_ptr->port_id);

    /**< Access Port handler */
    gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

    svc_pwr_peripherallock_acquire( port_hdlr_ptr->write_peripherallockid);

    /**< Check if port can be used */
    if( (LLD_I2C_GetOperatingMode( i2c_id) == LLD_I2C_BUSCTRLMODE_MASTER) &&
        ( LLD_I2C_GetControllerStatus( i2c_id) != LLD_I2C_CTRLSTATUS_ABORT)
      )
    {
      /**< If previous COM using the port was different, update I2C configuration */
      if( port_hdlr_ptr->curr_com_ptr != i2c_com_hdlr)
      {
        LLD_I2C_ResetReg( i2c_id);
        LLD_I2C_Config( i2c_id, &i2c_com_hdlr->config);
        LLD_I2C_SetOperatingMode( i2c_id, LLD_I2C_BUSCTRLMODE_MASTER);

        port_hdlr_ptr->curr_com_ptr   = i2c_com_hdlr;
      }

      /**< Loop until all bytes are written or an error occurs */
      while( (written_bytes < len) && (error != gpOS_FAILURE))
      {
        boolean_t writing = TRUE;

        /**< Configure transmission on port */
        if( (len - written_bytes) > bytes_in_a_row)
        {
          curr_bytes_to_write = bytes_in_a_row;
        }
        else
        {
          curr_bytes_to_write = len - written_bytes;
        }

        port_hdlr_ptr->curr_op        = SVC_I2C_OPERATION_WRITE;
        port_hdlr_ptr->buf_ptr        = &out_buf[written_bytes];
        port_hdlr_ptr->buf_len        = curr_bytes_to_write;
        port_hdlr_ptr->buf_pos        = 0;

        /**<
          Try to start writing. If a NACK_ADDR occurs at first transmission, returns an error
          otherwise restart a write after a timeout until no error occurs
        */
        while( writing == TRUE)
        {
          gpOS_clock_t tnow, twait;
          LLD_I2C_CtrlStatusTy status;
          LLD_I2C_AbortCauseTy abort_cause;

          gpOS_kernel_lock();

          tnow = gpOS_time_now();
          if( (LLD_I2C_SpeedModeTy)port_hdlr_ptr->curr_com_ptr->config.speed_mode == (LLD_I2C_SpeedModeTy)LLD_I2C_STANDARD_MODE)
          {
            twait = gpOS_time_plus( tnow, ((tUInt)SVC_I2C_BAUDRATE_FACTOR_STD*(tUInt)SVC_I2C_SLVADDR_TICKS * (tUInt)gpOS_timer_ticks_per_sec())/(tUInt)port_hdlr_ptr->curr_com_ptr->config.speed_mode);
          }
          else if( (LLD_I2C_SpeedModeTy)port_hdlr_ptr->curr_com_ptr->config.speed_mode == (LLD_I2C_SpeedModeTy)LLD_I2C_FAST_MODE)
          {
            twait = gpOS_time_plus( tnow, ((tUInt)SVC_I2C_BAUDRATE_FACTOR_FAST*(tUInt)SVC_I2C_SLVADDR_TICKS * (tUInt)gpOS_timer_ticks_per_sec())/(tUInt)port_hdlr_ptr->curr_com_ptr->config.speed_mode);
          }
          else if ( (LLD_I2C_SpeedModeTy)port_hdlr_ptr->curr_com_ptr->config.speed_mode == (LLD_I2C_SpeedModeTy)LLD_I2C_HIGH_SPEED_MODE)
          {
            twait = gpOS_time_plus( tnow, ((tUInt)SVC_I2C_BAUDRATE_FACTOR_HS*(tUInt)SVC_I2C_SLVADDR_TICKS * (tUInt)gpOS_timer_ticks_per_sec())/(tUInt)SVC_I2C_MAX_OUT_FREQ);
          }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */

          port_hdlr_ptr->buf_len        = curr_bytes_to_write;
          port_hdlr_ptr->buf_pos        = 0;

          LLD_I2C_ClearInterrupt( i2c_id, LLD_I2C_IRQ_ALL_CLEARABLE);

          LLD_I2C_SetMCR( i2c_id, LLD_I2C_MSTMODE_WRITE, i2c_com_hdlr->addr, LLD_I2C_STARTPROC_DISABLED, LLD_I2C_STOPCOND_STOP, curr_bytes_to_write + addr_bytes);
          LLD_I2C_Enable( i2c_id);

          /**< Send start address for write operation */
          if( addr_bytes == 4)
          {
            LLD_I2C_WriteFifo( i2c_id, &(((tU8 *)(&addr_start))[3]), 1);
            LLD_I2C_WriteFifo( i2c_id, &(((tU8 *)(&addr_start))[2]), 1);
          }
          if( addr_bytes >= 2)
          {
            LLD_I2C_WriteFifo( i2c_id, &(((tU8 *)(&addr_start))[1]), 1);
          }
          if( addr_bytes >= 1)
          {
            LLD_I2C_WriteFifo( i2c_id, &(((tU8 *)(&addr_start))[0]), 1);
          }

          /**< Send bytes to FIFO until it is full or no more bytes must be written*/
          port_hdlr_ptr->buf_pos +=  LLD_I2C_WriteFifo( i2c_id, &port_hdlr_ptr->buf_ptr[port_hdlr_ptr->buf_pos], port_hdlr_ptr->buf_len - port_hdlr_ptr->buf_pos);

          /**< Wait a delay depending on bus speed and check operation status */
          while( ((tS32)gpOS_time_minus( twait, gpOS_time_now())) > 0 );

          status = LLD_I2C_GetControllerStatus( i2c_id);
          abort_cause = LLD_I2C_GetAbortCause( i2c_id);

          gpOS_kernel_unlock();

          /**< If, after timeout, status is abort, check the cause */
          if( status == LLD_I2C_CTRLSTATUS_ABORT)
          {
            switch( abort_cause)
            {
              case LLD_I2C_ABORTCAUSE_NACK_ADDR:
                if( per_found == FALSE)
                {
                  /**< peripheral does not answer at all, not connected */
                  error = gpOS_FAILURE;
                  writing = FALSE;
                  break;
                }
                else
                {
                  /**< peripheral answered before, so it's busy. Retry. */
                  LLD_I2C_ResetReg( i2c_id);
                  LLD_I2C_Config( i2c_id, &i2c_com_hdlr->config);
                  LLD_I2C_FlushTX( i2c_id);
                  LLD_I2C_SetOperatingMode( i2c_id, LLD_I2C_BUSCTRLMODE_MASTER);
                }
                break;

              default:
                error = gpOS_FAILURE;
                writing = FALSE;
                break;
            }
          }
          /**< otherwise all is ok and wait for end of write */
          else
          {
            per_found = TRUE;

            LLD_I2C_InterruptEnable( i2c_id, SVC_I2C_TXIRQS | LLD_I2C_IRQ_BUS_ERROR | LLD_I2C_IRQ_MASTER_TRANSACTION_DONE);

            if( gpOS_semaphore_wait_timeout( port_hdlr_ptr->done_sem, timeout) == gpOS_FAILURE)
            {
              status = LLD_I2C_GetControllerStatus( i2c_id);
              abort_cause = LLD_I2C_GetAbortCause( i2c_id);

              if( status == LLD_I2C_CTRLSTATUS_ABORT)
              {
                if( (abort_cause == LLD_I2C_ABORTCAUSE_BERR_START) || (abort_cause == LLD_I2C_ABORTCAUSE_BERR_STOP))
                {
                  error = gpOS_FAILURE;
                }
                else
                {
                  /**< error not handled, lookup! */
                  while(1);
                }
              }
            }

            writing = FALSE;
          }
        }

        /**< Update write handler for next write */
        written_bytes += curr_bytes_to_write;
        addr_start += bytes_in_a_row;
      }

      /**< reset port handler */
      port_hdlr_ptr->buf_ptr        = NULL;
      port_hdlr_ptr->buf_len        = 0;
      port_hdlr_ptr->buf_pos        = 0;
    }
    else
    {
      error = gpOS_FAILURE;
    }

    svc_pwr_peripherallock_release( port_hdlr_ptr->write_peripherallockid);

    gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

    gpOS_semaphore_signal( i2c_com_hdlr->access_sem);
  }

  return error;
}

/********************************************//**
 * \brief Read buffer on I2C COM
 *
 * \param i2c_com_hdlr  COM handler pointer
 * \param addr_start    peripheral start address
 * \param addr_bytes    number of bytes of start address
 * \param in_buf        pointer to buffer to fill
 * \param len           bytes to receive
 * \param timeout       timeout
 * \return gpOS_SUCCESS if ok, gpOS_FAILURE otherwise
 *
 ***********************************************/
gpOS_error_t svc_i2c_read( svc_i2c_com_handler_t *i2c_com_hdlr, tU32 addr_start, tU32 addr_bytes, tU8 *in_buf, tU32 len, gpOS_clock_t *timeout)
{
  gpOS_error_t error = gpOS_SUCCESS;

  if( (len == 0) || (i2c_com_hdlr == NULL))
  {
    return gpOS_FAILURE;
  }
  else
  {
    svc_i2c_port_handler_t *port_hdlr_ptr;
    LLD_I2C_IdTy i2c_id;

    /**< Access COM handler */
    gpOS_semaphore_wait( i2c_com_hdlr->access_sem);

    port_hdlr_ptr = svc_i2c_get_hdlr_ptr( i2c_com_hdlr->port_id);
    i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, port_hdlr_ptr->port_id);

    /**< Access Port handler */
    gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

    svc_pwr_peripherallock_acquire( port_hdlr_ptr->read_peripherallockid);

    /**< Check if port can be used */
    if( (LLD_I2C_GetOperatingMode( i2c_id) == LLD_I2C_BUSCTRLMODE_MASTER) &&
        ( LLD_I2C_GetControllerStatus( i2c_id) != LLD_I2C_CTRLSTATUS_ABORT)
      )
    {
      /**< If previous COM using the port was different, update I2C configuration */
      if( port_hdlr_ptr->curr_com_ptr != i2c_com_hdlr)
      {
        LLD_I2C_ResetReg( i2c_id);
        LLD_I2C_Config( i2c_id, &i2c_com_hdlr->config);
        LLD_I2C_SetOperatingMode( i2c_id, LLD_I2C_BUSCTRLMODE_MASTER);

        port_hdlr_ptr->curr_com_ptr   = i2c_com_hdlr;
      }

      /**< Loop until all bytes are read */
      while( (port_hdlr_ptr->buf_len < len) && (error != gpOS_FAILURE))
      {
        LLD_I2C_CtrlStatusTy status;
        LLD_I2C_AbortCauseTy abort_cause;

        /**< Configure transmission on port */
        port_hdlr_ptr->curr_op        = SVC_I2C_OPERATION_READ;
        port_hdlr_ptr->buf_ptr        = &in_buf[0];
        port_hdlr_ptr->buf_len        = len;
        port_hdlr_ptr->buf_pos        = 0;

        /**< Send address if provided */
        if( addr_bytes > 0)
        {
          gpOS_clock_t tnow, twait;

          gpOS_kernel_lock();

          tnow = gpOS_time_now();
          if( (LLD_I2C_SpeedModeTy)port_hdlr_ptr->curr_com_ptr->config.speed_mode == (LLD_I2C_SpeedModeTy)LLD_I2C_STANDARD_MODE)
          {
            twait = gpOS_time_plus( tnow, ((tUInt)SVC_I2C_BAUDRATE_FACTOR_STD*(tUInt)SVC_I2C_SLVADDR_TICKS * (tUInt)gpOS_timer_ticks_per_sec())/(tUInt)port_hdlr_ptr->curr_com_ptr->config.speed_mode);
          }
          else if( (LLD_I2C_SpeedModeTy)port_hdlr_ptr->curr_com_ptr->config.speed_mode == (LLD_I2C_SpeedModeTy)LLD_I2C_FAST_MODE)
          {
            twait = gpOS_time_plus( tnow, ((tUInt)SVC_I2C_BAUDRATE_FACTOR_FAST*(tUInt)SVC_I2C_SLVADDR_TICKS * (tUInt)gpOS_timer_ticks_per_sec())/(tUInt)port_hdlr_ptr->curr_com_ptr->config.speed_mode);
          }
          else if ( (LLD_I2C_SpeedModeTy)port_hdlr_ptr->curr_com_ptr->config.speed_mode == (LLD_I2C_SpeedModeTy)LLD_I2C_HIGH_SPEED_MODE)
          {
            twait = gpOS_time_plus( tnow, ((tUInt)SVC_I2C_BAUDRATE_FACTOR_HS*(tUInt)SVC_I2C_SLVADDR_TICKS * (tUInt)gpOS_timer_ticks_per_sec())/(tUInt)SVC_I2C_MAX_OUT_FREQ);
          }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */


          LLD_I2C_SetMCR( i2c_id, LLD_I2C_MSTMODE_WRITE, i2c_com_hdlr->addr, LLD_I2C_STARTPROC_DISABLED, LLD_I2C_STOPCOND_REPEATEDSTART, addr_bytes);
          LLD_I2C_Enable( i2c_id);

          if( addr_bytes == 4)
          {
            LLD_I2C_WriteFifo( i2c_id, &(((tU8 *)(&addr_start))[3]), 1);
            LLD_I2C_WriteFifo( i2c_id, &(((tU8 *)(&addr_start))[2]), 1);
          }
          if( addr_bytes >= 2)
          {
            LLD_I2C_WriteFifo( i2c_id, &(((tU8 *)(&addr_start))[1]), 1);
          }
          if( addr_bytes >= 1)
          {
            LLD_I2C_WriteFifo( i2c_id, &(((tU8 *)(&addr_start))[0]), 1);
          }

          /**< Wait a delay depending on bus speed and check operation status */
          while( ((tS32)gpOS_time_minus( twait, gpOS_time_now())) > 0 );

          status = LLD_I2C_GetControllerStatus( i2c_id);
          abort_cause = LLD_I2C_GetAbortCause( i2c_id);

          gpOS_kernel_unlock();

          if( status == LLD_I2C_CTRLSTATUS_ABORT)
          {
            switch( abort_cause)
            {
              case LLD_I2C_ABORTCAUSE_NACK_ADDR:
                /**< peripheral does not answer at all, not ready or not connected */

                error = gpOS_FAILURE;
                break;

              default:
                error = gpOS_FAILURE;
                break;
            }
          }
        }

        if( error == gpOS_SUCCESS)
        {
          LLD_I2C_ClearInterrupt( i2c_id, LLD_I2C_IRQ_ALL_CLEARABLE);

          LLD_I2C_SetMCR( i2c_id, LLD_I2C_MSTMODE_READ, i2c_com_hdlr->addr, LLD_I2C_STARTPROC_DISABLED, LLD_I2C_STOPCOND_STOP, len);
          LLD_I2C_Enable( i2c_id);

          LLD_I2C_InterruptEnable( i2c_id, SVC_I2C_RXIRQS | LLD_I2C_IRQ_BUS_ERROR | LLD_I2C_IRQ_MASTER_TRANSACTION_DONE);

          if( gpOS_semaphore_wait_timeout( port_hdlr_ptr->done_sem, timeout) == gpOS_FAILURE)
          {
            status = LLD_I2C_GetControllerStatus( i2c_id);
            abort_cause = LLD_I2C_GetAbortCause( i2c_id);

            if( status == LLD_I2C_CTRLSTATUS_ABORT)
            {
              if( (abort_cause == LLD_I2C_ABORTCAUSE_BERR_START) || (abort_cause == LLD_I2C_ABORTCAUSE_BERR_STOP))
              {
                error = gpOS_FAILURE;
              }
              else
              {
                /**< error not handled, lookup! */
                while(1);
              }
            }
          }
        }
      }

      /**< reset port handler */
      port_hdlr_ptr->buf_ptr        = NULL;
      port_hdlr_ptr->buf_len        = 0;
      port_hdlr_ptr->buf_pos        = 0;
    }
    else
    {
      error = gpOS_FAILURE;
    }

    svc_pwr_peripherallock_release( port_hdlr_ptr->read_peripherallockid);

    gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

    gpOS_semaphore_signal( i2c_com_hdlr->access_sem);


  }

  return error;
}

/********************************************//**
 * \brief   Change I2C bus mode
 *
 * \param   i2c_port    port to change
 * \param   mode        new I2C mode
 * \return  gpOS_SUCCESS
 *
 ***********************************************/
gpOS_error_t svc_i2c_set_port_mode( tUInt i2c_port, LLD_I2C_BusCtrlModeTy mode)
{
  svc_i2c_port_handler_t *port_hdlr_ptr;
  LLD_I2C_IdTy i2c_id;

  port_hdlr_ptr = svc_i2c_get_hdlr_ptr( i2c_port);
  i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, port_hdlr_ptr->port_id);

  /**< Access Port handler */
  gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

  if( mode != LLD_I2C_GetOperatingMode( i2c_id) )
  {
    LLD_I2C_Disable( i2c_id);
    LLD_I2C_InterruptDisable( i2c_id, LLD_I2C_IRQ_ALL);
    LLD_I2C_ResetReg( i2c_id);
    LLD_I2C_FlushRX( i2c_id);
    LLD_I2C_FlushTX( i2c_id);

    if( mode == LLD_I2C_BUSCTRLMODE_MASTER)
    {
      /**< Switch to master mode (default is slave listening) */
      LLD_I2C_SetOperatingMode( i2c_id, LLD_I2C_BUSCTRLMODE_MASTER);
    }
    else if( mode == LLD_I2C_BUSCTRLMODE_SLAVE)
    {
      /**< switch I2C in slave mode */
      LLD_I2C_Config( i2c_id, &svc_i2c_basecfg);
      LLD_I2C_SetOperatingMode( i2c_id, LLD_I2C_BUSCTRLMODE_SLAVE);

      /**< enables interrupts to answer to requests... */
      LLD_I2C_Enable( i2c_id);
    }
  }

  /**< Release Port handler */
  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Reset I2C port
 *
 * \param   i2c_port    I2C device to reset
 * \return gpOS_SUCCESS
 *
 ***********************************************/
gpOS_error_t svc_i2c_reset_port( tUInt i2c_port)
{
  svc_i2c_port_handler_t *port_hdlr_ptr;
  LLD_I2C_IdTy i2c_id;
  LLD_I2C_BusCtrlModeTy curr_mode;

  port_hdlr_ptr = svc_i2c_get_hdlr_ptr( i2c_port);
  i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, port_hdlr_ptr->port_id);

  /**< Access Port handler */
  gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

  curr_mode = LLD_I2C_GetOperatingMode( i2c_id);

  LLD_I2C_Disable( i2c_id);
  LLD_I2C_InterruptDisable( i2c_id, LLD_I2C_IRQ_ALL);
  LLD_I2C_ResetReg( i2c_id);
  LLD_I2C_FlushRX( i2c_id);
  LLD_I2C_FlushTX( i2c_id);

  if( curr_mode == LLD_I2C_BUSCTRLMODE_MASTER)
  {
    /**< Switch to master mode (default is slave listening) */
    LLD_I2C_SetOperatingMode( i2c_id, LLD_I2C_BUSCTRLMODE_MASTER);
  }
  else if( curr_mode == LLD_I2C_BUSCTRLMODE_SLAVE)
  {
    /**< switch I2C in slave mode */
    LLD_I2C_Config( i2c_id, &svc_i2c_basecfg);
    LLD_I2C_SetSlaveAddr( i2c_id, port_hdlr_ptr->slave_addr);
    LLD_I2C_SetOperatingMode( i2c_id, LLD_I2C_BUSCTRLMODE_SLAVE);

    /**< enables interrupts to answer to requests... */
    LLD_I2C_Enable( i2c_id);
  }

  port_hdlr_ptr->curr_com_ptr = NULL;

  /**< Release Port handler */
  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Reset I2C device
 *
 * \param   i2c_port    I2C device to reset
 * \return gpOS_SUCCESS
 *
 ***********************************************/
gpOS_error_t svc_i2c_reset( tUInt i2c_port)
{
  tUInt i, j;
  boolean_t stop = FALSE;
  boolean_t reset = FALSE;
  boolean_t status_changed = FALSE;
  svc_i2c_port_handler_t *port_hdlr_ptr;
  LLD_I2C_IdTy i2c_id;
  tU32 SDAIN_val, SDAOUT_val, SCLIN_val, SCLOUT_val;
  gpOS_clock_t startTime = 0;
  gpOS_clock_t endTime = 0;
  tUInt timeout = 0;
  tU32 threshold;
  gpOS_error_t error = gpOS_SUCCESS;

  port_hdlr_ptr = svc_i2c_get_hdlr_ptr( i2c_port);
  i2c_id = (LLD_I2C_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_I2C, port_hdlr_ptr->port_id);  /*lint !e923 */

  LLD_I2C_EnableTM(i2c_id); // I2C put in Test Mode.

  i = 0U;
  j = 0U;

  threshold = 10U;
  startTime = gpOS_time_now();
  endTime = startTime;
  timeout = gpOS_time_minus(endTime, startTime) / gpOS_timer_ticks_per_msec();
  while ((reset == FALSE) && (timeout < threshold))
  {
    SDAIN_val = LLD_I2C_GetTMSDAIN(i2c_id);
    SDAOUT_val = LLD_I2C_GetTMSDAOUT(i2c_id);
    SCLIN_val = LLD_I2C_GetTMSCLIN(i2c_id);
    SCLOUT_val = LLD_I2C_GetTMSCLOUT(i2c_id);
    if((SDAIN_val == 0x1U) && (SDAOUT_val == 0x1U) && (stop == FALSE))
    {
      if((SCLIN_val == 0x0U) && (SCLOUT_val == 0x0U))   /* Slave can release the SDA line only if SCL line is LOW */
      {
        j = i;
        stop = TRUE;                                    /* Slave released the SDA line; it can be generated a stop condition to reset I2C */
        status_changed = TRUE;
      }
      else if ((SCLIN_val == 0x1U) && (SCLOUT_val == 0x1U))
      {
        LLD_I2C_DisableTM(i2c_id);
        reset = TRUE;
      }
    }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    if( stop == FALSE)
    {
      if((i & 0x1U) == 0U)
      {
        LLD_I2C_ClearTMSCLOUT(i2c_id);
      }
      else if((i & 0x1U) == 1U)
      {
        LLD_I2C_SetTMSCLOUT(i2c_id);
      }
    }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    else if( stop == TRUE)
    {
      if(i == (j + 1U))
      {
        LLD_I2C_SetTMSCLOUT(i2c_id);     /* NACK pulse clock */
      }
      else if( i == (j + 2U))
      {
        LLD_I2C_ClearTMSCLOUT(i2c_id);   /* NACK pulse clock */
      }
      else if(i == (j + 3U))
      {
        LLD_I2C_ClearTMSDAOUT(i2c_id);   /* stop condition */
        LLD_I2C_SetTMSCLOUT(i2c_id);
      }
      else if (i == (j + 4U))
      {
        LLD_I2C_SetTMSDAOUT(i2c_id);     /* stop condition */
        LLD_I2C_DisableTM(i2c_id);
        reset = TRUE;
      }
    }/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    i++;/*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
    gpOS_task_delay(100U * gpOS_timer_ticks_per_usec());

    endTime = gpOS_time_now();
    if ( status_changed == TRUE)
    {
      startTime = endTime;
    }
    timeout = gpOS_time_minus(endTime, startTime) / gpOS_timer_ticks_per_msec();
  }

  if (reset == TRUE)
  {
    error = gpOS_SUCCESS;
  }
  else
  {
   error = gpOS_FAILURE;
  }

  return error;
}
