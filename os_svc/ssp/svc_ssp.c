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
#include "clibs.h"

#include "lld_ssp.h"

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_ssp.h"
#include "svc_pwr.h"

#if defined( __STA8088__)
#include "mapping_sta8088.h"
#elif defined( __STA8090__)
#include "mapping_sta8090.h"
#endif

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_SSP_HANDLER_SIZE                   sizeof( svc_ssp_handler_t)
#define SVC_SSP_PORT_HANDLER_SIZE              sizeof( svc_ssp_port_handler_t)
#define SVC_SSP_COM_HANDLER_SIZE               sizeof( svc_ssp_com_handler_t)
#define SVC_SSP_CR0_REG_SPI_DEFAULT_VAL        0xC7
#define SVC_SSP_CR0_REG_MICROWIRE_DEFAULT_VAL  0x470027
#define SVC_SSP_CR1_REG_SPI_DEFAULT_VAL        0x908
#define SVC_SSP_CR1_REG_MICROWIRE_DEFAULT_VAL  0x908
#define SVC_SSP_CR0_REG_ADDRESS                (SSP_REG_START_ADDR + 0x00)
#define SVC_SSP_CR1_REG_ADDRESS                (SSP_REG_START_ADDR + 0x04)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Handler for specific SPI peripheral
 ***********************************************/
typedef struct svc_ssp_com_hooks_s
{
  svc_ssp_hook_t                 pre_cb;         /**< callback to use before any R/w operation */
  tVoid *                         pre_cb_param;   /**< parameter fro pre_cb */

  svc_ssp_hook_t                 post_cb;        /**< callback to use after any R/w operation */
  tVoid *                         post_cb_param;  /**< parameter fro post_cb */
} svc_ssp_com_hooks_t;

/********************************************//**
 * \brief SSP COM handler
 ***********************************************/
struct svc_ssp_com_handler_s
{
  svc_ssp_com_handler_t *     next;                   /**< Pointer to next COM in the queue */

  tUInt                       port_id;                /**< SSP port associated to the COM */
  gpOS_semaphore_t *            access_sem;             /**< access semaphore to com */

  svc_ssp_com_hooks_t         hooks;                  /**< Pre/post callbacks for specified COM */

  tU32                        out_clk;                /**< Output frequency of COM */
  tU8                         buf_size;               /**< Buffer item size of COM (8/16/32) */
  LLD_SSP_ConfigTy            config;                 /**< SSP configuration for COM */
};

/********************************************//**
 * \brief SSP Port handler
 ***********************************************/

typedef struct svc_ssp_port_handler_s svc_ssp_port_handler_t;

struct svc_ssp_port_handler_s
{
  svc_ssp_port_handler_t *    next;                   /**< Next SSP port */
  tUInt                       port_id;                /**< Port ID */

  svc_ssp_com_handler_t *     curr_com_ptr;           /**< COM currently using the port */

  gpOS_semaphore_t *            access_sem;             /**< access semaphore to port */

  gpOS_semaphore_t *            done_sem;               /**< semaphore to signal end of transfer */
  void *                      tx_buf_ptr;             /**< Pointer to TX buffer */
  void *                      rx_buf_ptr;             /**< Pointer to RX buffer */
  tU16                        tx_pos;                 /**< Position in TX buffer */
  tU16                        rx_pos;                 /**< Position in RX buffer */
  tU16                        len;                    /**< Length of I/O request */
  peripherallockid_t          peripherallockid;       /**< pwr status */
};

/********************************************//**
 * \brief SSP svc_mcu handler
 ***********************************************/
typedef struct svc_ssp_handler_s
{
  svc_mcu_item_t             svc_mcu_item_handler;

  svc_ssp_com_handler_t *    com_head;               /**< Linked list of COMs */
  svc_ssp_port_handler_t *   port_head;              /**< Linked list of ports */
} svc_ssp_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/**< Standard SSP_configuration for SPI protocol */
static const LLD_SSP_ConfigTy svc_ssp_basecfg_spi = {
  LLD_SSP_INTERFACE_MOTOROLA_SPI,
  LLD_SSP_MASTER,
  TRUE,
  { MIN_CPSDVR, MIN_SCR},
  LLD_SSP_RX_MSB_TX_MSB,
  LLD_SSP_DATA_BITS_8,
  LLD_SSP_RX_8_OR_MORE_ELEM,
  LLD_SSP_TX_8_OR_MORE_EMPTY_LOC,
  LLD_SSP_CLK_RISING_EDGE,
  LLD_SSP_CLK_POL_IDLE_HIGH,
  (LLD_SSP_MicrowireCtrlTy)0,
  (LLD_SSP_MicrowireWaitStatelTy)0,
  (LLD_SSP_DuplexTy)0,
  FALSE
};

/**< Standard SSP_configuration for NS MicroWire protocol */
static const LLD_SSP_ConfigTy svc_ssp_basecfg_microwire = {
  LLD_SSP_INTERFACE_NATIONAL_MICROWIRE,
  LLD_SSP_MASTER,
  TRUE,
  { MIN_CPSDVR, MIN_SCR},
  LLD_SSP_RX_MSB_TX_MSB,
  LLD_SSP_DATA_BITS_8,
  LLD_SSP_RX_8_OR_MORE_ELEM,
  LLD_SSP_TX_8_OR_MORE_EMPTY_LOC,
  (LLD_SSP_ClkPhaseTy)0,
  (LLD_SSP_ClkPolarityTy)0,
  LLD_SSP_BITS_8,
  LLD_SSP_MICROWIRE_WAIT_ZERO,
  LLD_SSP_MICROWIRE_CHANNEL_HALF_DUPLEX,
  FALSE
};

static svc_ssp_handler_t *svc_ssp_handler = NULL;
static tU32 SSP_CR0_reg_value = 0x0;
//static tU32 SSP_CR1_reg_value = 0x0;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static svc_ssp_port_handler_t *  svc_ssp_get_hdlr_ptr ( tUInt ssp_port);
static void                       svc_ssp_callback     ( svc_ssp_port_handler_t *);

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Extract a given size data from buffer
 *
 * \param buf_ptr Pointer to buffer
 * \param pos Position in buffer
 * \param fifo_item_size Size of each item in buffer
 * \return value from buffer in given fifo_item_size
 *
 ***********************************************/
static LLD_ISR_SSP tU32 svc_ssp_buf_read_data( void *buf_ptr, tU32 pos, tU32 fifo_item_size)
{
  tU32 data;

  if( fifo_item_size == 1)
  {
    data = ((tU8 *)buf_ptr)[pos];
  }
  else if( fifo_item_size == 2)
  {
    data = ((tU16 *)buf_ptr)[pos];
  }
  else if( fifo_item_size == 4)
  {
    data = ((tU32 *)buf_ptr)[pos];
  }
  else
  {
    data = 0;
  }

  return data;
}

/********************************************//**
 * \brief Set a given size data into a buffer
 *
 * \param buf_ptr Pointer to buffer
 * \param pos Position in buffer
 * \param data data to set
 * \param fifo_item_size Size of each item in buffer
 * \return void
 *
 ***********************************************/
static LLD_ISR_SSP void svc_ssp_buf_write_data( void *buf_ptr, tU32 pos, tU32 data, tU32 fifo_item_size)
{
  if( fifo_item_size == 1)
  {
    ((tU8 *)buf_ptr)[pos] = data;
  }
  else if( fifo_item_size == 2)
  {
    ((tU16 *)buf_ptr)[pos] = data;
  }
  else if( fifo_item_size == 4)
  {
    ((tU32 *)buf_ptr)[pos] = data;
  }
}

/********************************************//**
 * \brief   Get SSP handler for specific port
 *
 * \param   ssp_port  port of wanted SSP handler
 * \return  svc_ssp_port_handler_t *  pointer to SSP handler, or NULL if not open
 *
 ***********************************************/
static svc_ssp_port_handler_t *svc_ssp_get_hdlr_ptr( tUInt ssp_port)
{
  svc_ssp_port_handler_t *port_hdlr_ptr;

  port_hdlr_ptr = svc_ssp_handler->port_head;

  while( port_hdlr_ptr != NULL)
  {
    if( port_hdlr_ptr->port_id == ssp_port)
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
 * \brief SSP svc_mcu interrupt callback
 *
 * \param port_hdlr_ptr Port handler pointer
 * \return void
 *
 ***********************************************/
static LLD_ISR_SSP void svc_ssp_callback( svc_ssp_port_handler_t *port_hdlr_ptr)
{
  LLD_SSP_IdTy ssp_id = (LLD_SSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SSP, port_hdlr_ptr->port_id);
  svc_ssp_com_handler_t *com_hdlr_ptr = port_hdlr_ptr->curr_com_ptr;
  tU32 rx_pos = port_hdlr_ptr->rx_pos;
  tU32 tx_pos = port_hdlr_ptr->tx_pos;
  tU32 len = port_hdlr_ptr->len;
  tU32 tx_cnt = 0;
  tU32 data;

  LLD_SSP_IRQSrcTy irq_status = LLD_SSP_GetIRQSrc( ssp_id);

  //
  // Handler RX interrupts
  //

  if( irq_status & (LLD_SSP_IRQ_SRC_RECEIVE | LLD_SSP_IRQ_SRC_RECEIVE_TIMEOUT))
  {
    //
    // Save all available data from RX FIFO
    //
    while( LLD_SSP_IsRxFifoNotEmpty( ssp_id) && (rx_pos < len))
    {
      LLD_SSP_GetData( ssp_id, &data);

      if( port_hdlr_ptr->rx_buf_ptr != NULL)
      {
        svc_ssp_buf_write_data( port_hdlr_ptr->rx_buf_ptr, rx_pos, data, com_hdlr_ptr->buf_size);
      }

      rx_pos++;

      // If there is a remaining data to transmit, send it and this will then trig a new data in RX FIFO
      if ((LLD_SSP_IsTxFifoNotFull( ssp_id)) && (tx_pos < len))
      {
        data = svc_ssp_buf_read_data( port_hdlr_ptr->tx_buf_ptr, tx_pos++, com_hdlr_ptr->buf_size);
        LLD_SSP_SetData( ssp_id, data);
        tx_cnt++;
      }
    }

    port_hdlr_ptr->rx_pos = rx_pos;

    //
    // If all data were received, disable port and RX IRQ
    //
    if( port_hdlr_ptr->rx_pos == port_hdlr_ptr->len)
    {
      LLD_SSP_DisableIRQSrc( ssp_id, LLD_SSP_IRQ_SRC_RECEIVE | LLD_SSP_IRQ_SRC_RECEIVE_TIMEOUT);
      LLD_SSP_Disable( ssp_id);
      LLD_SSP_ResetClock( ssp_id);
      gpOS_semaphore_signal( port_hdlr_ptr->done_sem);
    }

    LLD_SSP_ClearIRQSrc( ssp_id, LLD_SSP_IRQ_SRC_RECEIVE | LLD_SSP_IRQ_SRC_RECEIVE_TIMEOUT);
  }

  //
  // Handler TX interrupts
  //
  if( LLD_SSP_GetIRQConfig( ssp_id) & LLD_SSP_IRQ_SRC_TRANSMIT)
  {
    //
    // Write all possible data to TX FIFO
    //
    while( (LLD_SSP_IsTxFifoNotFull( ssp_id)) && (tx_pos < len) && (tx_cnt < 32))
    {
      tU32 data = svc_ssp_buf_read_data( port_hdlr_ptr->tx_buf_ptr, tx_pos++, com_hdlr_ptr->buf_size);
      LLD_SSP_SetData( ssp_id, data);
      tx_cnt++;
    }

    port_hdlr_ptr->tx_pos = tx_pos;

    //
    // If all data where transmitted, disable TX interrupts
    //
    if( tx_pos == len)
    {
      LLD_SSP_DisableIRQSrc( ssp_id, LLD_SSP_IRQ_SRC_TRANSMIT);
    }

    LLD_SSP_ClearIRQSrc( ssp_id, LLD_SSP_IRQ_SRC_TRANSMIT);
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
static gpOS_error_t svc_ssp_cmdcallback_exec( svc_mcu_cmd_id_t cmd_id, void *param)
{
  svc_ssp_port_handler_t *curr_port;
  curr_port = svc_ssp_handler->port_head;

  if (cmd_id == SVC_MCU_CMD_ID_CHANGE_SPEED)
  {
    /* SSP sample rate shall be adjusted  to maintain current sample rate if possible  and to respect frequency limitation:
      - PCLK >= SSPCLKI ; SSPCLK<=SSPCLKI/2 in master mode
      - SSPCLK <= SSPCLKI/12 in  slave mode
      In slave mode, the SSP is reconfigured in the fastest possible sample rate that can be reached
      assuming the slowest clock input (1MHz). It is the responsibility of the SSP master to adapt
      its clock to this SSP sample rate. */

    while( curr_port != NULL)
    {
      tU32 new_freq;
      tU32 effective_out_clk;
      LLD_SSP_IdTy ssp_id = (LLD_SSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SSP, curr_port->port_id);

      // Updates all baudrates
      svc_mcu_busclk_get( svc_ssp_handler->svc_mcu_item_handler.bus_id, &new_freq);

      if (curr_port->curr_com_ptr != NULL)
      {
        LLD_SSP_ResolveClockFrequency( new_freq, curr_port->curr_com_ptr->out_clk, &effective_out_clk, &curr_port->curr_com_ptr->config.clk_freq);
      }

      curr_port = curr_port->next;
    }
  }
  else if (cmd_id == SVC_MCU_CMD_ID_SUSPEND_TRANSFER)
  {

    // If Master then stop all SSP tranfer else do nothing
    // Stop all UART ports
    while( curr_port != NULL)
    {
      LLD_SSP_IdTy ssp_id = (LLD_SSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SSP, curr_port->port_id);

      // Suspend transfer if Master
      if ((LLD_SSP_IsMaster(ssp_id)) && curr_port->curr_com_ptr != NULL)
      {
//        gpOS_semaphore_wait( curr_port->curr_com_ptr->access_sem);
//        gpOS_semaphore_wait( curr_port->access_sem);
      }
      curr_port = curr_port->next;
    }
  }
  else if (cmd_id == SVC_MCU_CMD_ID_RESTORE_TRANSFER)
  {
    // Restart all UART ports
    while( curr_port != NULL)
    {
      LLD_SSP_IdTy ssp_id = (LLD_SSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SSP, curr_port->port_id);

      // Restore transfer if Master
      if ((LLD_SSP_IsMaster(ssp_id)) && curr_port->curr_com_ptr != NULL)
      {
//        gpOS_semaphore_signal( curr_port->curr_com_ptr->access_sem);
//        gpOS_semaphore_signal( curr_port->access_sem);
      }
      curr_port = curr_port->next;
    }
  }
  return gpOS_SUCCESS;
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Initialize SSP svc_mcu
 *
 * \param partition Partition to use to allocate memory
 * \param bus_speed Bus speed
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_ssp_init( gpOS_partition_t *partition, tU32 bus_id)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_ssp_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_ssp_handler = gpOS_memory_allocate_p( partition, SVC_SSP_HANDLER_SIZE);

  if( svc_ssp_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  // Install service on main handler and fill physical info
  if( svc_mcu_install( SVC_MCU_PER_ID_SSP, bus_id, &svc_ssp_handler->svc_mcu_item_handler) == gpOS_FAILURE)
  {
    gpOS_memory_deallocate_p( partition, svc_ssp_handler);
    return gpOS_FAILURE;
  }

  // Fill specific fields
  svc_ssp_handler->svc_mcu_item_handler.part       = partition;
  svc_ssp_handler->svc_mcu_item_handler.mem_used   = mem_at_start - gpOS_memory_getheapfree_p( partition);
  svc_ssp_handler->svc_mcu_item_handler.cmdif     = svc_ssp_cmdcallback_exec;

  svc_ssp_handler->com_head    = NULL;
  svc_ssp_handler->port_head   = NULL;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Open SSP Port
 *
 * \param ssp_port Port number (hw specific)
 * \param irq_pri Interrupt priority of SSP svc_mcu
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_ssp_open_port( tUInt ssp_port, gpOS_interrupt_priority_t irq_pri)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_ssp_handler->svc_mcu_item_handler.part);
  svc_ssp_port_handler_t *last_port_hdlr_ptr, *port_hdlr_ptr;

  LLD_SSP_IdTy  ssp_phy_id        = (LLD_SSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SSP, ssp_port);
  VicLineTy     ssp_phy_irq_line  = (VicLineTy)svc_mcu_get_irq_line( SVC_MCU_PER_ID_SSP, ssp_port);

  if( svc_ssp_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Check if port was already open */
  last_port_hdlr_ptr = svc_ssp_handler->port_head;
  port_hdlr_ptr = svc_ssp_handler->port_head;

  while( port_hdlr_ptr != NULL)
  {
    if( port_hdlr_ptr->port_id == ssp_port)
    {
      return( gpOS_FAILURE);
    }
    else
    {
      last_port_hdlr_ptr = port_hdlr_ptr;
      port_hdlr_ptr = port_hdlr_ptr->next;
    }
  }

  port_hdlr_ptr = gpOS_memory_allocate_p( svc_ssp_handler->svc_mcu_item_handler.part, SVC_SSP_PORT_HANDLER_SIZE);

  if( port_hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;
  }

  port_hdlr_ptr->access_sem       = gpOS_semaphore_create_p( SEM_FIFO, svc_ssp_handler->svc_mcu_item_handler.part, 0);
  port_hdlr_ptr->done_sem         = gpOS_semaphore_create_p( SEM_FIFO, svc_ssp_handler->svc_mcu_item_handler.part, 0);

  if( (port_hdlr_ptr->access_sem == NULL) || (port_hdlr_ptr->done_sem == NULL))
  {
    gpOS_semaphore_delete( port_hdlr_ptr->done_sem);
    gpOS_semaphore_delete( port_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_ssp_handler->svc_mcu_item_handler.part, port_hdlr_ptr);
    return gpOS_FAILURE;
  }

  port_hdlr_ptr->port_id      = ssp_port;

  port_hdlr_ptr->curr_com_ptr = NULL;

  port_hdlr_ptr->tx_buf_ptr   = NULL;
  port_hdlr_ptr->tx_pos       = 0;
  port_hdlr_ptr->rx_buf_ptr   = NULL;
  port_hdlr_ptr->rx_pos       = 0;
  port_hdlr_ptr->len          = 0;
  port_hdlr_ptr->next         = NULL;

  svc_mcu_enable( SVC_MCU_PER_ID_SSP, ssp_port);

  LLD_SSP_Init( ssp_phy_id);
  LLD_SSP_Disable( ssp_phy_id);
  LLD_SSP_ResetClock( ssp_phy_id);
  LLD_SSP_Reset( ssp_phy_id);

  gpOS_interrupt_install( ssp_phy_irq_line, irq_pri, (gpOS_interrupt_callback_t)svc_ssp_callback, port_hdlr_ptr);
  gpOS_interrupt_enable( ssp_phy_irq_line);

  if( last_port_hdlr_ptr == NULL)
  {
    svc_ssp_handler->port_head = port_hdlr_ptr;
  }
  else
  {
    last_port_hdlr_ptr = port_hdlr_ptr;
  }

  svc_ssp_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_ssp_handler->svc_mcu_item_handler.part);
  svc_pwr_peripherallock_register(&port_hdlr_ptr->peripherallockid);

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Create COM on SSP port
 *
 * \param ssp_port SSP port used
 * \param ssp_mode Interface type
 * \param out_clk Out clock frequency
 * \param data_size Size of each data transmitted
 * \param pre_cb Callback called before any transfer
 * \param pre_cb_param Parameter passed to pre_cb
 * \param post_cb Callback called after any transfer
 * \param post_cb_param Parameter passed to post_cb
 * \return COM handler pointer
 *
 ***********************************************/
svc_ssp_com_handler_t *svc_ssp_create_com_config( tUInt ssp_port, LLD_SSP_InterfaceTy ssp_mode, svc_ssp_comconfig_t ssp_init_comconfig)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_ssp_handler->svc_mcu_item_handler.part);
  svc_ssp_com_handler_t *com_hdlr_ptr;
  svc_ssp_com_handler_t **last_com_hdlr_ptr_ptr;
  tU8 buf_size;
  tU32 bus_clk, effective_out_clk;
  const LLD_SSP_ConfigTy *ssp_config;

  //
  // Check if it's safe to create a new com
  //

  // Check if SSP svc_mcu was initialized
  if( svc_ssp_handler == NULL)
  {
    return NULL;
  }

  // check if specified port is available on platform
  if( ssp_port >= svc_ssp_handler->svc_mcu_item_handler.phy_item->number)
  {
    return NULL;
  }

  // check if handler for specified port was initialized
  if( svc_ssp_get_hdlr_ptr( ssp_port) == NULL)
  {
    return NULL;
  }

  // check if data size is ok and configure in/out buffer size
  if( (ssp_init_comconfig.data_size >= LLD_SSP_DATA_BITS_4) && (ssp_init_comconfig.data_size <= LLD_SSP_DATA_BITS_8))
  {
    buf_size = 1;
  }
  else if( ssp_init_comconfig.data_size <= LLD_SSP_DATA_BITS_16)
  {
    buf_size = 2;
  }
  else if( ssp_init_comconfig.data_size <= LLD_SSP_DATA_BITS_32)
  {
    buf_size = 4;
  }
  else
  {
    return NULL;
  }

  // check if SSP mode is supported
  switch( ssp_mode)
  {
    case LLD_SSP_INTERFACE_MOTOROLA_SPI:
      ssp_config = &svc_ssp_basecfg_spi;
      break;

    case LLD_SSP_INTERFACE_NATIONAL_MICROWIRE:
      ssp_config = &svc_ssp_basecfg_microwire;
      break;

    default:
      ssp_config = NULL;
      break;
  }

  if( ssp_config == NULL)
  {
    return NULL;
  }

  //
  // Allocate COM needed memory
  //

  // allocate memory for COM handler and exit if no space is available
  com_hdlr_ptr = gpOS_memory_allocate_p( svc_ssp_handler->svc_mcu_item_handler.part, SVC_SSP_COM_HANDLER_SIZE);

  if( com_hdlr_ptr == NULL)
  {
    return NULL;
  }

  // create access semaphore and exit if no space is available
  com_hdlr_ptr->access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_ssp_handler->svc_mcu_item_handler.part, 0);

  if( com_hdlr_ptr->access_sem == NULL)
  {
    gpOS_semaphore_delete( com_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_ssp_handler->svc_mcu_item_handler.part, com_hdlr_ptr);
    return NULL;
  }

  //
  // Enqueue new COM handler on COM queue for specified port
  //

  last_com_hdlr_ptr_ptr = &svc_ssp_handler->com_head;

  while( *last_com_hdlr_ptr_ptr != NULL)
  {
    last_com_hdlr_ptr_ptr = &(*last_com_hdlr_ptr_ptr)->next;
  }

  *last_com_hdlr_ptr_ptr = com_hdlr_ptr;

  com_hdlr_ptr->next    = NULL;

  //
  // Configure COM parameters
  //

  com_hdlr_ptr->port_id             = ssp_port;
  com_hdlr_ptr->hooks.pre_cb        = ssp_init_comconfig.pre_cb;
  com_hdlr_ptr->hooks.pre_cb_param  = ssp_init_comconfig.pre_cb_param;
  com_hdlr_ptr->hooks.post_cb       = ssp_init_comconfig.post_cb;
  com_hdlr_ptr->hooks.post_cb_param = ssp_init_comconfig.post_cb_param;
  com_hdlr_ptr->buf_size            = buf_size;

  //
  // Configure SSP configuration for specified COM
  //
  memcpy( &com_hdlr_ptr->config, ssp_config, sizeof( LLD_SSP_ConfigTy));
  com_hdlr_ptr->config.data_size  = ssp_init_comconfig.data_size;
  com_hdlr_ptr->out_clk           = ssp_init_comconfig.out_clk;

  svc_mcu_busclk_get( svc_ssp_handler->svc_mcu_item_handler.bus_id, &bus_clk);
  LLD_SSP_ResolveClockFrequency( bus_clk, ssp_init_comconfig.out_clk, &effective_out_clk, &com_hdlr_ptr->config.clk_freq);

  com_hdlr_ptr->config.clk_phase = ssp_init_comconfig.clk_phase;
  com_hdlr_ptr->config.clk_pol = ssp_init_comconfig.clk_pol;

  com_hdlr_ptr->config.ctrl_len = ssp_init_comconfig.ctrl_len;
  com_hdlr_ptr->config.wait_state = ssp_init_comconfig.wait_state;
  com_hdlr_ptr->config.duplex = ssp_init_comconfig.duplex;

  svc_ssp_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_ssp_handler->svc_mcu_item_handler.part);

  gpOS_semaphore_signal( com_hdlr_ptr->access_sem);

  return com_hdlr_ptr;
}

/********************************************//**
 * \brief Create COM on SSP port
 *
 * \param ssp_port SSP port used
 * \param ssp_mode Interface type
 * \param out_clk Out clock frequency
 * \param data_size Size of each data transmitted
 * \param pre_cb Callback called before any transfer
 * \param pre_cb_param Parameter passed to pre_cb
 * \param post_cb Callback called after any transfer
 * \param post_cb_param Parameter passed to post_cb
 * \return COM handler pointer
 *
 ***********************************************/
svc_ssp_com_handler_t *svc_ssp_create_com( tUInt ssp_port, LLD_SSP_InterfaceTy ssp_mode, tU32 out_clk, LLD_SSP_DataSizeTy data_size, svc_ssp_hook_t pre_cb, svc_ssp_hook_param_t pre_cb_param, svc_ssp_hook_t post_cb, svc_ssp_hook_param_t post_cb_param)
{
  svc_ssp_com_handler_t *com_hdlr_ptr;

  svc_ssp_comconfig_t ssp_cfg;

  ssp_cfg.out_clk = out_clk;
  ssp_cfg.data_size = data_size;
  if(ssp_mode == LLD_SSP_INTERFACE_NATIONAL_MICROWIRE)
  {
    ssp_cfg.clk_phase = LLD_SSP_CLK_FALLING_EDGE; //(LLD_SSP_ClkPhaseTy)0U;
    ssp_cfg.clk_pol = LLD_SSP_CLK_POL_IDLE_LOW;//(LLD_SSP_ClkPolarityTy)0U;
    ssp_cfg.ctrl_len = svc_ssp_basecfg_microwire.ctrl_len;
    ssp_cfg.wait_state = svc_ssp_basecfg_microwire.wait_state;
    ssp_cfg.duplex = svc_ssp_basecfg_microwire.duplex;
  }
  else if(ssp_mode == LLD_SSP_INTERFACE_MOTOROLA_SPI)
  {
    ssp_cfg.clk_phase = svc_ssp_basecfg_spi.clk_phase;
    ssp_cfg.clk_pol = svc_ssp_basecfg_spi.clk_pol;
    ssp_cfg.ctrl_len = (LLD_SSP_MicrowireCtrlTy)0x0U; /*lint !e9030 !e9034 */
    ssp_cfg.wait_state = LLD_SSP_MICROWIRE_WAIT_ZERO; //(LLD_SSP_MicrowireWaitStatelTy)0U;
    ssp_cfg.duplex = LLD_SSP_MICROWIRE_CHANNEL_FULL_DUPLEX; //(LLD_SSP_DuplexTy)0U;
  }
  ssp_cfg.pre_cb = pre_cb; /*lint !e9013: no 'else' at end of 'if ... else if' chain [MISRA 2012 Rule 15.7, required] */
  ssp_cfg.pre_cb_param = pre_cb_param;
  ssp_cfg.post_cb = post_cb;
  ssp_cfg.post_cb_param = post_cb_param;


  com_hdlr_ptr = svc_ssp_create_com_config(ssp_port, ssp_mode, ssp_cfg);
  return com_hdlr_ptr;
}
/********************************************//**
 * \brief Change protocol for given SSP peripheral
 *
 * \param ssp_com_hdlr COM handler pointer
 * \param ssp_mode Interface type
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_ssp_com_setmode( svc_ssp_com_handler_t *ssp_com_hdlr, LLD_SSP_InterfaceTy ssp_mode)
{
  const LLD_SSP_ConfigTy *ssp_config;

  if( ssp_com_hdlr == NULL)
  {
    return gpOS_FAILURE;
  }

  // check if SSP mode is supported
  switch( ssp_mode)
  {
    case LLD_SSP_INTERFACE_MOTOROLA_SPI:
      ssp_config = &svc_ssp_basecfg_spi;
      break;

    case LLD_SSP_INTERFACE_NATIONAL_MICROWIRE:
      ssp_config = &svc_ssp_basecfg_microwire;
      break;

    default:
      ssp_config = NULL;
      break;
  }

  if( ssp_config == NULL)
  {
    return gpOS_FAILURE;
  }

  //
  // Access COM handler
  //
  gpOS_semaphore_wait( ssp_com_hdlr->access_sem);

  if( ssp_com_hdlr->config.iface != ssp_mode)
  {
    //
    // Save custom setting from config
    //
    LLD_SSP_DataSizeTy data_size = ssp_com_hdlr->config.data_size;
    LLD_SSP_ClockParamsTy clock_cfg = ssp_com_hdlr->config.clk_freq;

    _clibs_memcpy( &ssp_com_hdlr->config, ssp_config, sizeof( LLD_SSP_ConfigTy));

    ssp_com_hdlr->config.data_size  = data_size;
    ssp_com_hdlr->config.clk_freq   = clock_cfg;

    //
    // Reset current port to reconfigure at next write
    //
    svc_ssp_handler->port_head->curr_com_ptr = NULL;
  }

  gpOS_semaphore_signal( ssp_com_hdlr->access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Get protocol for given SSP peripheral
 *
 * \param ssp_com_hdlr COM handler pointer
 * \param ssp_mode_ptr Pointer to interface type
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_ssp_com_getmode( svc_ssp_com_handler_t *ssp_com_hdlr, LLD_SSP_InterfaceTy *ssp_mode_ptr)
{
  if( ssp_com_hdlr == NULL)
  {
    return gpOS_FAILURE;
  }

  gpOS_semaphore_wait( ssp_com_hdlr->access_sem);

  *ssp_mode_ptr = ssp_com_hdlr->config.iface;

  gpOS_semaphore_signal( ssp_com_hdlr->access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Write buffer on SSP COM
 *
 * \param ssp_com_hdlr COM handler pointer
 * \param out_buf Pointer to buffer to transmit
 * \param len size of data to transfer
 * \param in_buf Pointer to buffer for receiving
 * \param timeout Timeout for accessing the resource
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_ssp_write( svc_ssp_com_handler_t *ssp_com_hdlr, tVoid *out_buf, tU32 len, tVoid *in_buf, gpOS_clock_t *timeout)
{
  svc_ssp_port_handler_t *port_hdlr_ptr;
  LLD_SSP_IdTy ssp_id;

  if( (len == 0) || (ssp_com_hdlr == NULL))
  {
    return gpOS_FAILURE;
  }

  if( (ssp_com_hdlr->config.iface == LLD_SSP_INTERFACE_NATIONAL_MICROWIRE) && (len > 2))
  {
    return gpOS_FAILURE;
  }

  //
  // Access COM handler
  //
  gpOS_semaphore_wait( ssp_com_hdlr->access_sem);

  port_hdlr_ptr = svc_ssp_get_hdlr_ptr( ssp_com_hdlr->port_id);
  ssp_id = (LLD_SSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_SSP, port_hdlr_ptr->port_id);

  //
  // Access Port handler
  //
  gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

  svc_pwr_peripherallock_acquire(port_hdlr_ptr->peripherallockid);

  //
  // If previous COM using the port was different, update SSP configuration
  //
  /*
  if( port_hdlr_ptr->curr_com_ptr != ssp_com_hdlr)
  {
    LLD_SSP_Reset( ssp_id);
    LLD_SSP_SetConfiguration( ssp_id, &ssp_com_hdlr->config);
  }
  */

  SSP_CR0_reg_value = ((ssp_com_hdlr->config.data_size & 0x1F) | ((ssp_com_hdlr->config.clk_phase << 7) & 0x80) | ((ssp_com_hdlr->config.clk_pol << 4) & 0x40));

  if( port_hdlr_ptr->curr_com_ptr == NULL)
  {
    LLD_SSP_Reset( ssp_id);
    LLD_SSP_SetConfiguration( ssp_id, &ssp_com_hdlr->config);
  }
  else
  {
    if( port_hdlr_ptr->curr_com_ptr != ssp_com_hdlr)
    {
      if((READ32(SVC_SSP_CR0_REG_ADDRESS) & 0xFFFF) != SSP_CR0_reg_value)
      {
            SET32_BIT(SVC_SSP_CR0_REG_ADDRESS, SSP_CR0_reg_value);
      }
    }
  }


  //
  // Configure transmission on port
  //
  port_hdlr_ptr->curr_com_ptr   = ssp_com_hdlr;
  port_hdlr_ptr->tx_buf_ptr     = out_buf;
  port_hdlr_ptr->rx_buf_ptr     = in_buf;
  port_hdlr_ptr->len            = len;
  port_hdlr_ptr->tx_pos         = 0;
  port_hdlr_ptr->rx_pos         = 0;

  LLD_SSP_SetClock( ssp_id, &ssp_com_hdlr->config);

  //
  // Execute callback to be used before transmission
  //
  if( ssp_com_hdlr->hooks.pre_cb != NULL)
  {
    ssp_com_hdlr->hooks.pre_cb(  ssp_com_hdlr->hooks.pre_cb_param);
  }

  //
  // Enable peripheral
  //
  LLD_SSP_EnableIRQSrc( ssp_id, LLD_SSP_IRQ_SRC_TRANSMIT | LLD_SSP_IRQ_SRC_RECEIVE | LLD_SSP_IRQ_SRC_RECEIVE_TIMEOUT);
  LLD_SSP_Enable( ssp_id);

  gpOS_semaphore_wait( port_hdlr_ptr->done_sem);

  //
  // Execute callback to be used after transmission
  //
  if( ssp_com_hdlr->hooks.post_cb != NULL)
  {
    ssp_com_hdlr->hooks.post_cb( ssp_com_hdlr->hooks.post_cb_param);
  }

  svc_pwr_peripherallock_release(port_hdlr_ptr->peripherallockid);

  LLD_SSP_DisableIRQSrc( ssp_id, LLD_SSP_IRQ_SRC_TRANSMIT | LLD_SSP_IRQ_SRC_RECEIVE);

  port_hdlr_ptr->tx_buf_ptr     = NULL;
  port_hdlr_ptr->rx_buf_ptr     = NULL;
  port_hdlr_ptr->len            = 0;
  port_hdlr_ptr->tx_pos         = 0;
  port_hdlr_ptr->rx_pos         = 0;

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  gpOS_semaphore_signal( ssp_com_hdlr->access_sem);

  return gpOS_SUCCESS;
}

