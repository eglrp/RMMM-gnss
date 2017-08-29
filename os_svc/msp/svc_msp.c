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

#include "lld_msp.h"

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_msp.h"
#include "svc_pwr.h"
#include "string.h"
#include "platform.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_MSP_HANDLER_SIZE        sizeof( svc_msp_handler_t)
#define SVC_MSP_PORT_HANDLER_SIZE   sizeof( svc_msp_port_handler_t)
#define SVC_MSP_COM_HANDLER_SIZE    sizeof( svc_msp_com_handler_t)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief Handler for specific SPI peripheral
 ***********************************************/
typedef struct svc_msp_com_hooks_s
{
  svc_msp_hook_t                 pre_cb;         /**< callback to use before any R/w operation */
  tVoid *                         pre_cb_param;   /**< parameter fro pre_cb */

  svc_msp_hook_t                 post_cb;        /**< callback to use after any R/w operation */
  tVoid *                         post_cb_param;  /**< parameter fro post_cb */
} svc_msp_com_hooks_t;

/********************************************//**
 * \brief SSP COM handler
 ***********************************************/
struct svc_msp_com_handler_s
{
  svc_msp_com_handler_t *     next;                   /**< Pointer to next COM in the queue */

  tUInt                       port_id;                /**< SSP port associated to the COM */
  gpOS_semaphore_t *            access_sem;             /**< access semaphore to com */

  svc_msp_com_hooks_t         hooks;                  /**< Pre/post callbacks for specified COM */

  tU32                        out_clk;                /**< Output frequency of COM */
  tU8                         buf_size;               /**< Buffer item size of COM (8/16/32) */
  LLD_MSP_ConfigTy            config;                 /**< MSP configuration for COM */
  LLD_MSP_ProtocolTy          protocol;               /**< MSP protocol configuration for COM */
};

/********************************************//**
 * \brief SSP Port handler
 ***********************************************/

typedef struct svc_msp_port_handler_s svc_msp_port_handler_t;

struct svc_msp_port_handler_s
{
  svc_msp_port_handler_t *    next;                   /**< Next SSP port */
  tUInt                       port_id;                /**< Port ID */

  svc_msp_com_handler_t *     curr_com_ptr;           /**< COM currently using the port */

  gpOS_semaphore_t *          access_sem;             /**< access semaphore to port */

  gpOS_semaphore_t *          done_sem;               /**< semaphore to signal end of transfer */
  void *                      tx_buf_ptr;             /**< Pointer to TX buffer */
  void *                      rx_buf_ptr;             /**< Pointer to RX buffer */
  tU32                        tx_pos;                 /**< Position in TX buffer */
  tU32                        rx_pos;                 /**< Position in RX buffer */
  tU32                        len;                    /**< Length of I/O request */
  svc_msp_protocol_t          msp_mode;
  peripherallockid_t          peripherallockid;       /**< pwr status */
};

/********************************************//**
 * \brief SSP svc_mcu handler
 ***********************************************/
typedef struct svc_msp_handler_s
{
  svc_mcu_item_t             svc_mcu_item_handler;

  svc_msp_com_handler_t *     com_head;               /**< Linked list of COMs */
  svc_msp_port_handler_t *    port_head;              /**< Linked list of ports */
} svc_msp_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/**< Standard MSP_configuration for SPI protocol */
static const LLD_MSP_ConfigTy svc_msp_basecfg_spi = {
  LLD_MSP_CLOCK_SEL_EXT,                  /* srg_clock_sel                 */
  LLD_MSP_CLOCK_POL_FALL,                 /* sck_pol                       */
  LLD_MSP_LOOPBACKMODE_DISABLE,           /* msp_loopback_mode             */
  LLD_MSP_DIRCOMPMODE_DISABLE,            /* msp_direct_companding_mode    */
  LLD_MSP_CLOCK_SEL_INT,                  /* rx_clock_sel                  */
  LLD_MSP_CLOCK_SEL_INT,                  /* tx_clock_sel                  */
  LLD_MSP_DMAMODE_OFF,                    /* rx_msp_dma_mode               */
  LLD_MSP_DMAMODE_OFF,                    /* tx_msp_dma_mode               */
  LLD_MSP_FRAME_SEL_GEN_LOGIC,            /* rx_frame_sync_sel             */
  LLD_MSP_FRAME_SEL_GEN_LOGIC,            /* tx_frame_sync_sel             */
  LLD_MSP_UNEXPFRAMESYNCMODE_IGNORED,     /* rx_unexpect_frame_sync        */
  LLD_MSP_UNEXPFRAMESYNCMODE_IGNORED,     /* tx_unexpect_frame_sync        */
  LLD_MSP_FIFOMODE_ENABLE,                /* rx_fifo_config                */
  LLD_MSP_FIFOMODE_ENABLE,                /* tx_fifo_config                */
  LLD_MSP_TXEXTRADELAYMODE_OFF            /* tx_extra_delay                */
};

static const LLD_MSP_ProtocolTy svc_msp_baseprotocol_spi = {
  LLD_MSP_PHASEMODE_SINGLE,               /* rx_phase_mode                 */
  LLD_MSP_PHASEMODE_SINGLE,               /* tx_phase_mode                 */
  LLD_MSP_2PHSTARTMODE_IMMEDIATE,         /* rx_phase2_start_mode          */
  LLD_MSP_2PHSTARTMODE_IMMEDIATE,         /* tx_phase2_start_mode          */
  LLD_MSP_ENDIANFORM_MSB_FIRST,           /* rx_endianess                  LLD_MSP_ENDIANFORM_MSB_FIRST,*/
  LLD_MSP_ENDIANFORM_MSB_FIRST,           /* tx_endianess                  LLD_MSP_ENDIANFORM_MSB_FIRST*/
  1,                                      /* rx_frame_length_1             */
  1,                                      /* rx_frame_length_2             */
  1,                                      /* tx_frame_length_1             */
  1,                                      /* tx_frame_length_2             */
  LLD_MSP_BITS4ELEM_8,                    /* rx_element_length_1           LLD_MSP_BITS4ELEM_8*/
  LLD_MSP_BITS4ELEM_8,                    /* rx_element_length_2           */
  LLD_MSP_BITS4ELEM_8,                    /* tx_element_length_1           */
  LLD_MSP_BITS4ELEM_8,                    /* tx_element_length_2           */
  LLD_MSP_DATADELAY_1_CLOCK,              /* rx_data_delay                 LLD_MSP_DATADELAY_1_CLOCK*/
  LLD_MSP_DATADELAY_1_CLOCK,              /* tx_data_delay                 LLD_MSP_DATADELAY_1_CLOCK*/
  LLD_MSP_CLOCK_POL_RISE,                 /* rx_clock_pol                  LLD_MSP_CLOCK_POL_RISE,*/
  LLD_MSP_CLOCK_POL_RISE,                 /* tx_clock_pol                  LLD_MSP_CLOCK_POL_RISE*/
  LLD_MSP_FRAME_SYNC_POL_LOW,             /* rx_msp_frame_pol              */
  LLD_MSP_FRAME_SYNC_POL_LOW,             /* tx_msp_frame_pol              */
  LLD_MSP_HALFWORDSWAP_NONE,              /* rx_half_word_swap             */
  LLD_MSP_HALFWORDSWAP_NONE,              /* tx_half_word_swap             */
  LLD_MSP_DATATYPE_NOCOMPANDING,          /* compression_mode              */
  LLD_MSP_DATATYPE_NOCOMPANDING,          /* expansion_mode                */
  LLD_MSP_SPICLOCKMDODE_ZERO_DELAY,       /* spi_clk_mode                  */
  LLD_MSP_SPIBURSTMODE_ENABLE,            /* spi_burst_mode                */
  7,                                      /* frame_period                  */
  0,                                      /* frame_width                   */
};

/**< Standard MSP_configuration for I2S protocol */
static const LLD_MSP_ConfigTy svc_msp_basecfg_i2s = {
  LLD_MSP_CLOCK_SEL_EXT,                  /* srg_clock_sel                 */
  LLD_MSP_CLOCK_POL_RISE,                 /* sck_pol                       */
  LLD_MSP_LOOPBACKMODE_DISABLE,           /* msp_loopback_mode             */
  LLD_MSP_DIRCOMPMODE_DISABLE,            /* msp_direct_companding_mode    */
  LLD_MSP_CLOCK_SEL_INT,                  /* rx_clock_sel                  */
  LLD_MSP_CLOCK_SEL_INT,                  /* tx_clock_sel                  */
  LLD_MSP_DMAMODE_OFF,                    /* rx_msp_dma_mode               */
  LLD_MSP_DMAMODE_OFF,                    /* tx_msp_dma_mode               */
  LLD_MSP_FRAME_SEL_EXTERNAL,             /* rx_frame_sync_sel             */
  LLD_MSP_FRAME_SEL_GEN_LOGIC_PERIOD,     /* tx_frame_sync_sel             */
  LLD_MSP_UNEXPFRAMESYNCMODE_IGNORED,     /* rx_unexpect_frame_sync        */
  LLD_MSP_UNEXPFRAMESYNCMODE_IGNORED,     /* tx_unexpect_frame_sync        */
  LLD_MSP_FIFOMODE_DISABLE,               /* rx_fifo_config                */
  LLD_MSP_FIFOMODE_ENABLE,                /* tx_fifo_config                */
  LLD_MSP_TXEXTRADELAYMODE_OFF            /* tx_extra_delay                */
};

static const LLD_MSP_ProtocolTy svc_msp_baseprotocol_i2s = {
  LLD_MSP_PHASEMODE_SINGLE,               /* rx_phase_mode                 */
  LLD_MSP_PHASEMODE_SINGLE,               /* tx_phase_mode                 */
  LLD_MSP_2PHSTARTMODE_IMMEDIATE,         /* rx_phase2_start_mode          */
  LLD_MSP_2PHSTARTMODE_IMMEDIATE,         /* tx_phase2_start_mode          */
  LLD_MSP_ENDIANFORM_MSB_FIRST,           /* rx_endianess                  */
  LLD_MSP_ENDIANFORM_MSB_FIRST,           /* tx_endianess                  */
  1,                                      /* rx_frame_length_1             */
  1,                                      /* rx_frame_length_2             */
  2,                                      /* tx_frame_length_1             */
  1,                                      /* tx_frame_length_2             */
  LLD_MSP_BITS4ELEM_32,                   /* rx_element_length_1           */
  LLD_MSP_BITS4ELEM_32,                   /* rx_element_length_2           */
  LLD_MSP_BITS4ELEM_32,                   /* tx_element_length_1           */
  LLD_MSP_BITS4ELEM_32,                   /* tx_element_length_2           */
  LLD_MSP_DATADELAY_0_CLOCK,              /* rx_data_delay                 */
  LLD_MSP_DATADELAY_0_CLOCK,              /* tx_data_delay                 */
  LLD_MSP_CLOCK_POL_RISE,                 /* rx_clock_pol                  */
  LLD_MSP_CLOCK_POL_FALL,                 /* tx_clock_pol                  */
  LLD_MSP_FRAME_SYNC_POL_HIGH,            /* rx_msp_frame_pol              */
  LLD_MSP_FRAME_SYNC_POL_HIGH,            /* tx_msp_frame_pol              */
  LLD_MSP_HALFWORDSWAP_NONE,              /* rx_half_word_swap             */
  LLD_MSP_HALFWORDSWAP_HALF_SWAP,         /* tx_half_word_swap             */
  LLD_MSP_DATATYPE_NOCOMPANDING,          /* compression_mode              */
  LLD_MSP_DATATYPE_NOCOMPANDING,          /* expansion_mode                */
  LLD_MSP_SPICLOCKMDODE_NOCLOCK,          /* spi_clk_mode                  */
  LLD_MSP_SPIBURSTMODE_DISABLE,           /* spi_burst_mode                */
  31,                                     /* frame_period                  */
  15,                                     /* frame_width                   */
};

static svc_msp_handler_t *svc_msp_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static svc_msp_port_handler_t *  svc_msp_get_hdlr_ptr ( tUInt msp_port);
static void                       svc_msp_callback     ( svc_msp_port_handler_t *);

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
static LLD_ISR_MSP tU32 svc_msp_buf_read_data( void *buf_ptr, tU32 pos, tU32 fifo_item_size)
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
static LLD_ISR_MSP void svc_msp_buf_write_data( void *buf_ptr, tU32 pos, tU32 data, tU32 fifo_item_size)
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
 * \brief   Get MSP handler for specific port
 *
 * \param   msp_port  port of wanted MSP handler
 * \return  svc_msp_port_handler_t *  pointer to MSP handler, or NULL if not open
 *
 ***********************************************/
static svc_msp_port_handler_t *svc_msp_get_hdlr_ptr( tUInt msp_port)
{
  svc_msp_port_handler_t *port_hdlr_ptr;

  port_hdlr_ptr = svc_msp_handler->port_head;

  while( port_hdlr_ptr != NULL)
  {
    if( port_hdlr_ptr->port_id ==  msp_port)
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
 * \brief MSP svc_mcu interrupt callback
 *
 * \param port_hdlr_ptr Port handler pointer
 * \return void
 *
 ***********************************************/
static LLD_ISR_MSP void svc_msp_callback( svc_msp_port_handler_t *port_hdlr_ptr)
{
  LLD_MSP_IdTy msp_id = (LLD_MSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_MSP, port_hdlr_ptr->port_id);
  svc_msp_com_handler_t *com_hdlr_ptr = port_hdlr_ptr->curr_com_ptr;

  LLD_MSP_IRQSrcTy irq_status = LLD_MSP_GetIRQStatus( msp_id);

  if( port_hdlr_ptr->msp_mode == SVC_MSP_PROTOCOL_SPI)
  {
    tU32 rx_pos = port_hdlr_ptr->rx_pos;
    tU32 tx_pos = port_hdlr_ptr->tx_pos;
    tU32 len = port_hdlr_ptr->len;
    tU32 data;
    tU32 tx_cnt = 0;

    //
    // Handle RX interrupts
    //
    if( (irq_status & (LLD_MSP_IRQ_RECEIVE | LLD_MSP_IRQ_RECEIVE_FIFO_NOT_EMPTY)) || LLD_MSP_GetFlagStatus( msp_id, LLD_MSP_STATUSFLAG_RECEIVE_FIFOFULL))
    {
      while( !LLD_MSP_GetFlagStatus( msp_id, LLD_MSP_STATUSFLAG_RECEIVE_FIFOEMPTY) && (rx_pos < len))
      {
        data = LLD_MSP_ReadData( msp_id);

        if( port_hdlr_ptr->rx_buf_ptr != NULL)
        {
          svc_msp_buf_write_data( port_hdlr_ptr->rx_buf_ptr, rx_pos++, data, com_hdlr_ptr->buf_size);
        }

        // If there is a remaining data to transmit, send it and this will then trig a new data in RX FIFO
        if( !LLD_MSP_GetFlagStatus( msp_id, LLD_MSP_STATUSFLAG_TRANSMIT_FIFOFULL) && (tx_pos < len) && (tx_cnt < LLD_MSP_FIFOSIZE))
        {
          data = svc_msp_buf_read_data( port_hdlr_ptr->tx_buf_ptr, tx_pos++, com_hdlr_ptr->buf_size);
          LLD_MSP_WriteData( msp_id, data);
          tx_cnt++;
        }
      }

      port_hdlr_ptr->rx_pos = rx_pos;

      if( rx_pos == len)
      {
        LLD_MSP_IRQDisable( msp_id, LLD_MSP_IRQ_RECEIVE | LLD_MSP_IRQ_RECEIVE_FIFO_NOT_EMPTY);

        data = LLD_MSP_ReadData( msp_id);

        if( port_hdlr_ptr->rx_buf_ptr != NULL)
        {
          svc_msp_buf_write_data( port_hdlr_ptr->rx_buf_ptr, port_hdlr_ptr->rx_pos++, data, com_hdlr_ptr->buf_size);
        }

        gpOS_semaphore_signal( port_hdlr_ptr->done_sem);
      }

      LLD_MSP_IRQClear( msp_id, irq_status & (LLD_MSP_IRQ_RECEIVE | LLD_MSP_IRQ_RECEIVE_FIFO_NOT_EMPTY));
    }

    //
    // Handle TX interrupt
    // If all data where transmitted and Transmit is on, disable TX interrupts
    //
    if( LLD_MSP_GetIRQConfig( msp_id) & LLD_MSP_IRQ_TRANSMIT)
    {
      while( !LLD_MSP_GetFlagStatus( msp_id, LLD_MSP_STATUSFLAG_TRANSMIT_FIFOFULL) && (tx_pos < len) && (tx_cnt < LLD_MSP_FIFOSIZE))
      {
        tU32 data = svc_msp_buf_read_data( port_hdlr_ptr->tx_buf_ptr, tx_pos++, com_hdlr_ptr->buf_size);
        LLD_MSP_WriteData( msp_id, data);
        tx_cnt++;
      }

      if( tx_pos == len)
      {
        LLD_MSP_IRQDisable( msp_id, LLD_MSP_IRQ_TRANSMIT | LLD_MSP_IRQ_RECEIVE);
        LLD_MSP_IRQEnable( msp_id,  LLD_MSP_IRQ_RECEIVE_FIFO_NOT_EMPTY);
      }

      LLD_MSP_IRQClear( msp_id, LLD_MSP_IRQ_TRANSMIT);
    }

    port_hdlr_ptr->tx_pos = tx_pos;
  }

  if(port_hdlr_ptr->msp_mode == SVC_MSP_PROTOCOL_I2S)
  {
    tU32 tx_pos = port_hdlr_ptr->tx_pos;
    tU32 len = port_hdlr_ptr->len;

    //
    // Handle TX interrupts
    //
    while( !LLD_MSP_GetFlagStatus( msp_id, LLD_MSP_STATUSFLAG_TRANSMIT_FIFOFULL) && (tx_pos < len))
    {
      tU32 data = svc_msp_buf_read_data( port_hdlr_ptr->tx_buf_ptr, tx_pos++, com_hdlr_ptr->buf_size);
      LLD_MSP_WriteData( msp_id, data);
    }

    port_hdlr_ptr->tx_pos = tx_pos;

    if( tx_pos == len)
    {
      LLD_MSP_IRQDisable( msp_id, LLD_MSP_IRQ_TRANSMIT);
      gpOS_semaphore_signal( port_hdlr_ptr->done_sem);
    }

    LLD_MSP_IRQClear( msp_id, LLD_MSP_IRQ_TRANSMIT);
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
static gpOS_error_t svc_msp_cmdcallback_exec( svc_mcu_cmd_id_t cmd_id, void *param)
  {
  svc_msp_port_handler_t *curr_port;
  curr_port = svc_msp_handler->port_head;

  if (cmd_id == SVC_MCU_CMD_ID_CHANGE_SPEED)
    {
    /* MSP sample rate generator shall be adjusted to maintain sample rate if possible and to respect frequency limitation:
      - AHB_CLK >= 2x MSP(T/R)CK in MSP mode
      - GCLK >= 8x data rate in SPI mode
      MSP enable sequence timing shall be ensured even at different frequencies.
      In slave mode, the MSP is reconfigured in the fastest possible frequency that can be reached assuming the slowest clock input (1MHz).
      It is the responsibility of the MSP master to adapt its clock to this MSP frequency. */

    while( curr_port != NULL)
      {
      LLD_MSP_IdTy  msp_phy_id        = (LLD_MSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_MSP, curr_port->port_id);

      {
        tU32 bus_speed;

        LLD_MSP_DisableTxRx(msp_phy_id);

        LLD_MSP_EmptyTxFifo( msp_phy_id);
        LLD_MSP_EmptyRxFifo( msp_phy_id);

        LLD_MSP_ResetReg( msp_phy_id);

        svc_mcu_busclk_get( svc_msp_handler->svc_mcu_item_handler.bus_id, &bus_speed);

        if( curr_port->msp_mode == SVC_MSP_PROTOCOL_SPI)
        {
          LLD_MSP_SetSPIClockMode(msp_phy_id, LLD_MSP_SPICLOCKMDODE_NOCLOCK);
          LLD_MSP_Configure(msp_phy_id, &curr_port->curr_com_ptr->config, &curr_port->curr_com_ptr->protocol);
          LLD_MSP_ConfigSampleRateGen( msp_phy_id, bus_speed, curr_port->curr_com_ptr->out_clk, 8, 0);
          LLD_MSP_SetSPIClockMode(msp_phy_id, LLD_MSP_SPICLOCKMDODE_ZERO_DELAY);
      }

      if( curr_port->msp_mode == SVC_MSP_PROTOCOL_I2S)
      {
        LLD_MSP_Configure(msp_phy_id, &curr_port->curr_com_ptr->config, &curr_port->curr_com_ptr->protocol);
        LLD_MSP_ConfigSampleRateGen(msp_phy_id, bus_speed, curr_port->curr_com_ptr->out_clk, 32,16);
      }
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
      LLD_MSP_IdTy  msp_phy_id        = (LLD_MSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_MSP, curr_port->port_id);

      // Suspend transfer if Master
      if (LLD_MSP_GetTxClock(msp_phy_id) == 1)
    {
        gpOS_semaphore_wait( curr_port->access_sem);
      }

      curr_port = curr_port->next;
    }
  }
  else if (cmd_id == SVC_MCU_CMD_ID_RESTORE_TRANSFER)
  {
    // Restart all UART ports
    while( curr_port != NULL)
    {
      LLD_MSP_IdTy  msp_phy_id        = (LLD_MSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_MSP, curr_port->port_id);

      // Restore transfer if Master
      if (LLD_MSP_GetTxClock(msp_phy_id) == 1)
      {
        gpOS_semaphore_signal( curr_port->access_sem);
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
 * \brief Initialize MSP svc_mcu
 *
 * \param partition Partition to use to allocate memory
 * \param bus_speed Bus speed
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_msp_init( gpOS_partition_t *partition, tU32 bus_speed)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_msp_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_msp_handler = gpOS_memory_allocate_p( partition, SVC_MSP_HANDLER_SIZE);

  if( svc_msp_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  // Install service on main handler and fill physical info
  if( svc_mcu_install( SVC_MCU_PER_ID_MSP, PLATFORM_BUSCLK_ID_MCLK, &svc_msp_handler->svc_mcu_item_handler) == gpOS_FAILURE)
  {
    gpOS_memory_deallocate_p( partition, svc_msp_handler);
    return gpOS_FAILURE;
  }

  // Fill specific fields
  svc_msp_handler->svc_mcu_item_handler.part   = partition;
  svc_msp_handler->svc_mcu_item_handler.mem_used    = mem_at_start - gpOS_memory_getheapfree_p( partition);

  svc_msp_handler->com_head    = NULL;
  svc_msp_handler->port_head   = NULL;

  svc_msp_handler->svc_mcu_item_handler.cmdif =  svc_msp_cmdcallback_exec;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Open MSP Port
 *
 * \param msp_port Port number (hw specific)
 * \param irq_pri Interrupt priority of MSP svc_mcu
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_msp_open_port( tUInt msp_port, gpOS_interrupt_priority_t irq_pri)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_msp_handler->svc_mcu_item_handler.part);
  svc_msp_port_handler_t *last_port_hdlr_ptr, *port_hdlr_ptr;

  LLD_MSP_IdTy  msp_phy_id        = (LLD_MSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_MSP, msp_port);
  VicLineTy     msp_phy_irq_line  = (VicLineTy)svc_mcu_get_irq_line( SVC_MCU_PER_ID_MSP, msp_port);

  if( svc_msp_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  /**< Check if port was already open */
  last_port_hdlr_ptr = svc_msp_handler->port_head;
  port_hdlr_ptr = svc_msp_handler->port_head;

  while( port_hdlr_ptr != NULL)
  {
    if( port_hdlr_ptr->port_id == msp_port)
    {
      return( gpOS_FAILURE);
    }
    else
    {
      last_port_hdlr_ptr = port_hdlr_ptr;
      port_hdlr_ptr = port_hdlr_ptr->next;
    }
  }

  port_hdlr_ptr = gpOS_memory_allocate_p( svc_msp_handler->svc_mcu_item_handler.part, SVC_MSP_PORT_HANDLER_SIZE);

  if( port_hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;
  }

  port_hdlr_ptr->access_sem       = gpOS_semaphore_create_p( SEM_FIFO, svc_msp_handler->svc_mcu_item_handler.part, 0);
  port_hdlr_ptr->done_sem         = gpOS_semaphore_create_p( SEM_FIFO, svc_msp_handler->svc_mcu_item_handler.part, 0);

  if( (port_hdlr_ptr->access_sem == NULL) || (port_hdlr_ptr->done_sem == NULL))
  {
    gpOS_semaphore_delete( port_hdlr_ptr->done_sem);
    gpOS_semaphore_delete( port_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_msp_handler->svc_mcu_item_handler.part, port_hdlr_ptr);
    return gpOS_FAILURE;
  }

  port_hdlr_ptr->port_id      = msp_port;

  port_hdlr_ptr->curr_com_ptr = NULL;

  port_hdlr_ptr->tx_buf_ptr   = NULL;
  port_hdlr_ptr->tx_pos       = 0;
  port_hdlr_ptr->rx_buf_ptr   = NULL;
  port_hdlr_ptr->rx_pos       = 0;
  port_hdlr_ptr->len          = 0;
  port_hdlr_ptr->next         = NULL;

  svc_mcu_enable( SVC_MCU_PER_ID_MSP, msp_port);

  LLD_MSP_DisableTxRx( msp_phy_id);

  LLD_MSP_EmptyTxFifo( msp_phy_id);
  LLD_MSP_EmptyRxFifo( msp_phy_id);

  LLD_MSP_ResetReg( msp_phy_id);

  gpOS_interrupt_install( msp_phy_irq_line, irq_pri, (gpOS_interrupt_callback_t)svc_msp_callback, port_hdlr_ptr);
  gpOS_interrupt_enable( msp_phy_irq_line);

  if( last_port_hdlr_ptr == NULL)
  {
    svc_msp_handler->port_head = port_hdlr_ptr;
  }
  else
  {
    last_port_hdlr_ptr = port_hdlr_ptr;
  }

  svc_msp_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_msp_handler->svc_mcu_item_handler.part);

  gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Create COM on MSP port
 *
 * \param msp_port MSP port used
 * \param msp_mode Interface type
 * \param out_clk Out clock frequency
 * \param data_size Size of each data transmitted
 * \param pre_cb Callback called before any transfer
 * \param pre_cb_param Parameter passed to pre_cb
 * \param post_cb Callback called after any transfer
 * \param post_cb_param Parameter passed to post_cb
 * \return COM handler pointer
 *
 ***********************************************/
svc_msp_com_handler_t *svc_msp_create_com_config( tUInt msp_port, svc_msp_protocol_t msp_mode, svc_msp_comconfig_t msp_init_comconfig)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( svc_msp_handler->svc_mcu_item_handler.part);
  svc_msp_com_handler_t *com_hdlr_ptr;
  svc_msp_com_handler_t **last_com_hdlr_ptr_ptr;
  tU8 buf_size;
  const LLD_MSP_ConfigTy *msp_config;
  const LLD_MSP_ProtocolTy *msp_protocol;
  svc_msp_port_handler_t *port_hdlr_ptr;

  //
  // Check if it's safe to create a new com
  //

  // Check if SSP svc_mcu was initialized
  if( svc_msp_handler == NULL)
  {
    return NULL;
  }

  // check if specified port is available on platform
  if( msp_port >= svc_msp_handler->svc_mcu_item_handler.phy_item->number)
  {
    return NULL;
  }

  // check if handler for specified port was initialized
  if( svc_msp_get_hdlr_ptr( msp_port) == NULL)
  {
    return NULL;
  }

  // check if data size is ok and configure in/out buffer size
  if(msp_init_comconfig.data_size <= LLD_MSP_BITS4ELEM_8)
  {
    buf_size = 1;
  }
  else if( msp_init_comconfig.data_size <= LLD_MSP_BITS4ELEM_16)
  {
    buf_size = 2;
  }
  else if( msp_init_comconfig.data_size <= LLD_MSP_BITS4ELEM_32)
  {
    buf_size = 4;
  }
  else
  {
    return NULL;
  }

  // check if SSP mode or IIS mode is supported
  switch( msp_mode)
  {
    case SVC_MSP_PROTOCOL_SPI:
      msp_config = &svc_msp_basecfg_spi;
      msp_protocol = &svc_msp_baseprotocol_spi;
      break;

    case SVC_MSP_PROTOCOL_I2S:
      msp_config = &svc_msp_basecfg_i2s;
      msp_protocol = &svc_msp_baseprotocol_i2s;
      break;

    default:
      msp_config = NULL;
      break;
  }

  if( msp_config == NULL)
  {
    return NULL;
  }

  //
  // Allocate COM needed memory
  //

  // allocate memory for COM handler and exit if no space is available
  com_hdlr_ptr = gpOS_memory_allocate_p( svc_msp_handler->svc_mcu_item_handler.part, SVC_MSP_COM_HANDLER_SIZE);

  if( com_hdlr_ptr == NULL)
  {
    return NULL;
  }

  // create access semaphore and exit if no space is available
  com_hdlr_ptr->access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_msp_handler->svc_mcu_item_handler.part, 0);

  if( com_hdlr_ptr->access_sem == NULL)
  {
    gpOS_semaphore_delete( com_hdlr_ptr->access_sem);
    gpOS_memory_deallocate_p( svc_msp_handler->svc_mcu_item_handler.part, com_hdlr_ptr);
    return NULL;
  }

  //
  // Enqueue new COM handler on COM queue for specified port
  //

  last_com_hdlr_ptr_ptr = &svc_msp_handler->com_head;

  while( *last_com_hdlr_ptr_ptr != NULL)
  {
    last_com_hdlr_ptr_ptr = &(*last_com_hdlr_ptr_ptr)->next;
  }

  *last_com_hdlr_ptr_ptr = com_hdlr_ptr;

  com_hdlr_ptr->next    = NULL;

  //
  // Configure COM parameters
  //

  memcpy( &com_hdlr_ptr->config, msp_config, sizeof( LLD_MSP_ConfigTy));
  memcpy( &com_hdlr_ptr->protocol, msp_protocol, sizeof( LLD_MSP_ProtocolTy));

  com_hdlr_ptr->port_id               = msp_port;
  com_hdlr_ptr->hooks.pre_cb          = msp_init_comconfig.pre_cb;
  com_hdlr_ptr->hooks.pre_cb_param    = msp_init_comconfig.pre_cb_param;
  com_hdlr_ptr->hooks.post_cb         = msp_init_comconfig.post_cb;
  com_hdlr_ptr->hooks.post_cb_param   = msp_init_comconfig.post_cb_param;
  com_hdlr_ptr->buf_size              = buf_size;
  com_hdlr_ptr->protocol.spi_clk_mode =msp_init_comconfig.spi_clk_mode;
  com_hdlr_ptr->protocol.frame_period =msp_init_comconfig.frame_period;

  //
  // Configure MSP configuration for specified COM
  //

  port_hdlr_ptr = svc_msp_get_hdlr_ptr( msp_port);
  port_hdlr_ptr->msp_mode = msp_mode;
  com_hdlr_ptr->out_clk = msp_init_comconfig.out_clk;

  //memcpy( &com_hdlr_ptr->config, msp_config, sizeof( LLD_MSP_ConfigTy));
  //memcpy( &com_hdlr_ptr->protocol, msp_protocol, sizeof( LLD_MSP_ProtocolTy));

  com_hdlr_ptr->protocol.rx_element_length_1  = msp_init_comconfig.data_size;
  com_hdlr_ptr->protocol.tx_element_length_1  = msp_init_comconfig.data_size;

  svc_msp_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_msp_handler->svc_mcu_item_handler.part);

  svc_pwr_peripherallock_register(&port_hdlr_ptr->peripherallockid);

  gpOS_semaphore_signal( com_hdlr_ptr->access_sem);

  return com_hdlr_ptr;
}

/********************************************//**
 * \brief Create COM on MSP port
 *
 * \param msp_port MSP port used
 * \param msp_mode Interface type
 * \param out_clk Out clock frequency
 * \param data_size Size of each data transmitted
 * \param pre_cb Callback called before any transfer
 * \param pre_cb_param Parameter passed to pre_cb
 * \param post_cb Callback called after any transfer
 * \param post_cb_param Parameter passed to post_cb
 * \return COM handler pointer
 *
 ***********************************************/
svc_msp_com_handler_t *svc_msp_create_com( tUInt msp_port, svc_msp_protocol_t msp_mode, tU32 out_clk, LLD_MSP_Bits4ElemTy data_size, svc_msp_hook_t pre_cb, svc_msp_hook_param_t pre_cb_param, svc_msp_hook_t post_cb, svc_msp_hook_param_t post_cb_param)
{
  svc_msp_com_handler_t *com_hdlr_ptr;

  svc_msp_comconfig_t msp_cfg;

  msp_cfg.out_clk = out_clk;
  msp_cfg.data_size = data_size;
  msp_cfg.spi_clk_mode = svc_msp_baseprotocol_spi.spi_clk_mode;
  msp_cfg.frame_period = svc_msp_baseprotocol_spi.frame_period;
  msp_cfg.pre_cb = pre_cb;
  msp_cfg.pre_cb_param = pre_cb_param;
  msp_cfg.post_cb = post_cb;
  msp_cfg.post_cb_param = post_cb_param;


  com_hdlr_ptr = svc_msp_create_com_config(msp_port, msp_mode, msp_cfg);
  return com_hdlr_ptr;
}

/********************************************//**
 * \brief Write buffer on MSP COM
 *
 * \param msp_com_hdlr COM handler pointer
 * \param out_buf Pointer to buffer to transmit
 * \param len size of data to transfer
 * \param in_buf Pointer to buffer for receiving
 * \param timeout Timeout for accessing the resource
 * \return gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_msp_write( svc_msp_com_handler_t *msp_com_hdlr, tVoid *out_buf, tU32 len, tVoid *in_buf, gpOS_clock_t *timeout)
{
  if( (len == 0) || (msp_com_hdlr == NULL))
  {
    return gpOS_FAILURE;
  }
  else
  {
    svc_msp_port_handler_t *port_hdlr_ptr;
    LLD_MSP_IdTy msp_id;

    //
    // Access COM handler
    //
    gpOS_semaphore_wait( msp_com_hdlr->access_sem);

    port_hdlr_ptr = svc_msp_get_hdlr_ptr( msp_com_hdlr->port_id);
    msp_id = (LLD_MSP_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_MSP, port_hdlr_ptr->port_id);

    //
    // Access Port handler
    //
    gpOS_semaphore_wait( port_hdlr_ptr->access_sem);

     svc_pwr_peripherallock_acquire(port_hdlr_ptr->peripherallockid);
    //
    // If previous COM using the port was different, update MSP configuration
    //
    if( port_hdlr_ptr->curr_com_ptr != msp_com_hdlr)
    {

      tU32 bus_speed;

      LLD_MSP_EmptyTxFifo( msp_id);
      LLD_MSP_EmptyRxFifo( msp_id);

      LLD_MSP_ResetReg( msp_id);

      svc_mcu_busclk_get( svc_msp_handler->svc_mcu_item_handler.bus_id, &bus_speed);

      if( port_hdlr_ptr->msp_mode == SVC_MSP_PROTOCOL_SPI)
      {
        LLD_MSP_Configure(msp_id, &msp_com_hdlr->config, &msp_com_hdlr->protocol);
        LLD_MSP_ConfigSampleRateGen( msp_id, bus_speed, msp_com_hdlr->out_clk, msp_com_hdlr->protocol.frame_period, msp_com_hdlr->protocol.frame_width);
      }
      //LLD_SSP_SetConfiguration( ssp_id, &ssp_com_hdlr->config);
      if( port_hdlr_ptr->msp_mode == SVC_MSP_PROTOCOL_I2S)
      {
        LLD_MSP_Configure(msp_id, &msp_com_hdlr->config, &msp_com_hdlr->protocol);
        LLD_MSP_ConfigSampleRateGen(msp_id, bus_speed, msp_com_hdlr->out_clk, 32,16);
      }
    }

    //
    // Configure transmission on port
    //
    port_hdlr_ptr->curr_com_ptr   = msp_com_hdlr;
    port_hdlr_ptr->tx_buf_ptr     = out_buf;
    port_hdlr_ptr->rx_buf_ptr     = in_buf;

    port_hdlr_ptr->tx_pos         = 0;
    port_hdlr_ptr->rx_pos         = 0;

    port_hdlr_ptr->len            = len;

    //
    // Execute callback to be used before transmission
    //
    if( msp_com_hdlr->hooks.pre_cb != NULL)
    {
      msp_com_hdlr->hooks.pre_cb(  msp_com_hdlr->hooks.pre_cb_param);
    }

    //
    // Enable peripheral
    //
    LLD_MSP_EnSampleRateGen( msp_id);

    if( port_hdlr_ptr->msp_mode == SVC_MSP_PROTOCOL_SPI)
    {
      LLD_MSP_SetSPIClockMode( msp_id, msp_com_hdlr->protocol.spi_clk_mode);
    }

    LLD_MSP_EnableTxRx(msp_id);
    LLD_MSP_IRQEnable(msp_id, LLD_MSP_IRQ_TRANSMIT | LLD_MSP_IRQ_RECEIVE);   //LLD_MSP_IRQ_RECEIVE

    gpOS_semaphore_wait( port_hdlr_ptr->done_sem);

    if( port_hdlr_ptr->msp_mode == SVC_MSP_PROTOCOL_I2S)
    {
      LLD_MSP_WaitTxEmpty(msp_id);
    }

    //
    // Execute callback to be used after transmission
    //
    if( msp_com_hdlr->hooks.post_cb != NULL)
    {
      msp_com_hdlr->hooks.post_cb( msp_com_hdlr->hooks.post_cb_param);
    }

    gpOS_task_delay( gpOS_timer_ticks_per_msec());

    LLD_MSP_DisableTxRx( msp_id);
    LLD_MSP_DisSampleRateGen( msp_id);
    LLD_MSP_IRQDisable( msp_id, LLD_MSP_IRQ_ALL);

    port_hdlr_ptr->tx_buf_ptr     = NULL;
    port_hdlr_ptr->rx_buf_ptr     = NULL;
    port_hdlr_ptr->len            = 0;
    port_hdlr_ptr->tx_pos         = 0;
    port_hdlr_ptr->rx_pos         = 0;

    svc_pwr_peripherallock_release(port_hdlr_ptr->peripherallockid);

    gpOS_semaphore_signal( port_hdlr_ptr->access_sem);

    gpOS_semaphore_signal( msp_com_hdlr->access_sem);

    return gpOS_SUCCESS;
  }
}

