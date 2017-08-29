/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Implements APIs for MTU svc_mcu
 *
 ******************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "typedefs.h"
#include "lld_mtu.h"

#warning "PRCC.CLKCTRL must not be controlled by MTU service"
#if defined(__STA8088__)
#include "lld_apb2_sta8088.h"
#endif
#if defined(__STA8090__)
#include "lld_prcc_sta8090.h"
#endif

#include "svc_mcu.h"
#include "svc_mtu.h"
#include "string.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_MTU_HANDLER_SIZE         sizeof( svc_mtu_handler_t)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/**< MTU port handler */
typedef struct svc_mtu_port_handler_s svc_mtu_port_handler_t;

struct svc_mtu_port_handler_s
{
  svc_mtu_port_handler_t *   next;             /**< Next port in the list */

  tU32                        id;               /**< ID of this port handler */

  LLD_MTU_ModeTy              mode;             /**< MTU mode */
  LLD_MTU_SizeTy              size;             /**< MTU counter size */
  LLD_MTU_PrescalerTy         prescaler;        /**< MTU clock prescaler */

  boolean_t                   use_ext_clk;      /**< Defines if external clock is used for this port */
  tU32                        int_clk_freq;     /**< Main bus frequency */
  tU32                        ext_clk_freq;     /**< External clock frequency */

  gpOS_interrupt_callback_t   irq_callback;     /**< Callback that will be used when MTU reaches 0 */
};

/**< MTU svc_mcu handler */
typedef struct svc_mtu_handler_s
{
  svc_mcu_item_t    svc_mcu_item_handler; /**< svc_mcu handler */

  tU8               num_of_ports;     /**< Total number of ports */
  tU16              open_bank_mask;   /**< Mask of all opened ports */

  svc_mtu_port_handler_t *   port_head;        /**< First port in the list */
} svc_mtu_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

/**< Array of the port specific MTU units IRQ lines */
static const gpOS_interrupt_line_t svc_mtu_port_phy_irq_table[] =
{
  #if defined( __STA8088__ )
  (gpOS_interrupt_line_t)VIC_MTU_TIM0_LINE,
  (gpOS_interrupt_line_t)VIC_MTU_TIM1_LINE,
  (gpOS_interrupt_line_t)VIC_MTU_TIM2_LINE,
  (gpOS_interrupt_line_t)VIC_MTU_TIM3_LINE
  #endif
  #if defined( __STA8090__ )
  (gpOS_interrupt_line_t)VIC_MTU0_TIM0_LINE,
  (gpOS_interrupt_line_t)VIC_MTU0_TIM1_LINE,
  (gpOS_interrupt_line_t)VIC_MTU0_TIM2_LINE,
  (gpOS_interrupt_line_t)VIC_MTU0_TIM3_LINE,
  (gpOS_interrupt_line_t)VIC_MTU1_TIM0_LINE,
  (gpOS_interrupt_line_t)VIC_MTU1_TIM1_LINE,
  (gpOS_interrupt_line_t)VIC_MTU1_TIM2_LINE,
  (gpOS_interrupt_line_t)VIC_MTU1_TIM3_LINE
  #endif
};

static svc_mtu_handler_t *svc_mtu_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief     Look up for a port in the handler
 *
 * \param[in]   mtu_port      MTU port number
 * \param[out]  hdlr_ptr_ptr  Pointer to save handler pointer
 * \param[out]  mtu_bank_ptr  Pointer to save MTU bank of the port
 * \param[out]  mtu_id_ptr    Pointer to save MTU unit ID in the bank
 * \return      gpOS_SUCCESS if found
 *
 ***********************************************/
static gpOS_error_t svc_mtu_port_lookup( const tU32 mtu_port, svc_mtu_port_handler_t **hdlr_ptr_ptr, tU32 *mtu_bank_ptr, LLD_MTU_IdTy *mtu_id_ptr)
{
  svc_mtu_port_handler_t *hdlr_ptr;

  if( svc_mtu_handler == NULL)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  if( mtu_port > svc_mtu_handler->num_of_ports)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  if( !MCR_ISBITSET( svc_mtu_handler->open_bank_mask, mtu_port))
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Lookup handler for specific port
  hdlr_ptr = svc_mtu_handler->port_head;

  while( hdlr_ptr != NULL)
  {
    if( hdlr_ptr->id == mtu_port)
    {
      break;
    }
    else
    {
      hdlr_ptr = hdlr_ptr->next;
    }
  }

  *hdlr_ptr_ptr   = hdlr_ptr;
  *mtu_bank_ptr   = mtu_port >> 2;
  *mtu_id_ptr     = (LLD_MTU_IdTy)(mtu_port & 0x3U);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief     MTU handler ISR callback
 *            This is used as generic framework to handle MTU ISR port ISR.
 *            The param is used as handler pointer to point related MTU port
 *
 * \param     irq_param   MTU port handler
 * \return    None
 *
 ***********************************************/
static gpOS_ISR void svc_mtu_irq_handler_stub( void *irq_param)
{
  svc_mtu_port_handler_t *hdlr_ptr = irq_param;
  tU32      mtu_bank    = hdlr_ptr->id >> 2;
  LLD_MTU_IdTy  mtu_id      = (LLD_MTU_IdTy)(hdlr_ptr->id & 0x3U);
  LLD_MTU_IPTy  mtu_phy_id  = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  LLD_MTU_ClearInterruptMask( mtu_phy_id, BIT_x(mtu_id));
  LLD_MTU_ClearInterrupt( mtu_phy_id, BIT_x(mtu_id));

  if( hdlr_ptr->irq_callback != NULL)
  {
    hdlr_ptr->irq_callback( NULL);
  }

  LLD_MTU_SetInterruptMask( mtu_phy_id, BIT_x(mtu_id));
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief   Initialize SVC MTU svc_mcu
 *
 * \param   partition Partition to use to allocate memory
 * \param   bus_speed Bus speed
 * \return  gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_mtu_init( gpOS_partition_t *partition, const tU32 bus_id)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_mtu_handler != NULL)
  {
    return gpOS_SUCCESS;/*lint !e904 Return statement before end of function */
  }

  svc_mtu_handler = gpOS_memory_allocate_p( partition, SVC_MTU_HANDLER_SIZE);

  if( svc_mtu_handler == NULL)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Install service on main handler and fill physical info
  if( svc_mcu_install( SVC_MCU_PER_ID_MTU, bus_id, &svc_mtu_handler->svc_mcu_item_handler) == gpOS_FAILURE)
  {
    gpOS_memory_deallocate_p( partition, svc_mtu_handler);
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  svc_mtu_handler->num_of_ports = (tU8)(svc_mtu_handler->svc_mcu_item_handler.phy_item->number) << 2;
  svc_mtu_handler->open_bank_mask = 0;

  svc_mtu_handler->svc_mcu_item_handler.part         = partition;
  svc_mtu_handler->svc_mcu_item_handler.mem_used     = mem_at_start - gpOS_memory_getheapfree_p( partition);
  svc_mtu_handler->svc_mcu_item_handler.cmdif        = NULL;

  svc_mtu_handler->port_head  = NULL;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Open MTU port
 *
 * \param   mtu_port  Port to be opened
 * \return  gpOS_SUCCESS if the port is successfully opened
 *
 ***********************************************/
gpOS_error_t svc_mtu_open_port( const tU32 mtu_port)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  tU32 mem_at_start;

  // Check if a port can be open (FIFO sizes, handler initialized, ...)
  if( svc_mtu_handler == NULL)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Check if port exists
  if( mtu_port > svc_mtu_handler->num_of_ports)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Check if port was already opened
  if( MCR_ISBITSET( svc_mtu_handler->open_bank_mask, mtu_port))
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Get MTU bank and bank ID related to the port
  mtu_bank  = mtu_port >> 2;
  /*mtu_id    = mtu_port & (tU32)0x3;*/

  // Get physical address in registry map for the port
  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  // Allocate port handler memory
  mem_at_start      = gpOS_memory_getheapfree_p( svc_mtu_handler->svc_mcu_item_handler.part);

  hdlr_ptr = gpOS_memory_allocate_p( svc_mtu_handler->svc_mcu_item_handler.part, sizeof( svc_mtu_port_handler_t));

  if( hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Enable MTU bank. If already enabled, it does not have effects
  svc_mcu_enable( SVC_MCU_PER_ID_MTU, mtu_bank);

  // Reset MTU bank if no port is currently open
  if( (svc_mtu_handler->open_bank_mask & ((tU16)0xFU << mtu_bank)) == 0U )
  {
    LLD_MTU_Reset( mtu_phy_id);
  }

  // Fill port handler fields
  hdlr_ptr->id              = mtu_port;
  hdlr_ptr->next            = svc_mtu_handler->port_head;
  hdlr_ptr->irq_callback    = NULL;

  // Add port to MTU ports list in MTU handler
  MCR_SETBIT( svc_mtu_handler->open_bank_mask, mtu_port);
  svc_mtu_handler->port_head = hdlr_ptr;

  svc_mtu_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_mtu_handler->svc_mcu_item_handler.part);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Enable MTU port to count
 *
 * \param   mtu_port  Port to enable
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_start( const tU32 mtu_port)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  LLD_MTU_IdTy mtu_id;

  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  LLD_MTU_Enable( mtu_phy_id, mtu_id);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Disable MTU port to count
 *
 * \param   mtu_port  Port to disable
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_stop( const tU32 mtu_port)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  LLD_MTU_IdTy mtu_id;

  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  LLD_MTU_Disable( mtu_phy_id, mtu_id);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Configure MTU port
 *
 * \param   mtu_port        Port to be configured
 * \param   mode            MTU mode (periodic or one shot)
 * \param   prescaler       Clock prescaler
 * \param   size            Size of counter registers (16 or 32)
 * \param   use_ext         TRUE if external clock must be used
 * \param   external_freq   External frequency (used if use_ext==TRUE)
 * \param   load_value      Value to be loaded in Load register (0 means no update)
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_port_config( const tU32 mtu_port, const svc_mtu_cfg_t *cfg_ptr)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  LLD_MTU_IdTy mtu_id;
  boolean_t is_enabled;
  tU32 bus_freq;
  boolean_t use_ext;

  // Lookup for port handler
  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Get physical address in register map of related bank
  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  // Save if MTU is enabled, then disable it
  is_enabled = LLD_MTU_IsEnabled( mtu_phy_id, mtu_id);

  LLD_MTU_Disable( mtu_phy_id, mtu_id);

  // Set handler with given parameters
  hdlr_ptr->size        = cfg_ptr->size;
  hdlr_ptr->prescaler   = cfg_ptr->prescaler;
  hdlr_ptr->mode        = cfg_ptr->mode;

  // Set frequency of the internal clock coming from bus clock
  svc_mcu_busclk_get( svc_mtu_handler->svc_mcu_item_handler.bus_id, &bus_freq);
  hdlr_ptr->int_clk_freq = bus_freq;

  // Set external frequency if needed
  use_ext = ( cfg_ptr->ext_freq == 0) ? FALSE : TRUE;

  if( use_ext == FALSE)
  {
#if defined(__STA8088__)
    LLD_APB2_DisableG3ctrldMTU( BIT_x(mtu_port));
#endif
#if defined(__STA8090__)
    LLD_PRCC_DisableG3ctrldMTU( BIT_x(mtu_port));
#endif
  }
  else
  {
#if defined(__STA8088__)
    LLD_APB2_EnableG3ctrldMTU( BIT_x(mtu_port));
#endif
#if defined(__STA8090__)
    LLD_PRCC_EnableG3ctrldMTU( BIT_x(mtu_port));
#endif
  }
  hdlr_ptr->use_ext_clk = use_ext;
  hdlr_ptr->ext_clk_freq = cfg_ptr->ext_freq;

  // Configure port and clean interrupts
  LLD_MTU_Config( mtu_phy_id, mtu_id, hdlr_ptr->size, hdlr_ptr->prescaler, hdlr_ptr->mode);

  if( cfg_ptr->load_value != 0U)
  {
    LLD_MTU_SetLoadRegister( mtu_phy_id, mtu_id, cfg_ptr->load_value);
  }

  LLD_MTU_ClearInterrupt( mtu_phy_id, BIT_x(mtu_id));

  // Reenable port if it was enabled when entering the API
  if( is_enabled == TRUE)
  {
    LLD_MTU_Enable( mtu_phy_id, mtu_id);
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Configure ISR for specific port
 *
 * \param   mtu_port            Port to be configured
 * \param   priority            Interrupt priority
 * \param   mtu_callback        Custom callback to be called at interrupt occurrency
 * \param   mtu_callback_param  Parameter to pass to callback (NOT USED)
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_port_set_irq( const tU32 mtu_port, gpOS_interrupt_priority_t priority, gpOS_interrupt_callback_t mtu_callback, const void *mtu_callback_param)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  LLD_MTU_IdTy mtu_id;
  boolean_t is_enabled;

  // Lookup for port handler
  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Get physical address in register map of related bank
  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  // Save if MTU is enabled, then disable it
  is_enabled = LLD_MTU_IsEnabled( mtu_phy_id, mtu_id);

  LLD_MTU_Disable( mtu_phy_id, mtu_id);

  // Set ISR callback and enable MTU port interrupt in bank mask
  hdlr_ptr->irq_callback = mtu_callback;

  LLD_MTU_SetInterruptMask( mtu_phy_id, BIT_x(mtu_id));

  // Install MTU handler in OS20 for line specific to port.
  gpOS_interrupt_install( svc_mtu_port_phy_irq_table[mtu_port], priority, svc_mtu_irq_handler_stub, hdlr_ptr);
  gpOS_interrupt_enable( svc_mtu_port_phy_irq_table[mtu_port]);

  // Reenable port if it was enabled when entering the API
  if( is_enabled == TRUE)
  {
    LLD_MTU_Enable( mtu_phy_id, mtu_id);
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Disable ISR for specific port
 *
 * \param   mtu_port            Port to be configured
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_port_disable_irq( const tU32 mtu_port)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  LLD_MTU_IdTy mtu_id;
  boolean_t is_enabled;

  // Lookup for port handler
  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Get physical address in register map of related bank
  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  // Save if MTU is enabled, then disable it
  is_enabled = LLD_MTU_IsEnabled( mtu_phy_id, mtu_id);

  LLD_MTU_Disable( mtu_phy_id, mtu_id);

  LLD_MTU_ClearInterruptMask(mtu_phy_id, BIT_x(mtu_id));

  gpOS_interrupt_disable( svc_mtu_port_phy_irq_table[mtu_port]);
  gpOS_interrupt_uninstall( svc_mtu_port_phy_irq_table[mtu_port]);


  // Reenable port if it was enabled when entering the API
  if( is_enabled == TRUE)
  {
    LLD_MTU_Enable( mtu_phy_id, mtu_id);
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Get value of the counter of specific port
 *
 * \param   mtu_port    Port to read
 * \param   value_ptr   Pointer to value variable
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_port_get_value( const tU32 mtu_port, tU32 *value_ptr)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  LLD_MTU_IdTy mtu_id;

  // Lookup for port handler
  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Get physical address in register map of related bank
  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  // Save value from MTU port
  *value_ptr = LLD_MTU_GetValueRegister( mtu_phy_id, mtu_id);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Get number of ticks per second for specific port
 *
 * \param   mtu_port    Port to read
 * \param   ticks_ptr   Pointer to ticks variable
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_port_get_ticks( const tU32 mtu_port, tU32 *ticks_ptr)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IdTy mtu_id;
  tU32 ticks = 0;

  // Lookup for port handler
  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // If external clock is used, use its frequency as start value for ticks evaluation,
  // else use bus clock frequency
  if( hdlr_ptr->use_ext_clk == TRUE)
  {
    ticks = hdlr_ptr->ext_clk_freq;
  }
  else
  {
    ticks = hdlr_ptr->int_clk_freq;
  }

  // Divide ticks if any prescaler is configured
  if( hdlr_ptr->prescaler == LLD_MTU_PRESCALER_DIV_16)
  {
    ticks >>= 4;
  }
  else if( hdlr_ptr->prescaler == LLD_MTU_PRESCALER_DIV_256)
  {
    ticks >>= 8;
  }
  else
  {
    /* LLD_MTU_PRESCALER_DIV_1 nothing to do */
  }

  // Save evaluated ticks in destination variable
  *ticks_ptr = ticks;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Set load register for specific MTU port
 *
 * \param   mtu_port    Port to be configured
 * \param   load_value  Value to load
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_port_set_load_value( const tU32 mtu_port, tU32 load_value)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  LLD_MTU_IdTy mtu_id;
  boolean_t is_enabled;

  // Lookup for port handler
  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Get physical address in register map of related bank
  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  // Save if MTU is enabled, then disable it
  is_enabled = LLD_MTU_IsEnabled( mtu_phy_id, mtu_id);

  // Set load register by temporarely disabling port
  LLD_MTU_Disable( mtu_phy_id, mtu_id);
  LLD_MTU_SetLoadRegister( mtu_phy_id, mtu_id, load_value);

  // Reenable port if it was enabled when entering the API
  if( is_enabled == TRUE)
  {
    LLD_MTU_Enable( mtu_phy_id, mtu_id);
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Get physical information of MTU port
 *
 * \param[in]   mtu_port          Port of which info are needed
 * \param[out]  mtu_phy_bank_ptr  Physical address of MTU bank relative to port
 * \param[out]  mtu_phy_id_ptr    Physical ID in MTU bank relative to port
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_port_get_phy_params( const tU32 mtu_port, LLD_MTU_IPTy *mtu_phy_bank_ptr, LLD_MTU_IdTy *mtu_phy_id_ptr)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;

  // Lookup for port handler, this fills also physical ID pointer
  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, mtu_phy_id_ptr) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  *mtu_phy_bank_ptr = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Configure mode for MTU port
 *
 * \param   mtu_port        Port to be configured
 * \param   mode            MTU mode (periodic or one shot)
 * \return  gpOS_SUCCESS if all is ok
 *
 ***********************************************/
gpOS_error_t svc_mtu_port_set_mode( const tU32 mtu_port, const LLD_MTU_ModeTy mode)
{
  svc_mtu_port_handler_t *hdlr_ptr;
  tU32 mtu_bank;
  LLD_MTU_IPTy mtu_phy_id;
  LLD_MTU_IdTy mtu_id;
  boolean_t is_enabled;

  // Lookup for port handler
  if( svc_mtu_port_lookup( mtu_port, &hdlr_ptr, &mtu_bank, &mtu_id) == gpOS_FAILURE)
  {
    return gpOS_FAILURE;/*lint !e904 Return statement before end of function */
  }

  // Get physical address in register map of related bank
  mtu_phy_id = (LLD_MTU_IPTy)svc_mtu_handler->svc_mcu_item_handler.phy_item->addr[mtu_bank];

  // Save if MTU is enabled, then disable it
  is_enabled = LLD_MTU_IsEnabled( mtu_phy_id, mtu_id);

  LLD_MTU_Disable( mtu_phy_id, mtu_id);

  hdlr_ptr->mode        = mode;

  LLD_MTU_SetMode(mtu_phy_id, mtu_id, mode);

  // Reenable port if it was enabled when entering the API
  if( is_enabled == TRUE)
  {
    LLD_MTU_Enable( mtu_phy_id, mtu_id);
  }

  return gpOS_SUCCESS;
}

