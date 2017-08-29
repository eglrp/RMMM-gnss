/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 *-----------------------------------------------------------------------------
 * Implements APIs for GPIO svc_mcu
 *
 ******************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "typedefs.h"
#include "clibs.h"

#include "lld_gpio.h"

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_gpio.h"
#include "platform.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define SVC_GPIO_PIN_NUMBER          32
#define SVC_GPIO_HANDLER_SIZE        sizeof( svc_gpio_handler_t)
#define SVC_GPIO_ALT_BITS            0x0000000000000003

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  SVC_GPIO_ALT_NONE  = 0x00,
  SVC_GPIO_ALT_A     = 0x01,
  SVC_GPIO_ALT_B     = 0x02,
  SVC_GPIO_ALT_C     = 0x03,
} svc_gpio_alternate_t;

typedef struct svc_gpio_port_handler_s svc_gpio_port_handler_t;

struct svc_gpio_port_handler_s
{
  svc_gpio_port_handler_t *   next;

  // access semaphore to the critical resource
  gpOS_semaphore_t *            access_sem;

  // Stream config infos
  tUInt                       id;
  svc_gpio_isr_t *            gpio_isr;
  tU64 *                      gpio_alt;
};

typedef struct svc_gpio_handler_s
{
  svc_mcu_item_t              svc_mcu_item_handler;

  svc_gpio_port_handler_t *   port_head;
} svc_gpio_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static svc_gpio_handler_t *svc_gpio_handler = NULL;

#if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
static const tU64 svc_gpio_port_0_alternate = 0x0000000000000000;
static const tU64 svc_gpio_port_1_alternate = 0x0000000000000000;
static const tU64 svc_gpio_port_2_alternate = 0x0000000000000000;
static const tU64 svc_gpio_port_3_alternate = 0x0000000000000000;
#elif defined( __STA8088__ )
static const tU64 svc_gpio_port_0_alternate = 0x5555555555550000;
static const tU64 svc_gpio_port_1_alternate = 0x5555555555555555;
#elif defined( __STA8090__ )
static const tU64 svc_gpio_port_0_alternate = 0x0545CF1555558F00;
static const tU64 svc_gpio_port_1_alternate = 0x0000000055555555;
#endif

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

static svc_gpio_port_handler_t *  svc_gpio_get_hdlr_ptr    ( tUInt gpio_id);
static void                       svc_gpio_com_create_fail ( svc_gpio_port_handler_t *hdlr_ptr);
static void                       svc_gpio_callback        ( svc_gpio_port_handler_t *hdlr_ptr);

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief
 *
 * \param gpio_id tUInt
 * \return svc_gpio_port_handler_t*
 *
 ***********************************************/
static svc_gpio_port_handler_t *svc_gpio_get_hdlr_ptr( tUInt gpio_id)
{
  svc_gpio_port_handler_t *hdlr_ptr;

  if( svc_gpio_handler == NULL)
  {
    return NULL;
  }

  hdlr_ptr = svc_gpio_handler->port_head;

  while( hdlr_ptr != NULL)
  {
    if( hdlr_ptr->id == gpio_id)
    {
      return( hdlr_ptr);
    }
    else
    {
      hdlr_ptr = hdlr_ptr->next;
    }
  }

  return NULL;
}

/********************************************//**
 * \brief
 *
 * \param hdlr_ptr svc_gpio_port_handler_t*
 * \return void
 *
 ***********************************************/
static LLD_ISR_GPIO void svc_gpio_callback( svc_gpio_port_handler_t *hdlr_ptr)
{
  LLD_GPIO_idTy gpio_phy_id = (LLD_GPIO_idTy)svc_mcu_get_addr( SVC_MCU_PER_ID_GPIO, hdlr_ptr->id);
  tU32 i, irq_status;

  irq_status = LLD_GPIO_GetInterruptStatus( gpio_phy_id);

  for( i=0; i<SVC_GPIO_PIN_NUMBER; i++)
  {
    if(( BIT_0 << i) & irq_status)
    {
      if( hdlr_ptr->gpio_isr[i] != NULL)
      {
        hdlr_ptr->gpio_isr[i]( NULL);
      }
      LLD_GPIO_ClearInterrupt( gpio_phy_id, ( BIT_0 << i));
    }
  }
}

/********************************************//**
 * \brief
 *
 * \param hdlr_ptr svc_gpio_port_handler_t*
 * \return void
 *
 ***********************************************/
static void svc_gpio_com_create_fail( svc_gpio_port_handler_t *hdlr_ptr)
{
  gpOS_interrupt_uninstall( svc_mcu_get_irq_line( SVC_MCU_PER_ID_GPIO, hdlr_ptr->id));

  gpOS_semaphore_delete( hdlr_ptr->access_sem);

  gpOS_memory_deallocate_p( svc_gpio_handler->svc_mcu_item_handler.part, hdlr_ptr->gpio_isr);
  gpOS_memory_deallocate_p( svc_gpio_handler->svc_mcu_item_handler.part, hdlr_ptr);
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
gpOS_error_t svc_gpio_init( gpOS_partition_t *partition)
{
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);

  if( svc_gpio_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_gpio_handler = gpOS_memory_allocate_p( partition, SVC_GPIO_HANDLER_SIZE);

  if( svc_gpio_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  // Install service on main handler and fill physical info
  if( svc_mcu_install( SVC_MCU_PER_ID_GPIO, PLATFORM_BUSCLK_ID_MCLK, &svc_gpio_handler->svc_mcu_item_handler) == gpOS_FAILURE)
  {
    gpOS_memory_deallocate_p( partition, svc_gpio_handler);
    return gpOS_FAILURE;
  }

  svc_gpio_handler->svc_mcu_item_handler.part      = partition;
  svc_gpio_handler->svc_mcu_item_handler.mem_used  = mem_at_start - gpOS_memory_getheapfree_p( partition);
  svc_gpio_handler->port_head  = NULL;

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param port_id svc_gpio_port_id_t
 * \param irq_pri gpOS_interrupt_priority_t
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_gpio_open_port( svc_gpio_port_id_t port_id, gpOS_interrupt_priority_t irq_pri)
{
  tU32 mem_at_start;
  svc_gpio_port_handler_t *last_hdlr_ptr, *hdlr_ptr;
  LLD_GPIO_idTy gpio_phy_id;
  VicLineTy     gpio_phy_irq_line;

  if( svc_gpio_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  if( port_id >= svc_gpio_handler->svc_mcu_item_handler.phy_item->number)
  {
    return gpOS_FAILURE;
  }

  // Read start address of free partition area
  mem_at_start = gpOS_memory_getheapfree_p( svc_gpio_handler->svc_mcu_item_handler.part);

  // Check if port was already open
  last_hdlr_ptr = svc_gpio_handler->port_head;
  hdlr_ptr = svc_gpio_handler->port_head;

  // Check if port is already open
  while( hdlr_ptr != NULL)
  {
    if( hdlr_ptr->id == port_id)
    {
      return( gpOS_FAILURE);
    }
    else
    {
      last_hdlr_ptr = hdlr_ptr;
      hdlr_ptr = hdlr_ptr->next;
    }
  }

  // Allocate memory needed for GPIO port handler
  hdlr_ptr = ( svc_gpio_port_handler_t *)gpOS_memory_allocate_p( svc_gpio_handler->svc_mcu_item_handler.part, sizeof( svc_gpio_port_handler_t));

  if( hdlr_ptr == NULL)
  {
    return gpOS_FAILURE;
  }

  // Allocate memory needed for GPIO ISRs
  hdlr_ptr->gpio_isr = ( svc_gpio_isr_t *)gpOS_memory_allocate_p( svc_gpio_handler->svc_mcu_item_handler.part, ( SVC_GPIO_PIN_NUMBER * sizeof( tInt)));

  if( hdlr_ptr->gpio_isr == NULL)
  {
    return gpOS_FAILURE;
  }
  _clibs_memset( hdlr_ptr->gpio_isr, 0, ( SVC_GPIO_PIN_NUMBER * sizeof( tInt)));

  // Create semaphore to protect shared area
  hdlr_ptr->access_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_gpio_handler->svc_mcu_item_handler.part, 1);

  if( hdlr_ptr->access_sem == NULL)
  {
    svc_gpio_com_create_fail( hdlr_ptr);
    return gpOS_FAILURE;
  }

  // Initialize GPIO device and handler
  hdlr_ptr->id            = port_id;
  hdlr_ptr->next          = NULL;

  svc_mcu_enable( SVC_MCU_PER_ID_GPIO, port_id);

  // Configure GPIO
  gpio_phy_id = ( LLD_GPIO_idTy)svc_mcu_get_addr( SVC_MCU_PER_ID_GPIO, port_id);
  LLD_GPIO_InterruptDisable( gpio_phy_id, LLD_GPIO_ALL_PINS);
  LLD_GPIO_ClearInterrupt( gpio_phy_id, LLD_GPIO_ALL_PINS);

  // Install general GPIO callback
  gpio_phy_irq_line = ( VicLineTy)svc_mcu_get_irq_line( SVC_MCU_PER_ID_GPIO, port_id);
  gpOS_interrupt_install( gpio_phy_irq_line, irq_pri, (gpOS_interrupt_callback_t)svc_gpio_callback, hdlr_ptr);
  gpOS_interrupt_enable( gpio_phy_irq_line);

  switch( port_id)
  {
    case SVC_GPIO_PORT_0:
      hdlr_ptr->gpio_alt = ( tU64 *)&svc_gpio_port_0_alternate;
      break;
    case SVC_GPIO_PORT_1:
      hdlr_ptr->gpio_alt = ( tU64 *)&svc_gpio_port_1_alternate;
      break;
#if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
    case SVC_GPIO_PORT_2:
      hdlr_ptr->gpio_alt = ( tU64 *)&svc_gpio_port_2_alternate;
      break;
    case SVC_GPIO_PORT_3:
      hdlr_ptr->gpio_alt = ( tU64 *)&svc_gpio_port_3_alternate;
      break;
#endif
    default:
      break;
  }

  if( last_hdlr_ptr == NULL)
  {
    svc_gpio_handler->port_head = hdlr_ptr;
  }
  else
  {
    last_hdlr_ptr->next = hdlr_ptr;
  }

  svc_gpio_handler->svc_mcu_item_handler.mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_gpio_handler->svc_mcu_item_handler.part);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param port_id svc_gpio_port_id_t
 * \param pin_id svc_gpio_pin_id_t
 * \param gpio_isr svc_gpio_isr_t
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_gpio_interrupt_install( svc_gpio_port_id_t port_id, svc_gpio_pin_id_t pin_id, svc_gpio_isr_t gpio_isr)
{
  svc_gpio_port_handler_t *hdlr_ptr = svc_gpio_get_hdlr_ptr( port_id);

  if(( hdlr_ptr == NULL) || ( pin_id >= SVC_GPIO_PIN_NUMBER) || ( gpio_isr == NULL))
  {
    return gpOS_FAILURE;
  }

  gpOS_semaphore_wait( hdlr_ptr->access_sem);

  hdlr_ptr->gpio_isr[pin_id] = gpio_isr;

  gpOS_semaphore_signal( hdlr_ptr->access_sem);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param port_id svc_gpio_port_id_t
 * \param pin_id svc_gpio_pin_id_t
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_gpio_pin_enable( svc_gpio_port_id_t port_id, svc_gpio_pin_id_t pin_id)
{
  LLD_GPIO_idTy gpio_phy_id;
  svc_gpio_alternate_t pin_alt;
  svc_gpio_port_handler_t *hdlr_ptr = svc_gpio_get_hdlr_ptr( port_id);

  if(( hdlr_ptr == NULL) || ( pin_id >= SVC_GPIO_PIN_NUMBER))
  {
    return gpOS_FAILURE;
  }

  gpio_phy_id = (LLD_GPIO_idTy)svc_mcu_get_addr( SVC_MCU_PER_ID_GPIO, port_id);
  pin_alt = ( svc_gpio_alternate_t)( SVC_GPIO_ALT_BITS & ( *hdlr_ptr->gpio_alt >> ( 2 * pin_id)));

  LLD_GPIO_SetControlMode( gpio_phy_id, ( BIT_0 << pin_id), ( LLD_GPIO_ModeTy)pin_alt);
  LLD_GPIO_SetDirectionInput( gpio_phy_id, ( BIT_0 << pin_id));

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param port_id svc_gpio_port_id_t
 * \param pin_id svc_gpio_pin_id_t
 * \param edge svc_gpio_edge_t
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_gpio_interrupt_enable( svc_gpio_port_id_t port_id, svc_gpio_pin_id_t pin_id, svc_gpio_edge_t edge)
{
  LLD_GPIO_idTy gpio_phy_id;
  svc_gpio_port_handler_t *hdlr_ptr = svc_gpio_get_hdlr_ptr( port_id);

  if(( hdlr_ptr == NULL) || ( pin_id >= SVC_GPIO_PIN_NUMBER) || ( edge >= SVC_GPIO_EDGE_COUNT))
  {
    return gpOS_FAILURE;
  }

  gpio_phy_id = (LLD_GPIO_idTy)svc_mcu_get_addr( SVC_MCU_PER_ID_GPIO, port_id);

  LLD_GPIO_SetInterruptType( gpio_phy_id, ( BIT_0 << pin_id), ( LLD_GPIO_IntTy)edge);

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief
 *
 * \param port_id svc_gpio_port_id_t
 * \param pin_id svc_gpio_pin_id_t
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_gpio_interrupt_disable( svc_gpio_port_id_t port_id, svc_gpio_pin_id_t pin_id)
{
  LLD_GPIO_idTy gpio_phy_id;
  svc_gpio_port_handler_t *hdlr_ptr = svc_gpio_get_hdlr_ptr( port_id);

  if(( hdlr_ptr == NULL) || ( pin_id >= SVC_GPIO_PIN_NUMBER))
  {
    return gpOS_FAILURE;
  }

  gpio_phy_id = (LLD_GPIO_idTy)svc_mcu_get_addr( SVC_MCU_PER_ID_GPIO, port_id);

  LLD_GPIO_SetInterruptType( gpio_phy_id, ( BIT_0 << pin_id), LLD_GPIO_DISABLED_INT);

  return gpOS_SUCCESS;
}
