/**
 * @file    svc_mcu.c
 * @brief   svc_mcus general handler.
 *
 * @addtogroup SERVICES
 */

/*****************************************************************************
   includes
*****************************************************************************/

#include "defines.h"
#include "macros.h"
#include "typedefs.h"

#include "gpOS.h"
#include "svc_mcu.h"

#include "lld.h"
#include "lld_gpio.h"
#include "lld_vic.h"
#include "lld_uart.h"
#include "lld_ssp.h"
#include "lld_sdi.h"
#include "lld_can.h"
#include "lld_i2c.h"
#include "lld_msp.h"

#if defined( __STA2062__ )
#include "lld_src.h"
#elif defined( __STA2064__ ) || defined( __STA2065__ )
#include "lld_src.h"
#include "lld_arm1176.h"
#elif defined( __STA8088__)
#include "lld_adc_sta8088.h"
#include "lld_apb2_sta8088.h"
#include "lld_arm946.h"
#include "lld_clk_ctrl_sta8088.h"
#elif defined( __STA8090__)
#include "svc_pwr.h"
#include "lld_adc_sta8090.h"
#include "lld_arm946.h"
#include "lld_prcc_sta8090.h"
#endif

#include "svc_fsw.h" // needed for frequency switch

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#if defined( __STA2062__ )
#define SVC_MCU_PROD_NAME "STA2062"

#define SVC_MCU_MTU_NUMBER     2U
#define SVC_MCU_UART_NUMBER    4U
#define SVC_MCU_SSP_NUMBER     2U
#define SVC_MCU_SDI_NUMBER     2U
#define SVC_MCU_CAN_NUMBER     2U
#define SVC_MCU_I2C_NUMBER     3U
#define SVC_MCU_USB_NUMBER     2U
#define SVC_MCU_MSP_NUMBER     4U
#define SVC_MCU_GPIO_NUMBER    4U
#define SVC_MCU_ADC_NUMBER     1U
#elif defined( __STA2064__)
#define SVC_MCU_PROD_NAME "STA2064"

#define SVC_MCU_MTU_NUMBER     2U
#define SVC_MCU_UART_NUMBER    4U
#define SVC_MCU_SSP_NUMBER     2U
#define SVC_MCU_SDI_NUMBER     2U
#define SVC_MCU_CAN_NUMBER     1U
#define SVC_MCU_I2C_NUMBER     3U
#define SVC_MCU_USB_NUMBER     2U
#define SVC_MCU_MSP_NUMBER     4U
#define SVC_MCU_GPIO_NUMBER    4U
#define SVC_MCU_ADC_NUMBER     1U
#elif defined( __STA2065__)
#define SVC_MCU_PROD_NAME "STA2065"

#define SVC_MCU_MTU_NUMBER     2U
#define SVC_MCU_UART_NUMBER    4U
#define SVC_MCU_SSP_NUMBER     2U
#define SVC_MCU_SDI_NUMBER     2U
#define SVC_MCU_CAN_NUMBER     2U
#define SVC_MCU_I2C_NUMBER     3U
#define SVC_MCU_USB_NUMBER     2U
#define SVC_MCU_MSP_NUMBER     4U
#define SVC_MCU_GPIO_NUMBER    4U
#define SVC_MCU_ADC_NUMBER     1U
#elif defined( __STA8088__)
#define SVC_MCU_PROD_NAME "STA8088"

#define SVC_MCU_MTU_NUMBER     1U
#define SVC_MCU_UART_NUMBER    3U
#define SVC_MCU_SSP_NUMBER     1U
#define SVC_MCU_SDI_NUMBER     1U
#define SVC_MCU_CAN_NUMBER     2U
#define SVC_MCU_I2C_NUMBER     1U
#define SVC_MCU_USB_NUMBER     1U
#define SVC_MCU_MSP_NUMBER     1U
#define SVC_MCU_GPIO_NUMBER    2U
#define SVC_MCU_ADC_NUMBER     1U
#elif defined( __STA8090__)
#define SVC_MCU_PROD_NAME "STA8090"

#define SVC_MCU_MTU_NUMBER     2U
#define SVC_MCU_UART_NUMBER    3U
#define SVC_MCU_SSP_NUMBER     1U
#define SVC_MCU_SDI_NUMBER     1U
#define SVC_MCU_CAN_NUMBER     2U
#define SVC_MCU_I2C_NUMBER     1U
#define SVC_MCU_USB_NUMBER     1U
#define SVC_MCU_MSP_NUMBER     1U
#define SVC_MCU_GPIO_NUMBER    2U
#define SVC_MCU_ADC_NUMBER     1U
#else
#error "Unsupported device"
#endif

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

struct svc_mcu_busclk_s;

/**< Bus clock info type */
typedef struct svc_mcu_busclk_s
{
  struct svc_mcu_busclk_s *  next;
  tU32                        id;
  tU32                        freq;
} svc_mcu_busclk_t;

typedef struct
{
  gpOS_partition_t *     part;              /**< Memory partition used for dynamic allocated data */
  gpOS_semaphore_t *     access_sem;        /**< Access semaphore to service handler */
  svc_mcu_busclk_t *     busclk_head;       /**< Head of linked list of available bus clocks */
  svc_mcu_item_t *       item_head;         /**< Head of linked list of services */

  svc_mcu_corefreqcfg_t  mclkid;            /**< ID of main clock */
} svc_mcu_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

// MTU peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_mtu_phy_id[SVC_MCU_MTU_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)MTU0_REG_START_ADDR,
  (svc_mcu_addr_t)MTU1_REG_START_ADDR,
  #endif

  #if defined( __STA8088__ )
  (svc_mcu_addr_t)MTU_REG_START_ADDR,
  #endif

  #if defined( __STA8090__ )
  (svc_mcu_addr_t)MTU0_REG_START_ADDR,
  (svc_mcu_addr_t)MTU1_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_mtu_phy_irq_line[] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_MTU_0_LINE,
  (svc_mcu_irq_line_t)VIC_MTU_1_LINE,
  #endif

  #if defined( __STA8088__ )
  (svc_mcu_irq_line_t)VIC_MTU_LINE,
  #endif

  #if defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_MTU_LINE,
  #endif
};

// UART peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_uart_phy_id[SVC_MCU_UART_NUMBER] =
{
  (svc_mcu_addr_t)UART0_REG_START_ADDR,
  (svc_mcu_addr_t)UART1_REG_START_ADDR,
  (svc_mcu_addr_t)UART2_REG_START_ADDR,

  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)UART3_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_uart_phy_irq_line[] =
{
  (svc_mcu_irq_line_t)VIC_UART_0_LINE,
  (svc_mcu_irq_line_t)VIC_UART_1_LINE,
  (svc_mcu_irq_line_t)VIC_UART_2_LINE,

  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_UART_3_LINE
  #endif
};

// SSP peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_ssp_phy_id[SVC_MCU_SSP_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)SSP0_REG_START_ADDR,
  (svc_mcu_addr_t)SSP1_REG_START_ADDR
  #elif defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_addr_t)SSP_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_ssp_phy_irq_line[] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_SSP_0_LINE,
  (svc_mcu_irq_line_t)VIC_SSP_1_LINE
  #elif defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_SSP_LINE
  #endif
};

// SDI peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_sdi_phy_id[SVC_MCU_SDI_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)SD_SDIO_MMC0_REG_START_ADDR,
  (svc_mcu_addr_t)SD_SDIO_MMC1_REG_START_ADDR
  #elif defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_addr_t)SD_SDIO_MMC_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_sdi_phy_irq_line[SVC_MCU_SDI_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_SD_MM_CARD_0_LINE,
  (svc_mcu_irq_line_t)VIC_SD_MM_CARD_1_LINE
  #elif defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_SDMMC_0_LINE
  #endif
};

// CAN peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_can_phy_id[SVC_MCU_CAN_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2065__ ) || defined( __STA8088__ ) || defined( __STA8090__ )
  (svc_mcu_addr_t)CAN0_REG_START_ADDR,
  (svc_mcu_addr_t)CAN1_REG_START_ADDR
  #elif defined( __STA2064__ )
  (svc_mcu_addr_t)CAN1_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_can_phy_irq_line[SVC_MCU_CAN_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2065__ ) || defined( __STA8088__ ) || defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_CAN_0_LINE,
  (svc_mcu_irq_line_t)VIC_CAN_1_LINE
  #elif defined( __STA2064__ )
  (svc_mcu_irq_line_t)VIC_CAN_1_LINE
  #endif
};

// I2C peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_i2c_phy_id[SVC_MCU_I2C_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)I2C0_REG_START_ADDR,
  (svc_mcu_addr_t)I2C1_REG_START_ADDR,
  (svc_mcu_addr_t)I2C2_REG_START_ADDR
  #elif defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_addr_t)I2C_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_i2c_phy_irq_line[] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_I2C_0_LINE,
  (svc_mcu_irq_line_t)VIC_I2C_1_LINE,
  (svc_mcu_irq_line_t)VIC_I2C_2_LINE
  #elif defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_I2C_LINE
  #endif
};

// USB peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_usb_phy_id[SVC_MCU_USB_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)USB_OTG_FS_START_ADDR,
  (svc_mcu_addr_t)USB_OTG_HS_START_ADDR
  #elif defined( __STA8088__ ) || defined( __STA8090__ )
  (svc_mcu_addr_t)USB_OTG_FS_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_usb_phy_irq_line[] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_USB_OTG_LINE,
  (svc_mcu_irq_line_t)VIC_USB_MODEM_LINE
  #elif defined( __STA8088__ ) || defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_USB_OTG_LINE
  #endif
};

// MSP peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_msp_phy_id[SVC_MCU_MSP_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)MSP0_REG_START_ADDR,
  (svc_mcu_addr_t)MSP1_REG_START_ADDR,
  (svc_mcu_addr_t)MSP2_REG_START_ADDR,
  (svc_mcu_addr_t)MSP3_REG_START_ADDR
  #endif

  #if defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_addr_t)MSP_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_msp_phy_irq_line[] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_MSP_0_LINE,
  (svc_mcu_irq_line_t)VIC_MSP_1_LINE,
  (svc_mcu_irq_line_t)VIC_MSP_2_LINE,
  (svc_mcu_irq_line_t)VIC_MSP_3_LINE
  #endif

  #if defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_MSP_LINE
  #endif
};

// GPIO peripherals
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_gpio_phy_id[SVC_MCU_GPIO_NUMBER] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)GPIO0_REG_START_ADDR,
  (svc_mcu_addr_t)GPIO1_REG_START_ADDR,
  (svc_mcu_addr_t)GPIO2_REG_START_ADDR,
  (svc_mcu_addr_t)GPIO3_REG_START_ADDR
  #endif

  #if defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_addr_t)GPIO0_REG_START_ADDR,
  (svc_mcu_addr_t)GPIO1_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_gpio_phy_irq_line[] =
{
  #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_GPIO_0_LINE,
  (svc_mcu_irq_line_t)VIC_GPIO_1_LINE,
  (svc_mcu_irq_line_t)VIC_GPIO_2_LINE,
  (svc_mcu_irq_line_t)VIC_GPIO_3_LINE
  #endif

  #if defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_GPIO_0_LINE,
  (svc_mcu_irq_line_t)VIC_GPIO_1_LINE
  #endif
};

// ADC peripheral
static GENERIC_DATA_ISR const svc_mcu_addr_t svc_mcu_adc_phy_id[SVC_MCU_ADC_NUMBER] =
{
  #if defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_addr_t)ADC_REG_START_ADDR
  #endif

  #if defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_addr_t)TSC_REG_START_ADDR
  #endif
};

static GENERIC_DATA_ISR const svc_mcu_irq_line_t svc_mcu_adc_phy_irq_line[] =
{
  #if defined( __STA8088__) || defined( __STA8090__ )
  (svc_mcu_irq_line_t)VIC_ADC_LINE
  #endif

  #if defined( __STA2064__ ) || defined( __STA2065__ )
  (svc_mcu_irq_line_t)VIC_TSC_LINE
  #endif
};

// Service peripherals table
static GENERIC_DATA_ISR const svc_mcu_phy_item_t svc_mcu_phy_table[SVC_MCU_PER_ID_NUMBER] =
{
  { SVC_MCU_MTU_NUMBER   , svc_mcu_mtu_phy_id   , svc_mcu_mtu_phy_irq_line   },
  { SVC_MCU_UART_NUMBER  , svc_mcu_uart_phy_id  , svc_mcu_uart_phy_irq_line  },
  { 0                    , NULL                 , NULL                       },
  { SVC_MCU_SSP_NUMBER   , svc_mcu_ssp_phy_id   , svc_mcu_ssp_phy_irq_line   },
  { SVC_MCU_SDI_NUMBER   , svc_mcu_sdi_phy_id   , svc_mcu_sdi_phy_irq_line   },
  { SVC_MCU_CAN_NUMBER   , svc_mcu_can_phy_id   , svc_mcu_can_phy_irq_line   },
  { SVC_MCU_I2C_NUMBER   , svc_mcu_i2c_phy_id   , svc_mcu_i2c_phy_irq_line   },
  { SVC_MCU_USB_NUMBER   , svc_mcu_usb_phy_id   , svc_mcu_usb_phy_irq_line   },
  { SVC_MCU_MSP_NUMBER   , svc_mcu_msp_phy_id   , svc_mcu_msp_phy_irq_line   },
  { SVC_MCU_GPIO_NUMBER  , svc_mcu_gpio_phy_id  , svc_mcu_gpio_phy_irq_line  },
  { SVC_MCU_ADC_NUMBER   , svc_mcu_adc_phy_id   , svc_mcu_adc_phy_irq_line  }
};

static const tChar *svc_mcu_prod_name = SVC_MCU_PROD_NAME;

// Service handler
static svc_mcu_handler_t *svc_mcu_handler = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/*************************************************//**
 * \brief   Get pointer to a given given peripheral
 *
 * \param   per_id  Peripheral ID
 * \return  pointer to item if exists, NULL otherwise
 *
 *****************************************************/
static svc_mcu_item_t *svc_mcu_lookup_item( svc_mcu_per_id_t per_id)
{
  svc_mcu_item_t *svc_mcu_item_ptr = NULL;

  if( (svc_mcu_handler != NULL) && ( per_id < SVC_MCU_PER_ID_NUMBER))
  {
    svc_mcu_item_ptr = svc_mcu_handler->item_head;

    while( svc_mcu_item_ptr != NULL)
    {
      if( svc_mcu_item_ptr->srv_id == per_id)
      {
        break;
      }
      else
      {
        svc_mcu_item_ptr = svc_mcu_item_ptr->next;
      }
    }
  }
  return svc_mcu_item_ptr;
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief   Initialize services handler
 *
 * \param   part  Partition to use for memory allocation
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_init( gpOS_partition_t *part, const svc_mcu_corefreqcfg_t mclkid)
{
  gpOS_error_t error = gpOS_FAILURE;

  if( svc_mcu_handler != NULL)
  {
    return gpOS_SUCCESS;
  }

  svc_mcu_handler = gpOS_memory_allocate_p( part, sizeof( svc_mcu_handler_t));

  if( svc_mcu_handler != NULL)
  {
    svc_mcu_handler->access_sem = gpOS_semaphore_create( SEM_FIFO, 0 );

    if( svc_mcu_handler->access_sem == NULL)
    {
      gpOS_memory_deallocate_p( part, svc_mcu_handler);
    }
    else
    {
      svc_mcu_handler->part         = part;
      svc_mcu_handler->busclk_head  = NULL;
      svc_mcu_handler->item_head    = NULL;
      svc_mcu_handler->mclkid       = mclkid;

      gpOS_semaphore_signal( svc_mcu_handler->access_sem);

      error = gpOS_SUCCESS;
    }
  }

  return error;
}

/********************************************//**
 * \brief   Get number of available ports for a given peripheral
 *
 * \param   per_id  Peripheral ID
 * \return  Number of available ports
 *
 ***********************************************/
tUInt svc_mcu_get_ports( svc_mcu_per_id_t per_id)
{
  tUInt ports = 0;

  if(  per_id < SVC_MCU_PER_ID_NUMBER)
  {
    ports = svc_mcu_phy_table[per_id].number;
  }

  return ports;
}

/********************************************//**
 * \brief   Get memory mapped address of a port of a
 *          specific peripheral
 *
 * \param   per_id    Peripheral ID
 * \param   port_num  Port
 * \return  Memory mapped address of port
 *
 ***********************************************/
GENERIC_CODE_ISR tUInt svc_mcu_get_addr( svc_mcu_per_id_t per_id, tUInt port_num)
{
  tUInt addr = 0;

  if( (per_id < SVC_MCU_PER_ID_NUMBER) && (port_num < svc_mcu_phy_table[per_id].number))
  {
    addr = svc_mcu_phy_table[per_id].addr[port_num];
  }

  return addr;
}

/********************************************//**
 * \brief   Get IRQ line of a port of a specific peripheral
 *
 * \param   per_id    Peripheral ID
 * \param   port_num  Port
 * \return  IRQ line of port
 *
 ***********************************************/
tUInt svc_mcu_get_irq_line( svc_mcu_per_id_t per_id, tUInt port_num)
{
  tUInt irq_line = 0;

  if( (per_id < SVC_MCU_PER_ID_NUMBER) && (port_num < svc_mcu_phy_table[per_id].number))
  {
    irq_line = svc_mcu_phy_table[per_id].irq_line[port_num];
  }

  return irq_line;
}

/********************************************//**
 * \brief   Enable a port of a specific peripheral
 *
 * \param   per_id    Peripheral ID
 * \param   port_num  Port to enable
 * \return  void
 *
 ***********************************************/
void svc_mcu_enable( svc_mcu_per_id_t per_id, tUInt port_num)
{
  switch( per_id)
  {
    case SVC_MCU_PER_ID_MTU:
      if( port_num < SVC_MCU_MTU_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
        #elif defined( __STA8088__)
        LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_MTU);
        #elif defined( __STA8090__)
        switch( port_num )
        {
          case 0:
            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_MTU0);
            break;

          case 1:
            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_MTU1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #else
        #error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_UART:
      if( port_num < SVC_MCU_UART_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
        switch( port_num)
        {
          case 0:
            LLD_UART_Enable( UART0_REG_START_ADDR);
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART0);
            break;

          case 1:
            LLD_UART_Enable( UART1_REG_START_ADDR);
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART1);
            break;

          case 2:
            LLD_UART_Enable( UART2_REG_START_ADDR);
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART2);
            break;

          case 3:
            LLD_UART_Enable( UART3_REG_START_ADDR);
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART3);

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8088__)
        switch( port_num)
        {
          case 0:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART0);
            LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_UART0);
            break;

          case 1:
            LLD_APB2_SetUART1USBPins( LLD_APB2_UART1USBPINS_UART1);
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART1);
            LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_UART1);
            LLD_UART_TestModeDisable( (LLD_UART_IdTy)UART1_REG_START_ADDR);
            break;

          case 2:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART2);
            LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_UART2);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8090__)
        switch( port_num)
        {
          case 0:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART0);
            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_UART0);
            break;

          case 1:
            LLD_PRCC_SetUART1USBPins( LLD_PRCC_UART1USBPINS_UART1);
            //LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART1);
            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_UART1);
            LLD_UART_TestModeDisable( (LLD_UART_IdTy)UART1_REG_START_ADDR);
            break;

          case 2:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART2);
            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_UART2);
            break;

          default:
            /* Should not come here */
            break;
        }
        #else
        #error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_UART_HW_FLOW_CTRL:
      if( port_num < SVC_MCU_UART_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
        switch( port_num)
        {
          case 0:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART0_HW_FLOW_CTRL);
            break;

          case 1:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART1_HW_FLOW_CTRL);
            break;

          case 2:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART2_HW_FLOW_CTRL);
            break;

          //#if defined( __STA2065__ )
          case 3:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART3_HW_FLOW_CTRL);
          //#endif

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8088__)
        switch( port_num)
        {
          case 0:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART0_HW_FLOW_CTRL);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8090__)
        switch( port_num)
        {
          case 0:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_UART0_HW_FLOW_CTRL);
            break;

          default:
            /* Should not come here */
            break;
        }
        #else
        #error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_SSP:
      if( port_num < SVC_MCU_SSP_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
        switch( port_num)
        {
          case 0:
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_SSP0);
            LLD_SRC_EnableClk( LLD_SRC_PERCLKEN_SSP0);
            break;

          case 1:
            LLD_SRC_EnableClk( LLD_SRC_PERCLKEN_SSP1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8088__)
        LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_SSP);
        #elif defined( __STA8090__)
        LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_SSP);
        #else
        #error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_SDI:
      if( port_num < SVC_MCU_SDI_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
        switch( port_num)
        {
          case 0:
            LLD_SRC_EnableClk( LLD_SRC_PERCLKEN_SDI0);
            break;

          case 1:
            LLD_SRC_EnableClk( LLD_SRC_PERCLKEN_SDI1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8088__)
        LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_SDMC);

        LLD_GPIO_SetControlMode((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, (LLD_GPIO_PinTy)(LLD_GPIO_PIN2 | LLD_GPIO_PIN3 | LLD_GPIO_PIN4 |
                                                       LLD_GPIO_PIN5 | LLD_GPIO_PIN6 | LLD_GPIO_PIN7), LLD_GPIO_ALTERNATE_MODE_A);
        LLD_GPIO_SetDirectionOutput((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, (LLD_GPIO_PinTy)(LLD_GPIO_PIN2 | LLD_GPIO_PIN3 | LLD_GPIO_PIN4 |
                                                           LLD_GPIO_PIN5 | LLD_GPIO_PIN6 | LLD_GPIO_PIN7));
        LLD_GPIO_SetStateHigh((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, (LLD_GPIO_PinTy)(LLD_GPIO_PIN2 | LLD_GPIO_PIN3 | LLD_GPIO_PIN4 |
                                                     LLD_GPIO_PIN5 | LLD_GPIO_PIN6 | LLD_GPIO_PIN7));
        LLD_GPIO_SetDirectionOutput((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN1);
        #elif defined( __STA8090__)
        LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_SDMC);
        LLD_GPIO_SetSleepModeReg((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, 0xF00030, LLD_GPIO_DRIVEN_SLEEP);
        LLD_GPIO_DisablePull((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, 0xF00030);
        #else
        #error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_CAN:
      if( port_num < SVC_MCU_CAN_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
        #elif defined( __STA8088__)
        switch( port_num)
        {
          case 0:
            LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_CAN0);
            break;

          case 1:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_I2C);
            LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_CAN1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8090__)
        switch( port_num)
        {
          case 0:
            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_CAN0);
            break;

          case 1:
            //LLD_GPIO_DisAltFunction( LLD_GPIO_AF_I2C);
            LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN8, LLD_GPIO_ALTERNATE_MODE_B);
            LLD_GPIO_SetDirection  ( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN8, LLD_GPIO_OUTPUT);

            LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN9, LLD_GPIO_ALTERNATE_MODE_B);
            LLD_GPIO_SetDirection  ( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN9, LLD_GPIO_INPUT);

            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_CAN1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #endif
      }
      break;

    case SVC_MCU_PER_ID_I2C:
      if( port_num < SVC_MCU_I2C_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ )
        switch( port_num)
        {
          case 0:
            //
            LLD_SRC_EnableClk( LLD_SRC_PERCLKEN_I2C0);
            LLD_GPIO_EnAltFunction( LLD_GPIO_AF_I2C0);
            break;

          case 1:
            //
            LLD_SRC_EnableClk( LLD_SRC_PERCLKEN_I2C1);
            break;

          case 2:
            //
            LLD_SRC_EnableClk( LLD_SRC_PERCLKEN_I2C2);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8088__)
        LLD_GPIO_EnAltFunction( LLD_GPIO_AF_I2C);
        LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_I2C);
        #elif defined( __STA8090__)
        LLD_GPIO_EnAltFunction( LLD_GPIO_AF_I2C);
        LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_I2C);
        #else
        #error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_USB:
      if( port_num < SVC_MCU_USB_NUMBER)
      {
        #if defined( __STA8088__)
        LLD_CLK_CTRL_EnablePeripheral(LLD_CLK_CTRL_PERIPHID_USB);

        LLD_APB2_SetUSBCfg( TRUE);
        LLD_APB2_SetUART1USBPins( LLD_APB2_UART1USBPINS_USB);
        #elif defined( __STA8090__)
        LLD_PRCC_EnablePeripheral(LLD_PRCC_PERIPHID_USB);

        LLD_PRCC_SetUSBCfg( TRUE);
        LLD_PRCC_SetUART1USBPins( LLD_PRCC_UART1USBPINS_USB);
        #endif
      }
      break;

    case SVC_MCU_PER_ID_MSP:
      if( port_num < SVC_MCU_MSP_NUMBER)
      {
        #if defined( __STA8088__)
        LLD_GPIO_EnAltFunction( LLD_GPIO_AF_MSP);

        LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_MSP);
        #elif defined( __STA8090__)
        //LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN16, LLD_GPIO_ALTERNATE_MODE_C);
        //LLD_GPIO_SetDirectionInput((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, (LLD_GPIO_PinTy)LLD_GPIO_PIN16);
        //LLD_GPIO_SetDirectionOutput((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, (LLD_GPIO_PinTy)(LLD_GPIO_PIN29 | LLD_GPIO_PIN30 | LLD_GPIO_PIN31));

        LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_MSP);
        #else
        //#error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_GPIO:
      if( port_num < SVC_MCU_GPIO_NUMBER)
      {
        #if defined( __STA8088__)
        switch( port_num )
        {
          case 0:
            LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_GPIO0);
            break;

          case 1:
            LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_GPIO1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8090__)
        switch( port_num )
        {
          case 0:
            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_GPIO0);
            break;

          case 1:
            LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_GPIO1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #else
        //#error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_ADC:
      if( port_num < SVC_MCU_ADC_NUMBER)
      {
        #if defined( __STA8088__)
        LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_ADC);
        #elif defined( __STA8090__)
        LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_ADC);
        #endif
      }
      break;

    default:
      /* Should not come here */
      break;
  }
}

/********************************************//**
 * \brief   Disable a port of a specific peripheral
 *
 * \param   per_id    Peripheral ID
 * \param   port_num  Port to disable
 * \return  void
 *
 ***********************************************/
void svc_mcu_disable( svc_mcu_per_id_t per_id, tUInt port_num)
{
  switch( per_id)
  {
    case SVC_MCU_PER_ID_MTU:
      if( port_num < SVC_MCU_MTU_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
        #elif defined( __STA8088__)
        LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_MTU);
        #elif defined( __STA8090__)
        switch( port_num )
        {
          case 0:
            LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_MTU0);
            break;

          case 1:
            LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_MTU1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #else
        #error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_UART:
      if( port_num < SVC_MCU_UART_NUMBER)
      {
        #if defined( __STA2062__ ) || defined( __STA2064__ ) || defined( __STA2065__ )
        switch( port_num)
        {
          case 0:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART0);
            LLD_UART_Disable( UART0_REG_START_ADDR);
            break;

          case 1:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART1);
            LLD_UART_Disable( UART1_REG_START_ADDR);
            break;

          case 2:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART2);
            LLD_UART_Disable( UART2_REG_START_ADDR);
            break;

          case 3:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART3);
            LLD_UART_Disable( UART3_REG_START_ADDR);

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8088__)
        switch( port_num)
        {
          case 0:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART0);
            LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_UART0);
            break;

          case 1:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART1);
            LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_UART1);
            break;

          case 2:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART2);
            LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_UART2);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8090__)
        switch( port_num)
        {
          case 0:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART0);
            LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_UART0);
            break;

          case 1:
            //LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART1);
            LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_UART1);
            break;

          case 2:
            LLD_GPIO_DisAltFunction( LLD_GPIO_AF_UART2);
            LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_UART2);
            break;

          default:
            /* Should not come here */
            break;
        }
        #else
        #error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_USB:
      if( port_num < SVC_MCU_USB_NUMBER)
      {
        #if defined( __STA8088__)
        LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_UART1);
        LLD_UART_TestModeEnable( (LLD_UART_IdTy)UART1_REG_START_ADDR);
        LLD_UART_TestModeSet( (LLD_UART_IdTy)UART1_REG_START_ADDR, 0);
        LLD_CLK_CTRL_PeripheralClkDis( LLD_CLK_CTRL_PERIPHID_UART1);

        LLD_APB2_SetUART1USBPins( LLD_APB2_UART1USBPINS_UART1);
        LLD_APB2_SetUSBCfg( FALSE);

        LLD_CLK_CTRL_DisablePeripheral(LLD_CLK_CTRL_PERIPHID_USB);
        #elif defined( __STA8090__)
        LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_UART1);
        LLD_UART_TestModeEnable( (LLD_UART_IdTy)UART1_REG_START_ADDR);
        LLD_UART_TestModeSet( (LLD_UART_IdTy)UART1_REG_START_ADDR, 0);
        LLD_PRCC_PeripheralClkDis( LLD_PRCC_PERIPHID_UART1);

        LLD_PRCC_SetUART1USBPins( LLD_PRCC_UART1USBPINS_UART1);
        LLD_PRCC_SetUSBCfg( FALSE);

        LLD_PRCC_DisablePeripheral(LLD_PRCC_PERIPHID_USB);
        // Check for other bits to be configured
        #endif
      }
      break;

    case SVC_MCU_PER_ID_MSP:
      if( port_num < SVC_MCU_MSP_NUMBER)
      {
        #if defined( __STA8088__)
        LLD_GPIO_DisAltFunction( LLD_GPIO_AF_MSP);

        LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_MSP);
        #elif defined( __STA8090__)
        //LLD_GPIO_SetControlMode( (LLD_GPIO_idTy)GPIO0_REG_START_ADDR, LLD_GPIO_PIN16, LLD_GPIO_ALTERNATE_MODE_C);
        //LLD_GPIO_SetDirectionInput((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, (LLD_GPIO_PinTy)LLD_GPIO_PIN16);
        //LLD_GPIO_SetDirectionOutput((LLD_GPIO_idTy)GPIO0_REG_START_ADDR, (LLD_GPIO_PinTy)(LLD_GPIO_PIN29 | LLD_GPIO_PIN30 | LLD_GPIO_PIN31));

        LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_MSP);
        #else
        //#error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_GPIO:
      if( port_num < SVC_MCU_GPIO_NUMBER)
      {
        #if defined( __STA8088__)
        switch( port_num )
        {
          case 0:
            LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_GPIO0);
            break;

          case 1:
            LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_GPIO1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #elif defined( __STA8090__)
        switch( port_num )
        {
          case 0:
            LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_GPIO0);
            break;

          case 1:
            LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_GPIO1);
            break;

          default:
            /* Should not come here */
            break;
        }
        #else
        //#error "Undefined micro"
        #endif
      }
      break;

    case SVC_MCU_PER_ID_ADC:
      if( port_num < SVC_MCU_ADC_NUMBER)
      {
        #if defined( __STA8088__)
        LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_ADC);
        #endif
      }
      break;

    default:
      /* Should not come here */
      break;
  }
}

/********************************************//**
 * \brief   Enter WFI (Wait for interrupt) status by
 *          calling a SWI.
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void svc_mcu_enter_wfi( void)
{
  #if defined( __STA2064__) || defined( __STA2065__)
  gpOS_kernel_user_system_call( (gpOS_syscall_func_t)LLD_ARM1176_EnterWFI, NULL);
  #endif

  #if defined( __STA8088__)
  gpOS_kernel_user_system_call( (gpOS_syscall_func_t)LLD_ARM946_EnterWFI, NULL);
  #endif

  #if defined( __STA8090__) && defined( RCIF_OVER_UART)
  gpOS_kernel_user_system_call( (gpOS_syscall_func_t)LLD_ARM946_EnterWFI, NULL);
  #endif

  #if defined( __STA8090__) && !defined( RCIF_OVER_UART)
  gpOS_kernel_user_system_call( (gpOS_syscall_func_t)svc_mcu_exec_wfi, NULL);
  #endif
}

#if defined( __STA8090__) && !defined( RCIF_OVER_UART)
/********************************************//**
 * \brief   Enter the WFI (Wait for interrupt) status by
 *          calling the LLD ARM.
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void GENERIC_CODE_ISR svc_mcu_exec_wfi( const gpOS_syscall_param_t param)
{
  boolean_t standby_status;

  if((svc_pwr_get_standby_status(&standby_status) == gpOS_SUCCESS)&&(standby_status == TRUE))
  {
    svc_pwr_enter_standby();
  }

  /* For any reason we couldn't enter STANDBY, continue with WFI */
  {
    svc_pwr_pre_WFI();

    LLD_ARM946_EnterWFI();

    svc_pwr_post_WFI();
  }
}
#endif

/********************************************//**
 * \brief   Generate SW reset
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
void svc_mcu_sw_reset( void)
{
  #if defined( __STA8088__)
  LLD_CLK_CTRL_GenerateSoftReset();
  #endif
  #if defined( __STA8090__)
  /* Depending on Cut version, need to power ON explicitly LDO2 */
  LLD_PRCC_LDOEnable(LLD_PRCC_LDO2);
  LLD_PRCC_GenerateSoftReset();
  #endif
}

/********************************************//**
 * \brief   Get ROM version of micro
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
const tChar *svc_mcu_getprodname( void)
{
  return svc_mcu_prod_name;
}

/********************************************//**
 * \brief   Get ROM version of micro
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
tUInt svc_mcu_getromver( void)
{
  tUInt rom_ver = 0;

  #if defined( __STA8088__)
  rom_ver = *( ( tUInt * )LLD_CLK_CTRL_ROMT2VERSION_ADDR );
  #endif
  #if defined( __STA8090__)
  rom_ver = *( ( tUInt * )LLD_PRCC_ROMT3VERSION_ADDR );
  #endif

  return rom_ver;
}

/********************************************//**
 * \brief   Get CPU idle time
 *
 * \param   void
 * \return  void
 *
 ***********************************************/
tUInt svc_mcu_getcpuusage( void)
{
  tUInt cpu_usage;

  #if defined( FREERTOS )
  cpu_usage = get_CPU_usage();
  #else
  extern gpOS_task_t *os20_idle_task;
  cpu_usage = 10000U - gpOS_task_get_cpuusage( os20_idle_task );
  #endif // FREERTOS

  return cpu_usage;
}


/********************************************//**
 * \brief   Install a new peripheral in service module and
 *          initialize it
 *
 * \param   per_id  Peripheral ID
 * \param   busclk_id  Bus ID
 * \param   item_ptr  Service item pointer
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_install( const svc_mcu_per_id_t per_id, const tU32 busclk_id, svc_mcu_item_t *item_ptr)
{
  gpOS_error_t error = gpOS_SUCCESS;

  // OS20 Services not initialized or peripheral id not valid or item not created, return error
  if( (svc_mcu_handler == NULL) || ( per_id >= SVC_MCU_PER_ID_NUMBER) || (item_ptr == NULL))
  {
    return gpOS_FAILURE;
  }

  // Peripheral already installed, return failure
  if( svc_mcu_lookup_item( per_id) != NULL)
  {
    return gpOS_FAILURE;
  }

  // Update services handler structure
  gpOS_semaphore_wait( svc_mcu_handler->access_sem);

  item_ptr->srv_id    = per_id;
  item_ptr->bus_id    = busclk_id;
  item_ptr->phy_item  = &svc_mcu_phy_table[per_id];
  item_ptr->cmdif     = NULL;

  // Update linked list
  item_ptr->next          = svc_mcu_handler->item_head;
  svc_mcu_handler->item_head  = item_ptr;

  gpOS_semaphore_signal( svc_mcu_handler->access_sem);

  return error;
}

/********************************************//**
 * \brief   Uninstall a  peripheral from service module
 *
 * \param   per_id    Peripheral ID
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_uninstall( const svc_mcu_per_id_t per_id)
{
  gpOS_error_t error = gpOS_FAILURE;
  /*svc_mcu_item_t *last = NULL;*/
  /*svc_mcu_item_t *curr = svc_mcu_handler->item_head;*/

  if( svc_mcu_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  if( (svc_mcu_handler == NULL) || ( per_id >= SVC_MCU_PER_ID_NUMBER))
  {
    error = gpOS_FAILURE;
  }

  // Lookup for item handler of per_id

  // Check for any open ports

  // Remove from linked list

  return error;
}

/********************************************//**
 * \brief   Set a bus clock configuration to service handler
 *
 * \param   id    Identifier for bus clock configuration
 * \param   freq  New frequency for this id. If the id is not present, a new one is created
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_busclk_set( const tU32 id, tU32 freq)
{
  svc_mcu_busclk_t *curr_busclk;
  gpOS_error_t error = gpOS_SUCCESS;

  if( svc_mcu_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  // Check if bus clock ID already exists
  curr_busclk = svc_mcu_handler->busclk_head;

  while( curr_busclk != NULL)
  {
    if( curr_busclk->id == id)
    {
#if defined( __STA8090__)
      if (id == svc_mcu_handler->mclkid)
      {
        freq = freq/2U;
      }
#endif
      if( curr_busclk->freq != freq)
      {
        // Bus clock id found, update it.
        curr_busclk->freq = freq;
      }
      break;
    }
    else
    {
      curr_busclk = curr_busclk->next;
    }
  }

  // Bus clock id not found, create a new one
  if( curr_busclk == NULL)
  {
    svc_mcu_busclk_t *new_busclk = gpOS_memory_allocate_p( svc_mcu_handler->part, sizeof( svc_mcu_busclk_t));

    if( new_busclk == NULL)
    {
      error = gpOS_FAILURE;
    }
    else
    {
      new_busclk->id    = id;
#if defined( __STA8090__)
      if (id == svc_mcu_handler->mclkid)
      {
        new_busclk->freq = freq/2U;
      }
      else
      {
        new_busclk->freq  = freq;
      }
#else
      new_busclk->freq  = freq;
#endif

      new_busclk->next  = svc_mcu_handler->busclk_head;

      svc_mcu_handler->busclk_head = new_busclk;
    }
  }

  return( error);
}

/********************************************//**
 * \brief   Set a bus clock configuration to service handler
 *
 * \param   id    Identifier for bus clock configuration
 * \param   freq  New frequency for this id. If the id is not present, a new one is created
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_update_data_rates( const tU32 id)
{
  gpOS_error_t error = gpOS_SUCCESS;
  svc_mcu_item_t *svc_mcu_item_ptr;

  if( svc_mcu_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  svc_mcu_item_ptr = svc_mcu_handler->item_head;

  while( svc_mcu_item_ptr != NULL)
  {
    if( svc_mcu_item_ptr->bus_id == id)
    {
      // Bus clock id found, update peripheral data rate
      if( svc_mcu_item_ptr->cmdif != NULL)
      {
        svc_mcu_item_ptr->cmdif( SVC_MCU_CMD_ID_CHANGE_SPEED, NULL);
      }
      svc_mcu_item_ptr = svc_mcu_item_ptr->next;
    }
    else
    {
      svc_mcu_item_ptr = svc_mcu_item_ptr->next;
    }
  }
  return( error);
}

/********************************************//**
 * \brief   Set a bus clock configuration to service handler
 *
 * \param   id    Identifier for bus clock configuration
 * \param   freq  New frequency for this id. If the id is not present, a new one is created
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_check_data_rates_feasibility( const tU32 id, const tU32 freq)
{
  gpOS_error_t error = gpOS_SUCCESS;
  svc_mcu_item_t *svc_mcu_item_ptr;

  if( svc_mcu_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  svc_mcu_item_ptr = svc_mcu_handler->item_head;

  while( svc_mcu_item_ptr != NULL)
  {
    if( svc_mcu_item_ptr->bus_id == id)
    {
      // Bus clock id found, update peripheral data rate
      if( svc_mcu_item_ptr->cmdif != NULL)
      {
        svc_mcu_item_ptr->cmdif( SVC_MCU_CMD_ID_CHECK_DATA_RATE_CHANGE, (void*)freq);
      }
      svc_mcu_item_ptr = svc_mcu_item_ptr->next;
    }
    else
    {
      svc_mcu_item_ptr = svc_mcu_item_ptr->next;
    }
  }
  return( error);
}

/********************************************//**
 * \brief   Disnable transfer on the concerned peripherals
 *
 * \param   id    Identifier for bus clock configuration
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_suspend_transfers( const tU32 id)
{
  gpOS_error_t error = gpOS_SUCCESS;
  svc_mcu_item_t *svc_mcu_item_ptr;

  if( svc_mcu_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  svc_mcu_item_ptr = svc_mcu_handler->item_head;

  while( svc_mcu_item_ptr != NULL)
  {
    if( svc_mcu_item_ptr->bus_id == id)
    {
      // Bus clock id found, suspend transfer on this peripheral if callback function exists
      if( svc_mcu_item_ptr->cmdif != NULL)
      {
        svc_mcu_item_ptr->cmdif( SVC_MCU_CMD_ID_SUSPEND_TRANSFER, NULL);
      }
      svc_mcu_item_ptr = svc_mcu_item_ptr->next;
    }
    else
    {
      svc_mcu_item_ptr = svc_mcu_item_ptr->next;
    }
  }
  return( error);
}

/*******************************************************//**
 * \brief   Re-enable transfer on the concerned peripherals
 *
 * \param   id    Identifier for bus clock configuration
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_restore_transfers( const tU32 id)
{
  gpOS_error_t error = gpOS_SUCCESS;
  svc_mcu_item_t *svc_mcu_item_ptr;

  if( svc_mcu_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  svc_mcu_item_ptr = svc_mcu_handler->item_head;

  while( svc_mcu_item_ptr != NULL)
  {
    if( svc_mcu_item_ptr->bus_id == id)
    {
      // Bus clock id found, restore transfer on this peripheral if callback function exists
      if( svc_mcu_item_ptr->cmdif != NULL)
      {
        svc_mcu_item_ptr->cmdif( SVC_MCU_CMD_ID_RESTORE_TRANSFER, NULL);
      }
      svc_mcu_item_ptr = svc_mcu_item_ptr->next;
  }
    else
    {
      svc_mcu_item_ptr = svc_mcu_item_ptr->next;
    }
  }
  return( error);
}

/********************************************//**
 * \brief   Get a bus clock configuration to service handler
 *
 * \param   id        Identifier for bus clock configuration
 * \param   freq_ptr  Pointer to variable to store related frequency
 * \return  gpOS_SUCCESS if all ok, gpOS_FAILURE elsewhere
 *
 ***********************************************/
gpOS_error_t svc_mcu_busclk_get( const tU32 id, tU32 *freq_ptr)
{
  svc_mcu_busclk_t *curr_busclk;

  *freq_ptr = 0;

  if( svc_mcu_handler == NULL)
  {
    return gpOS_FAILURE;
  }

  curr_busclk = svc_mcu_handler->busclk_head;

  while( curr_busclk != NULL)
  {
    if( curr_busclk->id == id)
    {
      *freq_ptr = curr_busclk->freq;
      break;
    }
    else
    {
      curr_busclk = curr_busclk->next;
    }
  }

  if( curr_busclk == NULL)
  {
    return gpOS_FAILURE;
  }

  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief   Change the states of the clock
 *
 * \param   tU32 indicates the clock mode
 * \return  void
 *
 ***********************************************/
void svc_mcu_changeclockmode( tUInt mode)
{
  if (mode == 1U)
  {
  #if defined( __STA8088__)
    LLD_CLK_CTRL_SetStandbyMode();
  #endif
  #if defined( __STA8090__)
    LLD_PRCC_RFEnable();
  #endif
  }
  else if (mode == 2U)
  {
  #if defined( __STA8088__)
    LLD_CLK_CTRL_SetSleepMode();
  #endif
  #if defined( __STA8090__)
    LLD_PRCC_RFDisable();
  #endif
  }
  else
  {
    /* Empty */
  }
}

/********************************************//**
 * \brief   Allows to enable G3/G4 RF
 *
 * \param   tBool indicates if enable or not RF
 * \return  void
 *
 ***********************************************/
void svc_mcu_RFenable( tBool enable)
{
  if (enable == TRUE)
  {
  #if defined( __STA8088__)
    LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_G3);
    LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_G3_HCLK);
  #endif
  #if defined( __STA8090__)
    LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_G3EP);
    LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_G3EP_AHB);
  #endif
  }
  else
  {
  #if defined( __STA8088__)
    LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_G3_HCLK);
    LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_G3);
  #endif
  #if defined( __STA8090__)
    LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_G3EP);
    LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_G3EP_AHB);
  #endif
  }
}

/******************************************************//**
 * \brief   Several services to configure Ring Oscillator
 *
 * \param   tU32 choice of service
 * \param   tU32 1st parameter (various usage)
 * \param   tU32 2nd parameter (various usage)
 * \return  tU32
 *
 *********************************************************/
tUInt svc_mcu_setRIOSCconfig( tUInt choice, tUInt param1, tUInt param2)
{
  #if defined( __STA8090__)
  tU32 RIOSC_Freq;
  #endif

  if (choice == 0U)
  {
  #if defined( __STA8088__)
    LLD_CLK_CTRL_SetRingOscillator(false);
  #endif
  #if defined( __STA8090__)
    LLD_PRCC_SetRingOscillator(false);
  #endif
    return 0;/*lint !e904 Return statement before end of function */
  }
  else if (choice == 1U)
  {
  #if defined( __STA8088__)
    LLD_CLK_CTRL_SetRingOscillator(true);
  #endif
  #if defined( __STA8090__)
    LLD_PRCC_SetRingOscillator(true);
  #endif
    return 0;/*lint !e904 Return statement before end of function */
  }
  else if (choice == 2U)
  {
  #if defined( __STA8088__)
    return( LLD_CLK_CTRL_RunRIOSCCalibration(param1, param2));/*lint !e904 Return statement before end of function */
  #endif
  #if defined( __STA8090__)
    return( LLD_PRCC_RunRIOSCCalibration(param1, param2, &RIOSC_Freq));/*lint !e904 Return statement before end of function */
  #endif
  }
  else
  {
    /* Empty */
  }

  return 0;
}

/******************************************************//**
 * \brief   Returns Ring Oscillator frequency
 *
 * \param[out] tU32* trim_out value
 * \return     tU32 frequency read
 *
 *********************************************************/
tUInt svc_mcu_getRIOSCfrequency(tU32* trim_out)
{
  #if defined( __STA8088__)
    return( LLD_CLK_CTRL_GetPCLK_RIOSCFrequency(trim_out));
  #endif
  #if defined( __STA8090__)
    return ( LLD_PRCC_GetPCLK_RIOSCFrequency(trim_out));
  #endif
}

/******************************************************//**
 * \brief   Enable or Disable PLL
 *
 * \param   tBool indicates if enable or disable PLL
 * \return  void
 *
 *********************************************************/
tVoid svc_mcu_setPLL(tBool enable)
{
  #if defined( __STA8088__)
    LLD_CLK_CTRL_SetPLL( enable);
  #endif
  #if defined( __STA8090__)
  // Not needed
  #endif
}
// End of file
