//!
//!   \file     mapping_sta8090.h
//!   \brief    <i><b>STA8090 mapping header file</b></i>
//!   \author   Fulvio Boggia
//!   \version  1.0
//!   \date     2013.05.23
//!   \bug      Unknown
//!   \warning  None
//!

#ifndef MAPPING_STA8090_H
#define MAPPING_STA8090_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "lld.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

//--------------------------------------------------------------
// AHB address mapping
//--------------------------------------------------------------

#define ITCM_START_ADDR                 0x00000000    // I-TCM (always at 0)

#define DTCM_START_ADDR                 0x00100000    // D-TCM (logic addressing)

#define SQI_START_ADDR                  0x10000000    // SQI

#define FSMC_BANK0_START_ADDR           0x20000000    // FSMC Bank 0

#define FSMC_BANK1_START_ADDR           0x24000000    // FSMC Bank 1

#define FSMC_BANK2_START_ADDR           0x28000000    // FSMC Bank 2

#define ROM_START_ADDR                  0x30000000    // ROM

#define PATCH_ADDR_RAM_START_ADDR       0x30080000    // ROM patch adress RAM

#define PATCH_CODE_RAM_START_ADDR       0x30100000    // ROM patch code RAM

#define PATCH_CONFIG_START_ADDR         0x30180000    // ROM patch config

#define REC_PLAY_REG_START_ADDR         0x30200000    // REC_PLAY

#define BACKUP_RAM_START_ADDR           0x40000000    // Backup RAM

#define VIC_REG_START_ADDR              0x50000000    // VIC Controller

#define SQI_REG_START_ADDR              0x50010000    // SQI Controller

#define FSMC_REG_START_ADDR             0x50020000    // FSMC Controller

#define USB_OTG_FS_START_ADDR           0x60000000    // USB-OTG Full Speed

#define G3EP_REG_START_ADDR             0x70000000    // G3 Controller

#define G3EP_RAM_GALPRN_START_ADDR      0x70050000    // G3 Galileo Codes RAM

//--------------------------------------------------------------
// APB peripheral address mapping
//--------------------------------------------------------------
// APB Bridge 1

#define PRCC_BCK_REG_START_ADDR         0x51000000    // Power, Reset and Clock Controller on backup

#define RTC_RTT_PWL_REG_START_ADDR      0x51001000    // RTC and RTT registers

// APB Bridge 2

#define WDG_REG_START_ADDR              0x52000000    // Watchdog Timer

#define EFT_REG_START_ADDR              0x52001000    // Extended Function Timer

#define SD_SDIO_MMC_REG_START_ADDR      0x52002000    // Secure Digital Card, SD IO, Multi-Media Card

#define SSP_REG_START_ADDR              0x52003000    // Synchronous Serial Port

#define MTU0_REG_START_ADDR             0x52004000    // Multi Timer Unit

#define ADC_REG_START_ADDR              0x52005000    // Analog/digital converter

#define MSP_REG_START_ADDR              0x52006000    // Multi Serial Port port

#define I2C_REG_START_ADDR              0x52007000    // Inter-Integrated Circuit

#define GPIO0_REG_START_ADDR            0x52008000    // General Purpose I/Os Bank 0

#define GPIO1_REG_START_ADDR            0x52009000    // General Purpose I/Os Bank 1

#define UART0_REG_START_ADDR            0x5200A000    // Asynchronous Serial Port 0

#define UART1_REG_START_ADDR            0x5200B000    // Asynchronous Serial Port 1

#define UART2_REG_START_ADDR            0x5200C000    // Asynchronous Serial Port 2

#define PRCC_REG_START_ADDR             0x5200F000    // Power, Reset and Clock Controller

#define MTU1_REG_START_ADDR             0x52014000    // Multi Timer Unit

// APB Bridge 3

#define CAN0_REG_START_ADDR             0x5200D000    // Controller Area Network 0

#define CAN1_REG_START_ADDR             0x5200E000    // Controller Area Network 1

//--------------------------------------------------------------
// STA8088 TCM specific constants
//--------------------------------------------------------------

#define ITCM_SIZE_MIN                   0x00004000    // Minimum ITCM size
#define DTCM_SIZE_MIN                   0x00020000    // Minimum DTCM size
#define TCM_BLOCK_SIZE                  0x00004000    // TCM block size

#define GPIO_TOTAL_PINS_NUMBER          48U

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//------------------------------------------------------------------------
// Vectored interrupt controller (VIC)
//------------------------------------------------------------------------
typedef enum
{
    VIC_WATCHDOG_LINE,      /*  00   */
    VIC_MTU0_TIM0_LINE,     /*  01   */
    VIC_MTU0_TIM1_LINE,     /*  02   */
    VIC_MTU0_TIM2_LINE,     /*  03   */
    VIC_MTU0_TIM3_LINE,     /*  04   */
    VIC_GPIO_0_LINE,        /*  05   */
    VIC_GPIO_1_LINE,        /*  06   */
    VIC_UART_0_RX_LINE,     /*  07   */
    VIC_G3EP_IREQ_LINE,     /*  08   */
    VIC_G3EP_AIREQ0_LINE,   /*  09   */
    VIC_G3EP_AIREQ1_LINE,   /*  10   */
    VIC_RTC_LINE,           /*  11   */
    VIC_RTT_LINE,           /*  12   */
    VIC_REC_PLAY_GNS_LINE,  /*  13   */
    VIC_REC_PLAY_GAL_LINE,  /*  14   */
    VIC_PRCC_LINE,          /*  15   */
    VIC_DTCM_ERROR_LINE,    /*  16   */
    VIC_ITCM_ERROR_LINE,    /*  17   */
    VIC_MTU1_TIM0_LINE,     /*  18   */
    VIC_MTU1_TIM1_LINE,     /*  19   */
    VIC_USB_OTG_LINE,       /*  20   */
    VIC_EFT_LINE,           /*  21   */
    VIC_SDMMC_0_LINE,       /*  22   */
    VIC_SDMMC_1_LINE,       /*  23   */
    VIC_SSP_LINE,           /*  24   */
    VIC_SSP_RX_LINE,        /*  25   */
    VIC_SSP_TX_LINE,        /*  26   */
    VIC_SSP_ROR_LINE,       /*  27   */
    VIC_SSP_RT_LINE,        /*  28   */
    VIC_MTU_LINE,           /*  29   */
    VIC_ADC_LINE,           /*  30   */
    VIC_MSP_LINE,           /*  31   */
    VIC_MSP_RX_LINE,        /*  32   */
    VIC_MSP_TX_LINE,        /*  33   */
    VIC_REC_PLAY_LINE,      /*  34   */
    VIC_I2C_LINE,           /*  35   */
    VIC_UART_0_MS_LINE,     /*  36   */
    VIC_UART_0_TX_LINE,     /*  37   */
    VIC_UART_0_RT_LINE,     /*  38   */
    VIC_UART_0_E_LINE,      /*  39   */
    VIC_UART_0_LINE,        /*  40   */
    VIC_UART_0_ABDONE_LINE, /*  41   */
    VIC_UART_0_ABERR_LINE,  /*  42   */
    VIC_UART_1_MS_LINE,     /*  43   */
    VIC_UART_1_RX_LINE,     /*  44   */
    VIC_UART_1_TX_LINE,     /*  45   */
    VIC_UART_1_RT_LINE,     /*  46   */
    VIC_UART_1_E_LINE,      /*  47   */
    VIC_UART_1_LINE,        /*  48   */
    VIC_UART_1_ABDONE_LINE, /*  49   */
    VIC_UART_1_ABERR_LINE,  /*  50   */
    VIC_UART_2_MS_LINE,     /*  51   */
    VIC_UART_2_RX_LINE,     /*  52   */
    VIC_UART_2_TX_LINE,     /*  53   */
    VIC_UART_2_RT_LINE,     /*  54   */
    VIC_UART_2_E_LINE,      /*  55   */
    VIC_UART_2_LINE,        /*  56   */
    VIC_UART_2_ABDONE_LINE, /*  57   */
    VIC_UART_2_ABERR_LINE,  /*  58   */
    VIC_CAN_0_LINE,         /*  59   */
    VIC_CAN_1_LINE,         /*  60   */
    VIC_SQI_LINE,           /*  61   */
    VIC_MTU1_TIM2_LINE,     /*  62   */
    VIC_MTU1_TIM3_LINE,     /*  63   */
    VIC_NO_LINE,            /*  None */
    VIC_ALL_LINES

} VicLineTy;

typedef enum
{
    PERIPHID_ADC,
    PERIPHID_CAN0,
    PERIPHID_CAN1,
    PERIPHID_EFT,
    PERIPHID_FSMC,
    PERIPHID_GPIO0,
    PERIPHID_GPIO1,
    PERIPHID_I2C,
    PERIPHID_MSP,
    PERIPHID_MTU0,
    PERIPHID_MTU1,
    PERIPHID_PRCC,
    PERIPHID_RECPLAY,
    PERIPHID_RTCRTT,
    PERIPHID_SDI,
    PERIPHID_SQI,
    PERIPHID_SSP,
    PERIPHID_UART0,
    PERIPHID_UART1,
    PERIPHID_UART2,
    PERIPHID_USB,
    PERIPHID_VIC,
    PERIPHID_WDG
} PeriphIdTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif // MAPPING_STA8090_H

// End of file - mapping_sta8090.h
