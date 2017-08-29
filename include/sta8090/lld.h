//!
//!   \file     lld.h
//!   \brief    <i><b>Include all basic interfaces for the LLD</b></i>
//!   \author   Fulvio Boggia
//!   \author   (original version) Alberto Saviotti, Luca Pesenti
//!   \version  1.0
//!   \date     2010.09.01
//!   \bug      Unknown
//!   \warning  This header shall be included by all LLDs
//!

#ifndef LLD_H
#define LLD_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "defines.h"
#include "macros.h"
#include "typedefs.h"
#include "gpOS_interrupt.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

//! \brief LLD ISR type
#if (__ARMCC_VERSION >= 300586) || defined( __GNUC__)
#define LLD_ISR_ADC             __attribute__((section ("ADC_ISR_REGION")))
#define LLD_ISR_CAN             __attribute__((section ("CAN_ISR_REGION")))
#define LLD_ISR_EFT             __attribute__((section ("EFT_ISR_REGION")))
#define LLD_ISR_GNSSDSP         __attribute__((section ("GNSSDSP_ISR_REGION")))
#define LLD_ISR_GPIO            __attribute__((section ("GPIO_ISR_REGION")))
#define LLD_ISR_I2C             __attribute__((section ("I2C_ISR_REGION")))
#define LLD_ISR_MSP             __attribute__((section ("MSP_ISR_REGION")))
#define LLD_ISR_MTU             __attribute__((section ("MTU_ISR_REGION")))
#define LLD_ISR_SDI             __attribute__((section ("SDI_ISR_REGION")))
#define LLD_ISR_SSP             __attribute__((section ("SSP_ISR_REGION")))
#define LLD_ISR_UART            __attribute__((section ("UART_ISR_REGION")))
#define LLD_ISR_USB             __attribute__((section ("USB_ISR_REGION")))
#define LLD_ISR_VIC             __attribute__((section ("VIC_ISR_REGION")))
#else
#define LLD_ISR_ADC
#define LLD_ISR_CAN
#define LLD_ISR_EFT
#define LLD_ISR_GNSSDSP
#define LLD_ISR_GPIO
#define LLD_ISR_I2C
#define LLD_ISR_MSP
#define LLD_ISR_MTU
#define LLD_ISR_SDI
#define LLD_ISR_SSP
#define LLD_ISR_USB
#define LLD_ISR_UART
#define LLD_ISR_VIC
#endif

/*lint -e9021 : use of '#undef' is discouraged  [MISRA 2012 Rule 20.5, advisory]" */
#undef LLD_UART_VER_5_1_2_0
#undef LLD_UART_VER_5_1_10_0
#undef LLD_I2C_DMASUPPORT
#undef LLD_I2C_ICSUPPORT
#undef LLD_RTC_VER_1_0
#undef LLD_RTC_VER_2_0

#if defined (__generic__)
#elif defined (__STA2062__)

  #if defined (__MST__)
  #if defined (__linux__)
  #include "mapping_sta2062_linux.h"
  #else
  #include "mapping_sta2062.h"
  #include "gpio_sta2062.h"
  #endif
  #define LLD_I2C_ICSUPPORT
  #elif defined (__SLV__)
  #include "mapping_hss.h"
  #endif

  #define LLD_UART_VER_5_1_2_0
  #define LLD_I2C_DMASUPPORT
  #define LLD_RTC_VER_1_0

#elif defined (__STA2064__)

  #if defined (__MST__)
  #if defined(_WIN32_WCE) || defined (__linux__)
  #include "mapping_sta2064_linux.h"
  #else
  #include "mapping_sta2064.h"
  #endif
  #include "gpio_sta2064.h"
  #define LLD_I2C_ICSUPPORT
  #elif defined (__SLV__)
  #include "mapping_gss.h"
  #endif

  #define LLD_UART_VER_5_1_10_0
  #define LLD_I2C_DMASUPPORT
  #define LLD_RTC_VER_2_0

#elif defined (__STA8088__)

  #include "mapping_sta8088.h"
  #include "gpio_sta8088.h"
//  #include "lld_arm946.h"

  #define LLD_I2C_ICSUPPORT
  #define LLD_UART_VER_5_1_10_0
  #define LLD_RTC_VER_2_0
  #define LLD_SQI_VER_1_0

//  #define LLD_EnterCritical()   LLD_ARM946_EnterCritical()
//  #define LLD_ExitCritical()    LLD_ARM946_ExitCritical()

#elif defined (__STA8090__)

  #include "mapping_sta8090.h"
  #include "gpio_sta8090.h"
//  #include "lld_arm946.h"

  #define LLD_I2C_ICSUPPORT
  #define LLD_UART_VER_5_1_10_0
  #define LLD_RTC_VER_2_0
  #define LLD_SQI_VER_2_0

//  #define LLD_EnterCritical()   LLD_ARM946_EnterCritical()
//  #define LLD_ExitCritical()    LLD_ARM946_ExitCritical()

#endif

//
// These APIs should not depend on operating system!
//
#if !defined( LLD_EnterCritical )
#define LLD_EnterCritical()   gpOS_interrupt_lock()
#endif
#if !defined( LLD_ExitCritical )
#define LLD_ExitCritical()    gpOS_interrupt_unlock()
#endif


//! \brief Returns size of a type aligned to the size of another one
#define LLD_MEMORY_ALIGN( size, type)     (( (size) + ( sizeof(type) - 1U)) & ~( sizeof(type) - 1U))

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//! \brief LLD generic error
typedef enum {
  LLD_NO_ERROR,
  LLD_ERROR
} LLD_ErrorTy;

/*****************************************************************************
   exported variables
*****************************************************************************/

//! \brief LLD version string
extern const tChar *lld_ver;

//! \brief LLD supplier id string
extern const tChar *lld_supplier_id;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

#endif  // LLD_H

// End of file
