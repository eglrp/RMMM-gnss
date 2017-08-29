/*****************************************************************************
   FILE:          interrupts_test1.c
   PROJECT:       STA8088 GPS application
   SW PACKAGE:    STA8088 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Interrupts
                  (OS20+ operative system specification, Chapter 12)
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2011 STMicroelectronics,
------------------------------------------------------------------------------
   Developers:
      AO:   Aldo Occhipinti
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
 ------------+------+------------------------------------------------------
             |      |
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/
#include "gpOS.h"
#include "gpOS_types.h"
#include "gnss_debug.h"
#include "lld_uart.h"
#if defined( __STA8088__ )
#include "lld_clk_ctrl_sta8088.h"
#elif defined( __STA8090__)
#include "lld_prcc_sta8090.h"
#endif
#include "lld_vic.h"
#include "lld_rtc.h"
#include "lld_rtt.h"
#include "platform.h"
#include "svc_mcu.h"

/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define INTERRUPTS_TEST1_WS_SIZE 2048
#define ALARM_DELAY 10
#if defined( __STA8088__ )
#define LPWFI_PERIPHERAL (( 1<<LLD_CLK_CTRL_PERIPHID_SQIO)  | \
                          ( 1<<LLD_CLK_CTRL_PERIPHID_GPIO0) | \
                          ( 1<<LLD_CLK_CTRL_PERIPHID_GPIO1) | \
                          ( 1<<LLD_CLK_CTRL_PERIPHID_SSP))
#elif defined( __STA8090__)
#define LPWFI_PERIPHERAL (( 1U << LLD_PRCC_PERIPHID_SQIO)  | \
                          ( 1U << LLD_PRCC_PERIPHID_GPIO0) | \
                          ( 1U << LLD_PRCC_PERIPHID_GPIO1) | \
                          ( 1U << LLD_PRCC_PERIPHID_SSP))
#endif

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
typedef struct interrupts_test1_manager_s
{
  gpOS_partition_t *     part;
  gpOS_task_t *          task;           /**< task pointer */

} interrupts_test1_manager_t;

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
static interrupts_test1_manager_t *interrupts_test1_manager = NULL;
int interrupts_test1_task_priority = 9;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
void enter_wfi_mode( void *dummy)
{
#if defined( __ARMCC_VERSION)
  #pragma arm
  __asm
  {
    MOV   r0, #0x0
    MCR   p15, 0, r0, c7, c0, 4
  }
#endif
#if defined( __GNUC__)
  __asm volatile ("\
    .balign 4\n                       ;\
    mov   r3, pc                      ;\
    bx    r3                          ;\
    .arm                              ;\
                                      ;\
    MOV   r0, #0x0                    ;\
    MCR   p15, 0, r0, c7, c0, 4       ;\
                                      ;\
    .arm\n                            ;\
    add   r3, pc, #1                  ;\
    bx    r3                          ;\
    .thumb                            ;\
    "
  );
#endif
}

tVoid MyRTC_InterruptManager(tVoid)
{
  tU32 irq_status;

  irq_status = LLD_RTC_GetInterruptStatus();

  if( irq_status)
  {
    LLD_RTC_DisableInterrupt();
    LLD_RTC_ClearInterrupt() ;
  }

}

/********************************************//**
 * \brief
 *
 * \param
 * \return
 ***********************************************/
static gpOS_task_exit_status_t interrupts_test1_process( void *p)
{
  tU32 counter, alarm;

  counter = LLD_RTC_GetDataRegister();
  alarm = counter + 10;
  LLD_RTC_SetMatchRegister( alarm);

  GPS_DEBUG_MSG(("[Interrupts] [1] [Before WFI] [%d]\r\n", gpOS_time_now()));

  gpOS_task_delay( 100 * gpOS_timer_ticks_per_msec());

  // Configure and enable the RTT interrupt channel
  gpOS_interrupt_install( VIC_RTC_LINE, gpOS_INTERRUPT_NOPRIORITY, (gpOS_interrupt_callback_t)MyRTC_InterruptManager, NULL);
  gpOS_interrupt_enable( VIC_RTC_LINE);
  LLD_RTC_EnableInterrupt();

  /* Disable all unused peripherals acting on Reset and Clock gate on CLK_CNTL Register 0 and CLK_CNTL Register 1*/
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_DisablePerMask( LPWFI_PERIPHERAL);
  #elif defined( __STA8090__)
  LLD_PRCC_DisablePerMask( LPWFI_PERIPHERAL);
  //SET32_BIT( 0x1200F040, LPWFI_PERIPHERAL);
  #endif

  #if defined( __STA8088__ )
  /* Clear the G3H RSTN (bit 11) of the CLK_CTRL_REG1 register */
  /* Clear the G3H CLK  (bit 11) of the CLK_CTRL_REG0 register */
  LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_G3_HCLK);
  /* Clear the G3 RSTN (bit 10) of the CLK_CTRL_REG1 register */
  /* Clear the G3 CLK  (bit 10) of the CLK_CTRL_REG0 register */
  LLD_CLK_CTRL_DisablePeripheral( LLD_CLK_CTRL_PERIPHID_G3);
  #elif defined( __STA8090__)
  LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_G3EP);
  LLD_PRCC_DisablePeripheral( LLD_PRCC_PERIPHID_G3EP_AHB);
  #endif

  /* Disable RF section */
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_RFDisable();
  #elif defined( __STA8090__)
  //LLD_PRCC_RFDisable();
  #endif

  /* Select 32 KHz RTC clock */
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_SetClockSource( LLD_CLK_CTRL_CLKSRC_RTC);
  #elif defined( __STA8090__)
  LLD_PRCC_SelectARMClkSrc( LLD_PRCC_ARMCLKSRC_RTC, TRUE);
  #endif

  LLD_RTT_SetLoadRegister( ALARM_DELAY * 32768);

  /* Shutdown Main High Power Regulator */
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_Disable_MVR();
  #elif defined( __STA8090__)
  //to write down how to switch off the MVR in t3
  #endif

  /* Set Teseo2 in WFI mode */
  //gpOS_kernel_user_system_call ( enter_wfi_mode, NULL);

  svc_mcu_enter_wfi();

  /* Restart Main High Power Regulator */
    #if defined( __STA8088__ )
  LLD_CLK_CTRL_Enable_MVR();
  #elif defined( __STA8090__)
  //to write down how to switch on the MVR in t3
  #endif

  /* back to running Mode clock */
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_SetClockSource( LLD_CLK_CTRL_CLKSRC_TWICE);
  #elif defined( __STA8090__)
  platform_set_cpu_clock_speed( LLD_PRCC_ARMCLKSRC_192f0, LLD_PRCC_CLKSEL_192);
  #endif

  /* Enabl/e RF section */
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_RFEnable();
  #elif defined( __STA8090__)
  //LLD_PRCC_RFEnable();
  #endif



  #if defined( __STA8088__ )
  /* Set the G3 RSTN (bit 10) of the CLK_CTRL_REG1 register */
  /* Set the G3 CLK  (bit 10) of the CLK_CTRL_REG0 register */
  LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_G3);

  /* Set the G3H RSTN (bit 11) of the CLK_CTRL_REG1 register */
  /* Set the G3H CLK  (bit 11) of the CLK_CTRL_REG0 register */
  LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_G3_HCLK);
  #elif defined( __STA8090__)
  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_G3EP_AHB);
  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_G3EP);
  #endif



  /* Enable all the previously disabled peripherals */
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_EnablePerMask( LPWFI_PERIPHERAL);
  #elif defined( __STA8090__)
  LLD_PRCC_EnablePerMask( LPWFI_PERIPHERAL);
  #endif

  GPS_DEBUG_MSG(("[Interrupts] [1] [After WFI] [%d]\r\n", gpOS_time_now()));

  return gpOS_SUCCESS;

}

/********************************************//**
 * \brief
 *
 * \param tVoid
 * \return os20_error_t
 ***********************************************/
gpOS_error_t interrupts_test1_run( gpOS_partition_t *part)
{

  interrupts_test1_manager->part    = part;

  interrupts_test1_manager->task    = gpOS_task_create_p( part, interrupts_test1_process, NULL, INTERRUPTS_TEST1_WS_SIZE, interrupts_test1_task_priority, "Interrupts T2 P1", gpOS_TASK_FLAGS_ACTIVE);

  if( interrupts_test1_manager->task    == NULL)
    {
      gpOS_task_delete( interrupts_test1_manager->task);
      return gpOS_FAILURE;
    }

  return gpOS_SUCCESS;

}
/*}}}  */
