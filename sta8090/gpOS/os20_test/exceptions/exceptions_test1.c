/*****************************************************************************
   FILE:          exceptions_test1.c
   PROJECT:       STA8088 GNSS application
   SW PACKAGE:    STA8088 OS20+ Examples
------------------------------------------------------------------------------
   DESCRIPTION:   Exceptions
                  (OS20+ operative system specification, Chapter 9
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics,
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
#include "except.h"
#include "lld_uart.h"
#if defined( __STA8088__ )
#include "lld_clk_ctrl_sta8088.h"
#elif defined( __STA8090__)
#include "lld_prcc_sta8090.h"
#endif
// Platform related
#include "platform.h"
/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define TEST5_WS_SIZE 1024

#define EXCEPTION_UNDEFINED       0
#define EXCEPTION_PREFETCH_ABORT  1
#define EXCEPTION_DATA_ABORT      2

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/
typedef struct exceptions_test1_manager_s
{
    gpOS_task_t *          task;
} exceptions_test1_manager_t;

static exceptions_test1_manager_t exceptions_test1_manager;
int exceptions_test1_task_priority = 9;
/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static gpOS_task_exit_status_t exceptions_test1_process( void *);

/********************************************//**
 * \brief
 *
 * \param
 * \return
 ***********************************************/
static gpOS_task_exit_status_t exceptions_test1_process( void *p)
{
  static tU32 counter = 5;

  while ( counter)
  {

    GPS_DEBUG_MSG(("[Exceptions] [1] [%d]\r\n", counter--));
    gpOS_task_delay( 1 * gpOS_timer_ticks_per_sec());

  }

  *(volatile uint32 *)(0x8aa00204) = 0x12345678;

  return gpOS_SUCCESS;
}

/* DIRECT UART WRITE */
void direct_uart_init( unsigned int uart_base_address)
{
  LLD_UART_ResetReg(( LLD_UART_IdTy)uart_base_address);
  LLD_UART_Config(
    ( LLD_UART_IdTy)uart_base_address,
    //platform_get_ahb_speed( PERIPHID_UART2),
    26000000,
    LLD_UART_115200_BPS,
    LLD_UART_ONE_STOP_BITS,
    LLD_UART_EIGTH_BITS,
    LLD_UART_NO_PARITY
  );

  LLD_UART_FifoEnable(( LLD_UART_IdTy)uart_base_address);
  LLD_UART_TxEnable  (( LLD_UART_IdTy)uart_base_address);
  LLD_UART_Enable    (( LLD_UART_IdTy)uart_base_address);
}

void gpOS_ISR direct_uart_write(unsigned int uart_base_address, char byte)
  {
    while( LLD_UART_IsBusy(( LLD_UART_IdTy)uart_base_address));
    LLD_UART_WriteTxFifo(( LLD_UART_IdTy)uart_base_address,byte);
  }

void gpOS_ISR direct_write( char byte)
  {
    direct_uart_write( UART2_REG_START_ADDR,byte);
  }

void gpOS_ISR debug_dump_buffer( char *buffer, int buffer_size)
  {
    int i;

    for( i=0; i<buffer_size; i++)
     {
      direct_write( buffer[ i]);
     }
  }

void gpOS_ISR debug_dump_memory( unsigned int exception_id)
{
  switch( exception_id)
    {
      case EXCEPTION_UNDEFINED:      debug_dump_buffer("\r\n<EXCEPTION UNDEFINED>       \r\n", 32);
      break;
      case EXCEPTION_PREFETCH_ABORT: debug_dump_buffer("\r\n<EXCEPTION PREFETCH ABORT>  \r\n", 32);
      break;
      case EXCEPTION_DATA_ABORT:     debug_dump_buffer("\r\n<EXCEPTION DATA ABORT>      \r\n", 32);
      break;
    }

}


void gpOS_ISR debug_undef_exception( void)
{

  #if defined( __STA8088__ )
  WRITE32( 0x11000004, 0xA509);
  #elif defined( __STA8090__)
  WRITE32( 0x40000004, 0xA509);
  #endif

  debug_dump_memory( EXCEPTION_UNDEFINED);
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_GenerateSoftReset();
  #elif defined( __STA8090__)
  LLD_PRCC_GenerateSoftReset();
  #endif

}

void gpOS_ISR debug_pfabort_exception( void)
{

  #if defined( __STA8088__ )
  WRITE32( 0x11000008, 0x84F3);
  #elif defined( __STA8090__)
  WRITE32( 0x40000008, 0x84F3);
  #endif

  debug_dump_memory( EXCEPTION_PREFETCH_ABORT);
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_GenerateSoftReset();
  #elif defined( __STA8090__)
  LLD_PRCC_GenerateSoftReset();
  #endif
}

void gpOS_ISR debug_dabort_exception( void)
{

  #if defined( __STA8088__ )
  WRITE32( 0x1100000C, 0xD271);
  #elif defined( __STA8090__)
  WRITE32( 0x4000000C, 0xD271);
  #endif


  debug_dump_memory( EXCEPTION_DATA_ABORT);
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_GenerateSoftReset();
  #elif defined( __STA8090__)
  LLD_PRCC_GenerateSoftReset();
  #endif
}

void debug_dump_memory_init( void)
{

  gpOS_exception_init( OS20_EXCEPTION_UND,   ( os20_exception_vector_t)debug_undef_exception);
  gpOS_exception_init( OS20_EXCEPTION_PFABT, ( os20_exception_vector_t)debug_pfabort_exception);
  gpOS_exception_init( OS20_EXCEPTION_DABT,  ( os20_exception_vector_t)debug_dabort_exception);

}

gpOS_error_t exceptions_test1_run( gpOS_partition_t *part)
{
  #if defined( __STA8088__ )
  unsigned int *ptr = (unsigned int *)0x11000000;
  #elif defined( __STA8090__)
  unsigned int *ptr = (unsigned int *)0x40000000;
  #endif

  unsigned int i;
  #if defined( __STA8088__ )
  LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_UART2);
  #elif defined( __STA8090__)
  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_UART2);
  #endif

  direct_uart_init( UART2_REG_START_ADDR);

  GPS_DEBUG_MSG(("\r\n[Exceptions] [1] [Countdown for DABORT]\r\n\r\n"));

  #if defined( __STA8088__ )
  LLD_CLK_CTRL_EnablePeripheral( LLD_CLK_CTRL_PERIPHID_SRAM);
  #elif defined( __STA8090__)
  LLD_PRCC_EnablePeripheral( LLD_PRCC_PERIPHID_SRAM);
  #endif

  for (i=0; i<4; i+=4)
    {
    GPS_DEBUG_MSG(("TRACE (UART0) 0x%08x 0x%08x 0x%08x 0x%08x\r\n",ptr[i],ptr[i+1],ptr[i+2],ptr[i+3]));
    }

  debug_dump_buffer("\r\n<TEST UART2>                \r\n", 32);

  #if defined( __STA8088__ )
  WRITE32( 0x11000000, 0xCC11AA00);
  #elif defined( __STA8090__)
  WRITE32( 0x40000000, 0xCC11AA00);
  #endif

  debug_dump_memory_init();

  exceptions_test1_manager.task = gpOS_task_create ( exceptions_test1_process, NULL, TEST5_WS_SIZE, exceptions_test1_task_priority, "Except T1 P1", gpOS_TASK_FLAGS_ACTIVE);

  if( exceptions_test1_manager.task == NULL)
    {
    GPS_DEBUG_MSG(( "OS20TEST_NO_ERROR task_init() failed\n"));

      return gpOS_FAILURE;
    }

  return gpOS_SUCCESS;
}
/*}}}  */
