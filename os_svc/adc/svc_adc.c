//!
//!   \file     svc_adc.c
//!   \brief    <i><b>ADC Low Level Driver source file</b></i>
//!   \author   Maristella Frazzetto
//!   \version  1.0
//!   \date     2013.18.03
//!   \bug      Unknown
//!   \warning  None
//!   \addtogroup LLD
//!

/*****************************************************************************
   includes
*****************************************************************************/
#include "typedefs.h"

#if defined( __STA8088__ )
#include "lld_adc_sta8088.h"
#endif
#if defined( __STA8090__ )
#include "lld_adc_sta8090.h"
#endif

#include "gpOS.h"
#include "svc_mcu.h"
#include "svc_adc.h"
#include "string.h"


/*****************************************************************************
   external declarations
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/
#define SVC_ADC_CHANNUM_MAX          8
#define SVC_ADC_HANDLER_SIZE         sizeof( svc_adc_handler_t)
#define SVC_ADC_CHAN_HANDLER_SIZE    sizeof( svc_adc_chan_handler_t)

#if defined( __STA8088__)
#define SVC_ADC_AVERG_MIN            256
#define SVC_ADC_AVERG_MAX            2048

#define START_PAGE_OFFSET_0   0
#define START_PAGE_OFFSET_1   256
#define START_PAGE_OFFSET_2   512
#define START_PAGE_OFFSET_3   768
#define START_PAGE_OFFSET_4   1024
#define START_PAGE_OFFSET_5   1280
#define START_PAGE_OFFSET_6   1536
#define START_PAGE_OFFSET_7   1792
#endif
/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

//typedef struct svc_adc_chan_handler_s svc_adc_chan_handler_t;

/********************************************//**
 * \brief ADC svc_mcu handler
 ***********************************************/
typedef struct svc_adc_handler_s
{
  gpOS_partition_t *            partition;                        /**< Partition used for svc_mcu */
  tU32                        mem_used;                         /**< Memory used by svc_mcu */

  gpOS_semaphore_t *            access_sem;                       /**< Access semaphore to ADC */

  LLD_ADC_IdTy                adc_id;                           /**< ADC peripheral address */
  tUInt                       adc_port;                         /**< ADC port */
  tU8                         acq_chan_mask[8];                 /**< ADC acquisition channel mask reporting the num of chan for which data are requested */
  tU32                        adc_clk_div_factor;               /**< ADC clock divisor factor */
  svc_adc_chan_cfg_t          num_channels;                     /**< ADC number of channels */
  #if defined( __STA8088__ )
  LLD_ADC_SelTy               sel_line_mask;                    /**< Selector line bit field to control which channels are masked */
  #endif
  svc_adc_int_mode_cfg_t      adc_mode;                         /**< ADC functional interrupt mode */
  #if defined( __STA8090__ )
  LLD_ADC_ConfigTy            adc_configuration;                /**< ADC configuration */
  #endif
  svc_adc_chan_handler_t *    chan_head[SVC_ADC_CHANNUM_MAX];   /**< Linked list of channels */
} svc_adc_handler_t;

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/
#if defined( __STA8088__ )
const tU32 page_offsets[] = {
    LLD_ADC_OffsetPage0,
    LLD_ADC_OffsetPage1,
    LLD_ADC_OffsetPage2,
    LLD_ADC_OffsetPage3,
    LLD_ADC_OffsetPage4,
    LLD_ADC_OffsetPage5,
    LLD_ADC_OffsetPage6,
    LLD_ADC_OffsetPage7
  };

const tU32 counter_vals[] = {
  START_PAGE_OFFSET_0,
  START_PAGE_OFFSET_1,
  START_PAGE_OFFSET_2,
  START_PAGE_OFFSET_3,
  START_PAGE_OFFSET_4,
  START_PAGE_OFFSET_5,
  START_PAGE_OFFSET_6,
  START_PAGE_OFFSET_7
};
#endif
/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

static svc_adc_handler_t *svc_adc_handler = NULL;
svc_adc_chan_handler_t *p_adc_chan_hdl_ptr = NULL;

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/
static void   svc_adc_callback     ( svc_adc_chan_handler_t **);

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

/********************************************//**
 * \brief ADC svc_mcu interrupt callback
 *
 * \param adc_hdlr_ptr
 * \return void
 *
 ***********************************************/
static LLD_ISR_ADC void svc_adc_callback( svc_adc_chan_handler_t **adc_hdlr_ptr)
{
#if defined( __STA8088__ )
  tInt i;
  tInt chan_num_cnt = 0;
#endif

  if ( NULL != *adc_hdlr_ptr)
  {
    svc_adc_chan_handler_t * adc_ptr = *adc_hdlr_ptr;

#if defined( __STA8088__ )
    /**< Check the first 4 ADC buffer pages */
    if (LLD_ADC_GetCmpReg(svc_adc_handler->adc_id) == START_PAGE_OFFSET_4)
    {
      /**< Stop ADC conversion */
      LLD_ADC_Disable(svc_adc_handler->adc_id);
      /**< Disable Comparator Interrupt source */
      LLD_ADC_SetCmpIntEnable(svc_adc_handler->adc_id, LLD_ADC_CmpIntDISABLED);
      /**< Clear the Comparator flag */
      LLD_ADC_ClearCmpFlag(svc_adc_handler->adc_id);
      /**< Check channels mask */
      switch ( svc_adc_handler->sel_line_mask)
      {
        case LLD_ADC_UnMasked_Channel_8:
          for (i = 0; i < (SVC_ADC_CHANNUM_MAX/2); i++)
          {
            if (svc_adc_handler->acq_chan_mask[i] == 1)
            {
              svc_adc_handler->acq_chan_mask[i] = 0;
              chan_num_cnt++;
            }
          }
          break;

        case LLD_ADC_UnMasked_Channel_4:
          for (i = 0; i < 3; i++)
          {
            if (svc_adc_handler->acq_chan_mask[i] == 1)
            {
              svc_adc_handler->acq_chan_mask[i] = 0;
              chan_num_cnt++;
            }
          }
          break;

        case LLD_ADC_UnMasked_Channel_2:
          if (svc_adc_handler->acq_chan_mask[0] == 1)
          {
            svc_adc_handler->acq_chan_mask[0] = 0;
            chan_num_cnt++;
          }
          break;

        case LLD_ADC_UnMasked_Channel_1:
          if (svc_adc_handler->acq_chan_mask[0] == 1)
          {
            if ( adc_ptr->buf_average_len == SVC_ADC_AVERG_MIN)
            {
              svc_adc_handler->acq_chan_mask[0] = 0;
            }
            chan_num_cnt++;
          }
          break;

      }
      /**< If only one channel is unmasked is needed to check if a minimun average is requested or a maximum */
      if (svc_adc_handler->sel_line_mask == LLD_ADC_UnMasked_Channel_1)
      {
        if ( adc_ptr->buf_average_len == SVC_ADC_AVERG_MIN)
        {
          /**< If chan mask is empty no task requested adc samples for input from AIN0 to AIN3. Start again adc conversion to complete the buffer */
          if (chan_num_cnt == 0)
          {
            /**< Set the Comparator Register to the start address of the first page and the counter value to the start address of the fourth page */
            /**< The cmp flag is cleared, the comparator interrupt source enabled again and the adc started again */
            /**< The semaphore is not released bacause the second half of page buffer has to be checked */
            LLD_ADC_SetCmpReg(svc_adc_handler->adc_id, START_PAGE_OFFSET_0);
            LLD_ADC_SetCntReg(svc_adc_handler->adc_id, START_PAGE_OFFSET_4);
            LLD_ADC_SetCmpIntEnable(svc_adc_handler->adc_id, LLD_ADC_CmpIntENABLED);
            LLD_ADC_Enable(svc_adc_handler->adc_id);
          }
          /**< If chan mask contains at least one element release the semaphore in order to return the requested adc averaged sample */
          else if (chan_num_cnt > 0)
          {
            gpOS_semaphore_signal(adc_ptr->done_sem);
          }
        }
        else if ( adc_ptr->buf_average_len == SVC_ADC_AVERG_MAX)
        {
          LLD_ADC_SetCmpReg(svc_adc_handler->adc_id, START_PAGE_OFFSET_0);
          LLD_ADC_SetCntReg(svc_adc_handler->adc_id, START_PAGE_OFFSET_4);
          LLD_ADC_SetCmpIntEnable(svc_adc_handler->adc_id, LLD_ADC_CmpIntENABLED);
          LLD_ADC_Enable(svc_adc_handler->adc_id);
        }
      }
      else if ( svc_adc_handler->sel_line_mask != LLD_ADC_UnMasked_Channel_1)
      {
        /**< If chan mask is empty no task requested adc samples for input from AIN0 to AIN3. Start again adc conversion to complete the buffer */
        if (chan_num_cnt == 0)
        {
          LLD_ADC_SetCmpReg(svc_adc_handler->adc_id, START_PAGE_OFFSET_0);
          LLD_ADC_SetCntReg(svc_adc_handler->adc_id, START_PAGE_OFFSET_4);
          LLD_ADC_SetCmpIntEnable(svc_adc_handler->adc_id, LLD_ADC_CmpIntENABLED);
          LLD_ADC_Enable(svc_adc_handler->adc_id);
        }
        /**< If chan mask contains at least one element release the semaphore in order to return the requested adc averaged sample */
        else if (chan_num_cnt > 0)
        {
          gpOS_semaphore_signal(adc_ptr->done_sem);
        }
      }
      chan_num_cnt = 0;
    }

    /**< Check the second 4 ADC buffer pages */
    if ((LLD_ADC_GetCmpFlag(svc_adc_handler->adc_id) && (LLD_ADC_GetCmpReg(svc_adc_handler->adc_id) == START_PAGE_OFFSET_0)))
    {
      /**< Stop ADC conversion */
      LLD_ADC_Disable(svc_adc_handler->adc_id);
      /**< Disable Comparator Interrupt source */
      LLD_ADC_SetCmpIntEnable(svc_adc_handler->adc_id, LLD_ADC_CmpIntDISABLED);
      /**< Clear the Comparator flag */
      LLD_ADC_ClearCmpFlag(svc_adc_handler->adc_id);
      /**< Check channels mask */
      switch ( svc_adc_handler->sel_line_mask)
      {
        case LLD_ADC_UnMasked_Channel_8:
          for (i = (SVC_ADC_CHANNUM_MAX/2); i < SVC_ADC_CHANNUM_MAX; i++)
          {
            if (svc_adc_handler->acq_chan_mask[i] == 1)
            {
              svc_adc_handler->acq_chan_mask[i] = 0;
              chan_num_cnt++;
            }
          }
          break;

        case LLD_ADC_UnMasked_Channel_4:
          for (i = (SVC_ADC_CHANNUM_MAX/2); i < 7; i++)
          {
            if (svc_adc_handler->acq_chan_mask[i] == 1)
            {
              svc_adc_handler->acq_chan_mask[i] = 0;
              chan_num_cnt++;
            }
          }
          break;

        case LLD_ADC_UnMasked_Channel_2:
          if (svc_adc_handler->acq_chan_mask[4] == 1)
          {
            svc_adc_handler->acq_chan_mask[4] = 0;
            chan_num_cnt++;
          }
          break;

        case LLD_ADC_UnMasked_Channel_1:
          if (svc_adc_handler->acq_chan_mask[0] == 1)
          {
            if ( adc_ptr->buf_average_len == SVC_ADC_AVERG_MAX)
            {
              svc_adc_handler->acq_chan_mask[0] = 0;
            }
            chan_num_cnt++;
          }
          break;
      }

      if (chan_num_cnt == 0)
      {
        LLD_ADC_SetCmpReg(svc_adc_handler->adc_id, START_PAGE_OFFSET_4);
        LLD_ADC_SetCntReg(svc_adc_handler->adc_id, START_PAGE_OFFSET_0);
        LLD_ADC_SetCmpIntEnable(svc_adc_handler->adc_id, LLD_ADC_CmpIntENABLED);
        LLD_ADC_Enable(svc_adc_handler->adc_id);
      }
      /**< If chan mask contains at least one element release the semaphore in order to return the requested adc averaged sample */
      else if (chan_num_cnt > 0)
      {
        gpOS_semaphore_signal(adc_ptr->done_sem);
      }

      chan_num_cnt = 0;

    }
    #elif defined( __STA8090__ )
    if(LLD_ADC_GetIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_Lthch0))
    {
      LLD_ADC_ClearIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_Lthch0);
      LLD_ADC_DisableIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_CH0);
      gpOS_semaphore_signal( adc_ptr->done_sem);
    }

    if (LLD_ADC_GetIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_Hthch0))
    {
      LLD_ADC_ClearIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_Hthch0);
      LLD_ADC_DisableIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_CH0);
      gpOS_semaphore_signal( adc_ptr->done_sem);
    }

    if(LLD_ADC_GetIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_Lthch1))
    {
      LLD_ADC_ClearIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_Lthch1);
      LLD_ADC_DisableIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_CH1);
      gpOS_semaphore_signal( adc_ptr->done_sem);
    }

    if (LLD_ADC_GetIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_Hthch1))
    {
      LLD_ADC_ClearIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_Hthch1);
      LLD_ADC_DisableIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_CH1);
      gpOS_semaphore_signal( adc_ptr->done_sem);
    }
    #endif
  }
}

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

/********************************************//**
 * \brief Initialize ADC svc_mcu
 *
 * \param partition Partition to use to allocate memory
 * \param adc_port ADC port number
 * \param irq_pri Interrupt priority of ADC svc_mcu
 * \param num_chan ADC number channels available (not masked)
 * \param adc_clk_div ADC clock divisor factor
 * \param adc_functional_mode ADC functional mode
 * \return gpOS_error_t gpOS_SUCCESS if all is ok, else gpOS_FAILURE
 *
 ***********************************************/
gpOS_error_t svc_adc_init( gpOS_partition_t *partition, tInt adc_port, gpOS_interrupt_priority_t irq_pri, svc_adc_chan_cfg_t num_chan, tU32 adc_clk_div, /*LLD_ADC_ChanSelTy adc_chan_sel_switch,*/ svc_adc_int_mode_cfg_t*  adc_functional_mode)
{
  tInt i;
  tU32 mem_at_start = gpOS_memory_getheapfree_p( partition);
  svc_adc_chan_handler_t *adc_chan_hdl_ptr[SVC_ADC_CHANNUM_MAX];

  /**< Get the ADC peripheral address */
  LLD_ADC_IdTy  adc_phy_id        = (LLD_ADC_IdTy)svc_mcu_get_addr( SVC_MCU_PER_ID_ADC, adc_port);
  /**< Get the ADC peripheral IRQ line */
  VicLineTy     adc_phy_irq_line  = (VicLineTy)svc_mcu_get_irq_line( SVC_MCU_PER_ID_ADC, adc_port);

  if( svc_adc_handler == NULL)
  {
    /**< Memory allocation for ADC svc_mcu handler */
    svc_adc_handler = gpOS_memory_allocate_p( partition, SVC_ADC_HANDLER_SIZE);

    if( svc_adc_handler == NULL)
    {
      return gpOS_FAILURE;
    }

    svc_adc_handler->access_sem       = gpOS_semaphore_create_p( SEM_FIFO, partition, 0);
    if( svc_adc_handler->access_sem == NULL)
    {
      gpOS_semaphore_delete( svc_adc_handler->access_sem);
      gpOS_memory_deallocate_p( partition, svc_adc_handler);
      return gpOS_FAILURE;
    }

    svc_adc_handler->partition     = partition;
    #if defined( __STA8088__ )
    svc_adc_handler->sel_line_mask = (LLD_ADC_SelTy)num_chan;
    #elif defined( __STA8090__ )
    if( num_chan == ADC_2CHAN_AVAILABLE)
    {
      svc_adc_handler->adc_configuration.mode = (LLD_ADC_ModeTy)LLD_ADC_NORMAL_MODE_ENABLED;
    }
    else if( num_chan == ADC_8CHAN_AVAILABLE)
    {
      svc_adc_handler->adc_configuration.mode = (LLD_ADC_ModeTy)LLD_ADC_SW_MODE_ENABLED;
    }
    else
    {
      return gpOS_FAILURE;
    }
    #endif
    svc_adc_handler->adc_clk_div_factor       = adc_clk_div;
    svc_adc_handler->num_channels             = num_chan;
    svc_adc_handler->adc_port                 = adc_port;
    svc_adc_handler->adc_id                   = adc_phy_id;
    svc_adc_handler->adc_mode.adc_int_mode    = adc_functional_mode->adc_int_mode;
    svc_adc_handler->mem_used                 = mem_at_start - gpOS_memory_getheapfree_p( partition);

    for ( i = 0; i < SVC_ADC_CHANNUM_MAX; i++)
    {
      svc_adc_handler->chan_head[i] = NULL;
      svc_adc_handler->acq_chan_mask[i] = 0;
    }

    mem_at_start = gpOS_memory_getheapfree_p( svc_adc_handler->partition);

    #if defined( __STA8090__)
    svc_adc_handler->adc_configuration.chirqsrc = adc_functional_mode->adc_chan_irq_src;
    svc_adc_handler->adc_configuration.sel = adc_functional_mode->adc_chan_sel_switch;
    #endif

    for ( i = 0; i < SVC_ADC_CHANNUM_MAX; i++)
    {
      adc_chan_hdl_ptr[i] = gpOS_memory_allocate_p( svc_adc_handler->partition, SVC_ADC_CHAN_HANDLER_SIZE);
      if( adc_chan_hdl_ptr[i] == NULL)
      {
        gpOS_memory_deallocate_p( svc_adc_handler->partition, adc_chan_hdl_ptr[i]);
        return gpOS_FAILURE;
      }

      #if defined( __STA8088__)
      adc_chan_hdl_ptr[i]->done_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_adc_handler->partition, 0);
      if( adc_chan_hdl_ptr[i]->done_sem == NULL)
      {
        gpOS_semaphore_delete( adc_chan_hdl_ptr[i]->done_sem);
        return gpOS_FAILURE;
      }
      #endif

      adc_chan_hdl_ptr[i]->buf_ptr = NULL;
      adc_chan_hdl_ptr[i]->buf_ptr_pos = 0;
      adc_chan_hdl_ptr[i]->buf_average_len = 0;

      svc_adc_handler->chan_head[i] = adc_chan_hdl_ptr[i];
    }

    #if defined( __STA8090__)
    for(i = 0; i < 2; i++)
    {
      adc_chan_hdl_ptr[i]->done_sem = gpOS_semaphore_create_p( SEM_FIFO, svc_adc_handler->partition, 0);
      if( adc_chan_hdl_ptr[i]->done_sem == NULL)
      {
        gpOS_semaphore_delete( adc_chan_hdl_ptr[i]->done_sem);
        return gpOS_FAILURE;
      }
    }

    if ( svc_adc_handler->adc_mode.adc_int_mode == ADC_INTERRUPT)
    {
      if ( svc_adc_handler->adc_configuration.chirqsrc == LLD_ADC_IRQ_SRC_CH0)
      {
        adc_chan_hdl_ptr[0]->irq_src = TRUE;
      }
      else
      {
        adc_chan_hdl_ptr[0]->irq_src = FALSE;
      }
      if ( svc_adc_handler->adc_configuration.chirqsrc == LLD_ADC_IRQ_SRC_CH1)
      {
        adc_chan_hdl_ptr[1]->irq_src = TRUE;
      }
      else
      {
        adc_chan_hdl_ptr[1]->irq_src = FALSE;
      }
      if ( svc_adc_handler->adc_configuration.chirqsrc == LLD_ADC_IRQ_SRC_ALL)
      {
        adc_chan_hdl_ptr[0]->irq_src = TRUE;
        adc_chan_hdl_ptr[1]->irq_src = TRUE;
      }
      LLD_ADC_EnableIRQSrc( adc_phy_id, svc_adc_handler->adc_configuration.chirqsrc);
    }
    #endif

    /**< Install and enable the ADC interrupt  */
    gpOS_interrupt_install( adc_phy_irq_line, irq_pri, (gpOS_interrupt_callback_t)svc_adc_callback, &p_adc_chan_hdl_ptr);
    gpOS_interrupt_enable( adc_phy_irq_line);
    #if defined( __STA8090__)
    svc_adc_handler->adc_mode.Hth0 = adc_functional_mode->Hth0;
    svc_adc_handler->adc_mode.Lth0 = adc_functional_mode->Lth0;
    svc_adc_handler->adc_mode.Hth1 = adc_functional_mode->Hth1;
    svc_adc_handler->adc_mode.Lth1 = adc_functional_mode->Lth1;
    #endif

    /**< Enable ADC peripheral clock */
    svc_mcu_enable( SVC_MCU_PER_ID_ADC, adc_port);

    #if defined( __STA8088__)
    /**< Configure ADC: disable all the adc interrupt sources, set the comparator register and set the selector bit field to configure the num of chan available */
    LLD_ADC_SetEOCIntEnable(adc_phy_id, LLD_ADC_EOCIntDISABLED);
    LLD_ADC_SetCntIntEnable(adc_phy_id, LLD_ADC_CntIntDISABLED);
    LLD_ADC_SetCmpIntEnable(adc_phy_id, LLD_ADC_CmpIntDISABLED);

    LLD_ADC_SetCmpReg(adc_phy_id, START_PAGE_OFFSET_4);

    LLD_ADC_SetSelConfiguration(adc_phy_id, svc_adc_handler->sel_line_mask);
    #elif defined( __STA8090__)
    LLD_ADC_ResetReg(adc_phy_id);
    LLD_ADC_SetHth0Reg(adc_phy_id, svc_adc_handler->adc_mode.Hth0);
    LLD_ADC_SetLth0Reg(adc_phy_id, svc_adc_handler->adc_mode.Lth0);
    LLD_ADC_SetHth1Reg(adc_phy_id, svc_adc_handler->adc_mode.Hth1);
    LLD_ADC_SetLth1Reg(adc_phy_id, svc_adc_handler->adc_mode.Lth1);
    #endif

    /**< Set the ADC clock division factor and enable the clock */
    LLD_ADC_ClkDivEnable(adc_phy_id);
    LLD_ADC_SetClkReg(adc_phy_id, adc_clk_div);

    if ( svc_adc_handler->adc_mode.adc_int_mode == ADC_NOINTERRUPT)
    {
      LLD_ADC_Enable(svc_adc_handler->adc_id);
    }

    svc_adc_handler->mem_used += mem_at_start - gpOS_memory_getheapfree_p( svc_adc_handler->partition);

    gpOS_semaphore_signal( svc_adc_handler->access_sem);
  }
  else
  {
    if( svc_adc_handler != NULL )
    {
      #if defined( __STA8088__)
      if( (svc_adc_handler->adc_mode.adc_int_mode != adc_functional_mode->adc_int_mode) ||  (svc_adc_handler->adc_clk_div_factor != adc_clk_div) || (svc_adc_handler->num_channels != num_chan))
      {
        if ( svc_adc_handler->adc_clk_div_factor != adc_clk_div)
        {
          LLD_ADC_SetClkReg( adc_phy_id, adc_clk_div);
        }
        if(svc_adc_handler->adc_mode.adc_int_mode != adc_functional_mode->adc_int_mode)
        {
          svc_adc_handler->adc_mode.adc_int_mode = adc_functional_mode->adc_int_mode;
          if ( svc_adc_handler->adc_mode.adc_int_mode == ADC_NOINTERRUPT)
          {
            LLD_ADC_Enable(svc_adc_handler->adc_id);
          }
        }
        if( svc_adc_handler->num_channels != num_chan)
        {
          LLD_ADC_SetSelConfiguration(adc_phy_id, (LLD_ADC_SelTy)num_chan);
        }
      }
      else if( (svc_adc_handler->adc_mode.adc_int_mode == adc_functional_mode->adc_int_mode) &&  (svc_adc_handler->adc_clk_div_factor == adc_clk_div) && (svc_adc_handler->num_channels == num_chan))
      {
        return gpOS_SUCCESS;
      }
      #elif defined( __STA8090__)
      if( (svc_adc_handler->adc_mode.adc_int_mode != adc_functional_mode->adc_int_mode) ||  (svc_adc_handler->adc_clk_div_factor != adc_clk_div) || (svc_adc_handler->num_channels != num_chan) || (svc_adc_handler->adc_configuration.sel != adc_functional_mode->adc_chan_sel_switch)
          || (svc_adc_handler->adc_configuration.chirqsrc != adc_functional_mode->adc_chan_irq_src) || (svc_adc_handler->adc_mode.Hth0 != adc_functional_mode->Hth0) || (svc_adc_handler->adc_mode.Hth1 != adc_functional_mode->Hth1)
          || (svc_adc_handler->adc_mode.Lth0 != adc_functional_mode->Lth0) || (svc_adc_handler->adc_mode.Lth1 != adc_functional_mode->Lth1))
      {
        if ( svc_adc_handler->adc_clk_div_factor != adc_clk_div)
        {
          svc_adc_handler->adc_clk_div_factor = adc_clk_div;
          LLD_ADC_SetClkReg( adc_phy_id, adc_clk_div);
        }
        if(svc_adc_handler->adc_mode.adc_int_mode != adc_functional_mode->adc_int_mode)
        {
          svc_adc_handler->adc_mode.adc_int_mode = adc_functional_mode->adc_int_mode;
        }
        if( svc_adc_handler->num_channels != num_chan)
        {
          svc_adc_handler->num_channels = num_chan;
          if( num_chan == ADC_2CHAN_AVAILABLE)
          {
            svc_adc_handler->adc_configuration.mode = (LLD_ADC_ModeTy)LLD_ADC_NORMAL_MODE_ENABLED;
          }
          else if( num_chan == ADC_8CHAN_AVAILABLE)
          {
            svc_adc_handler->adc_configuration.mode = (LLD_ADC_ModeTy)LLD_ADC_SW_MODE_ENABLED;
          }
          else
          {
            return gpOS_FAILURE;
          }
          LLD_ADC_SetMode(adc_phy_id, svc_adc_handler->adc_configuration.mode);
        }
        if ( svc_adc_handler->adc_configuration.sel != adc_functional_mode->adc_chan_sel_switch)
        {
          svc_adc_handler->adc_configuration.sel = adc_functional_mode->adc_chan_sel_switch;
        }
        if ( svc_adc_handler->adc_configuration.chirqsrc != adc_functional_mode->adc_chan_irq_src)
        {
          svc_adc_handler->adc_configuration.chirqsrc = adc_functional_mode->adc_chan_irq_src;
          if ( svc_adc_handler->adc_configuration.chirqsrc == LLD_ADC_IRQ_SRC_CH0)
          {
            svc_adc_handler->chan_head[0]->irq_src = TRUE;
          }
          else
          {
            svc_adc_handler->chan_head[0]->irq_src = FALSE;
          }
          if ( svc_adc_handler->adc_configuration.chirqsrc == LLD_ADC_IRQ_SRC_CH1)
          {
            svc_adc_handler->chan_head[1]->irq_src = TRUE;
          }
          else
          {
            svc_adc_handler->chan_head[1]->irq_src = FALSE;
          }
          if ( svc_adc_handler->adc_configuration.chirqsrc == LLD_ADC_IRQ_SRC_ALL)
          {
            svc_adc_handler->chan_head[0]->irq_src = TRUE;
            svc_adc_handler->chan_head[1]->irq_src = TRUE;
          }
          LLD_ADC_EnableIRQSrc( adc_phy_id, svc_adc_handler->adc_configuration.chirqsrc);
        }
        if ( svc_adc_handler->adc_mode.Hth0 != adc_functional_mode->Hth0)
        {
          LLD_ADC_SetHth0Reg( adc_phy_id, adc_functional_mode->Hth0);
        }
        if ( svc_adc_handler->adc_mode.Hth1 != adc_functional_mode->Hth1)
        {
          LLD_ADC_SetHth1Reg( adc_phy_id, adc_functional_mode->Hth1);
        }
        if ( svc_adc_handler->adc_mode.Lth0 != adc_functional_mode->Lth0)
        {
          LLD_ADC_SetLth0Reg( adc_phy_id, adc_functional_mode->Lth0);
        }
        if ( svc_adc_handler->adc_mode.Lth1 != adc_functional_mode->Lth1)
        {
          LLD_ADC_SetLth1Reg( adc_phy_id, adc_functional_mode->Lth1);
        }
        if((!LLD_ADC_IsADCEnabled(svc_adc_handler->adc_id)) && ( svc_adc_handler->adc_mode.adc_int_mode == ADC_NOINTERRUPT))
        {
          LLD_ADC_Enable(svc_adc_handler->adc_id);
        }
      }
      //#endif
      else if( (svc_adc_handler->adc_mode.adc_int_mode == adc_functional_mode->adc_int_mode) &&  (svc_adc_handler->adc_clk_div_factor == adc_clk_div) && (svc_adc_handler->num_channels == num_chan) || (svc_adc_handler->adc_configuration.sel == adc_functional_mode->adc_chan_sel_switch)
         && (svc_adc_handler->adc_configuration.chirqsrc == adc_functional_mode->adc_chan_irq_src) && (svc_adc_handler->adc_mode.Hth0 == adc_functional_mode->Hth0) && (svc_adc_handler->adc_mode.Hth1 == adc_functional_mode->Hth1)
          || (svc_adc_handler->adc_mode.Lth0 == adc_functional_mode->Lth0) && (svc_adc_handler->adc_mode.Lth1 == adc_functional_mode->Lth1))
      {
        if((!LLD_ADC_IsADCEnabled(svc_adc_handler->adc_id)) && ( svc_adc_handler->adc_mode.adc_int_mode == ADC_NOINTERRUPT))
        {
          LLD_ADC_Enable(svc_adc_handler->adc_id);
        }

        return gpOS_SUCCESS;
      }
      #endif
    }
  }
  #if defined( __STA8090__ )
  LLD_ADC_DisablePullUpAIN0( svc_adc_handler->adc_id);
  LLD_ADC_DisablePullUpAIN1( svc_adc_handler->adc_id);
  #endif
  return gpOS_SUCCESS;
}

/********************************************//**
 * \brief Read from ADC buffer and perform data filtering
 *
 * \param chan_id tInt channel id (0..7)
 * \param len_average tU32    number of samples to be averaged
 * \param out_data tU32*      data output
 * \return gpOS_error_t
 *
 ***********************************************/
gpOS_error_t svc_adc_read( tInt chan_id, tU32 len_average, tU32 *out_data, tVoid *out_buf)
{
  svc_adc_chan_handler_t *adc_chan_hdlr = svc_adc_handler->chan_head[chan_id];
  tU32 buf_pos = adc_chan_hdlr->buf_ptr_pos;

  #if defined( __STA8088__)
  tU32 adc_mean = 0;
  tU32 data_out_temp = 0;
  tU32 num_page_avrg = 0;
  tInt i, j;
  #endif
  #if defined( __STA8090__)
  LLD_ADC_Chan1_cntmask_Ty cnt_chan1_mask;
  LLD_ADC_ChanswitchTy chan_switch_mask;
  LLD_ADC_ChanSelTy chan_sel;
  #endif

  if( adc_chan_hdlr == NULL)
  {
    return gpOS_FAILURE;
  }
  else
  {
  #if defined( __STA8088__)
    /**< The reading of the ADC data stored is possible only if the channel id is compatible with the number of channels available and not masked */
    /**< If channels available = 8 it is possible reading data for all the eight channels: 0 .. 7 */
    /**< If channels available = 4 it is possible reading data for the channels: 0, 2, 4, 6 */
    /**< If channels available = 2 it is possible reading data for the channels: 0, 4 */
    /**< If channels available = 1 it is possible reading data only for the channel 0 */
    if((( svc_adc_handler->sel_line_mask == LLD_ADC_UnMasked_Channel_8) && ( chan_id >= 0) && ( chan_id <= 7))
         || (( svc_adc_handler->sel_line_mask == LLD_ADC_UnMasked_Channel_4) && (( chan_id == 0) || ( chan_id == 2) || ( chan_id == 4) || ( chan_id == 6)))
         || (( svc_adc_handler->sel_line_mask == LLD_ADC_UnMasked_Channel_2) && (( chan_id == 0) || ( chan_id == 4)))
         || (( svc_adc_handler->sel_line_mask == LLD_ADC_UnMasked_Channel_1) &&  ( chan_id == 0)))
    {
      /**< Access ADC handler */
      gpOS_semaphore_wait( svc_adc_handler->access_sem);

      /**< Lock Interrupt */
      gpOS_interrupt_lock();

      adc_chan_hdlr->out_data = 0;

      /**< Set the channel mask according to the chan id value */
      svc_adc_handler->acq_chan_mask[chan_id] = 1;

      /**< Set the length of data filtering: only two possibilities, MIN (only one page, 256 words of 32 bits i.e. 512 samples) or MAX (dipending on chan num available)  */
      adc_chan_hdlr->buf_average_len = len_average;

      /**< COnfigure the number of buffer pages to average */
      switch ( svc_adc_handler->sel_line_mask)
        {
          case LLD_ADC_UnMasked_Channel_8:
            num_page_avrg = 1;
            break;

          case LLD_ADC_UnMasked_Channel_4:
            if (len_average == SVC_ADC_AVERG_MIN)
            {
              num_page_avrg = 1;
            }
            else if (len_average == SVC_ADC_AVERG_MAX)
            {
              num_page_avrg = 2;
            }
            break;

          case LLD_ADC_UnMasked_Channel_2:
            if (len_average == SVC_ADC_AVERG_MIN)
            {
              num_page_avrg = 1;
            }
            else if (len_average == SVC_ADC_AVERG_MAX)
            {
              num_page_avrg = 4;
            }
            break;

          case LLD_ADC_UnMasked_Channel_1:
            if (len_average == SVC_ADC_AVERG_MIN)
            {
              num_page_avrg = 1;
            }
            else if (len_average == SVC_ADC_AVERG_MAX)
            {
              num_page_avrg = 8;
            }
            break;
        }


      LLD_ADC_ResetCntReg(svc_adc_handler->adc_id);

      p_adc_chan_hdl_ptr = adc_chan_hdlr;

      /**< Select the ADC comparator interrupt source, clear the comparator flag and start the adc conversion */
      LLD_ADC_ClearCmpFlag(svc_adc_handler->adc_id);
      if (svc_adc_handler->adc_mode.adc_int_mode == ADC_INTERRUPT)
      {
        LLD_ADC_SetCmpIntEnable(svc_adc_handler->adc_id, LLD_ADC_CmpIntENABLED);
        LLD_ADC_Enable(svc_adc_handler->adc_id);
      }


      adc_chan_hdlr->buf_ptr = out_buf;

      /**< Unlock Interrupt */
      gpOS_interrupt_unlock();

      if(svc_adc_handler->adc_mode.adc_int_mode == ADC_INTERRUPT)
      {
        gpOS_semaphore_wait(adc_chan_hdlr->done_sem);

        /**< Get the data from the adc buffer */
        /**< Perform the average on one page or on a number of pages according to the num of chan not masked and to the average length required */
        for(j = 0; j < num_page_avrg; j++)
        {
        /**< Set the buffer page read selector*/
        adc_chan_hdlr->offset_page = (LLD_ADC_OffsetPageTy)page_offsets[chan_id + j];
        LLD_ADC_SetOffset(svc_adc_handler->adc_id, adc_chan_hdlr->offset_page);
        for(i = 0; i< 256; i++)
        {
          tU32 data;

          data = *((tU32 *)svc_adc_handler->adc_id + i);

          if(adc_chan_hdlr->buf_ptr != NULL)
          {
            ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = data;
          }

          buf_pos++;


          adc_mean += (data & 0x3ff);
          adc_mean += (((data & 0x03ff0000) >> 16) & 0x3ff);
          }

          /**< Average on one page, 512 samples */
          data_out_temp += (adc_mean/512);

          adc_mean = 0;

        }
        /**< Average on number of pages required */
        adc_chan_hdlr->out_data = data_out_temp/num_page_avrg;

      }
      else if (svc_adc_handler->adc_mode.adc_int_mode == ADC_NOINTERRUPT)
      {

        for(j = 0; j < num_page_avrg; j++)
        {
        /**< Set the buffer page read selector*/
        adc_chan_hdlr->offset_page = (LLD_ADC_OffsetPageTy)page_offsets[chan_id + j];
        LLD_ADC_SetOffset(svc_adc_handler->adc_id, adc_chan_hdlr->offset_page);

          LLD_ADC_Disable(svc_adc_handler->adc_id);

          for(i = 0; i< 256; i++)
          {
            tU32 data;

            data = *((tU32 *)svc_adc_handler->adc_id + i);

            if(adc_chan_hdlr->buf_ptr != NULL)
            {
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = data;
            }

            buf_pos++;


            adc_mean += (data & 0x3ff);
            adc_mean += (((data & 0x03ff0000) >> 16) & 0x3ff);
          }

          /**< Average on one page, 512 samples */
          data_out_temp += (adc_mean/512);

          adc_mean = 0;

        }
        /**< Average on number of pages required */
        adc_chan_hdlr->out_data = data_out_temp/num_page_avrg;
        LLD_ADC_Enable(svc_adc_handler->adc_id);

      }

      adc_chan_hdlr->buf_ptr          = NULL;
      adc_chan_hdlr->buf_average_len  = 0;
      adc_chan_hdlr->buf_ptr_pos      = 0;

      *out_data = adc_chan_hdlr->out_data;

      gpOS_semaphore_signal( svc_adc_handler->access_sem);

      return gpOS_SUCCESS;

    }
    else
    {
      return gpOS_FAILURE;
    }
    #endif
    #if defined( __STA8090__)
    if(((LLD_ADC_GetMode(svc_adc_handler->adc_id) == (LLD_ADC_ModeTy)LLD_ADC_SW_MODE_ENABLED) && ( chan_id >= 0) && ( chan_id <= 7))
         || ((LLD_ADC_GetMode(svc_adc_handler->adc_id) == (LLD_ADC_ModeTy)LLD_ADC_NORMAL_MODE_ENABLED) && (( chan_id == 0) || ( chan_id == 1))))
    {
      /**< Access ADC handler */
      gpOS_semaphore_wait( svc_adc_handler->access_sem);

      /**< Lock Interrupt */
      gpOS_interrupt_lock();

      svc_adc_handler->adc_configuration.chid = (LLD_ADC_ChanIdTy)chan_id;
      LLD_ADC_DisableIRQSrc(svc_adc_handler->adc_id, LLD_ADC_IRQ_SRC_ALL);
      LLD_ADC_SetConfiguration(svc_adc_handler->adc_id, &svc_adc_handler->adc_configuration);
      //LLD_ADC_SetCntReg(svc_adc_handler->adc_id, 0);
      //while(LLD_ADC_GetCntReg(svc_adc_handler->adc_id) < 0xD);

      p_adc_chan_hdl_ptr = adc_chan_hdlr;

      adc_chan_hdlr->out_data = 0;

      adc_chan_hdlr->buf_average_len = len_average;
      adc_chan_hdlr->buf_ptr = out_buf;
      /**< Unlock Interrupt */
      gpOS_interrupt_unlock();
      if( LLD_ADC_GetMode(svc_adc_handler->adc_id) == (LLD_ADC_ModeTy)LLD_ADC_SW_MODE_ENABLED)
      {
        while(LLD_ADC_GetCntReg(svc_adc_handler->adc_id) < 0xD);
        if( svc_adc_handler->adc_mode.adc_int_mode == ADC_INTERRUPT)
        {
          return gpOS_FAILURE;
        }
        else if( svc_adc_handler->adc_mode.adc_int_mode == ADC_NOINTERRUPT)
        {
          if( adc_chan_hdlr->buf_average_len == 4)
          {
            adc_chan_hdlr->out_data = LLD_ADC_GetAvgSampleReg(svc_adc_handler->adc_id);
          }
          else if( adc_chan_hdlr->buf_average_len == 1)
          {
            adc_chan_hdlr->out_data = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
          }
          if(adc_chan_hdlr->buf_ptr != NULL)
          {
            if( adc_chan_hdlr->buf_average_len == 4)
            {
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
            }
            else if( adc_chan_hdlr->buf_average_len == 1)
            {
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = adc_chan_hdlr->out_data;
            }
            buf_pos++;
            ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD1Reg(svc_adc_handler->adc_id);
            buf_pos++;
            ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD2Reg(svc_adc_handler->adc_id);
            buf_pos++;
            ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD3Reg(svc_adc_handler->adc_id);
            buf_pos++;
          }
        }
      }
      else if( svc_adc_handler->adc_configuration.mode == (LLD_ADC_ModeTy)LLD_ADC_NORMAL_MODE_ENABLED)
      {
        if( chan_id == 0)
        {
          svc_adc_handler->adc_configuration.sel = LLD_ADC_NoChanswitch;
          LLD_ADC_SetChanSel(svc_adc_handler->adc_id, svc_adc_handler->adc_configuration.sel);
        }
        if( svc_adc_handler->adc_mode.adc_int_mode == ADC_INTERRUPT)
        {
          LLD_ADC_Enable(svc_adc_handler->adc_id);
          if( adc_chan_hdlr->irq_src)
          {
           /* if( chan_id == 1)
            {
              //LLD_ADC_switchChan1(svc_adc_handler->adc_id);
            }   */
            gpOS_semaphore_wait(adc_chan_hdlr->done_sem);

            if( adc_chan_hdlr->buf_average_len == 4)
            {
              if (chan_id == 0)
              {
                adc_chan_hdlr->out_data = LLD_ADC_GetAvg0SampleReg(svc_adc_handler->adc_id);
              }
              else if ( chan_id == 1)
              {
                adc_chan_hdlr->out_data = LLD_ADC_GetAvg1SampleReg(svc_adc_handler->adc_id);
              }
            }
            else if( adc_chan_hdlr->buf_average_len == 1)
            {
              if( chan_id == 1)
              {
                LLD_ADC_switchChan1(svc_adc_handler->adc_id);
              }
              adc_chan_hdlr->out_data = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
            }
            if(adc_chan_hdlr->buf_ptr != NULL)
            {
              if( adc_chan_hdlr->buf_average_len == 1)
              {
                ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = adc_chan_hdlr->out_data;
              }
              else if( adc_chan_hdlr->buf_average_len == 4)
              {
                if( chan_id == 1)
                {
                  LLD_ADC_switchChan1(svc_adc_handler->adc_id);
                }
                ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
              }
              buf_pos++;
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD1Reg(svc_adc_handler->adc_id);
              buf_pos++;
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD2Reg(svc_adc_handler->adc_id);
              buf_pos++;
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD3Reg(svc_adc_handler->adc_id);
              buf_pos++;
            }
          }
          else
          {
           return gpOS_FAILURE;
          }
        }
        else if( svc_adc_handler->adc_mode.adc_int_mode == ADC_NOINTERRUPT)
        {
          if( chan_id == 0)
          {
            if( adc_chan_hdlr->buf_average_len == 4)
            {
              while(LLD_ADC_GetCntReg(svc_adc_handler->adc_id) <= 20);
              adc_chan_hdlr->out_data = LLD_ADC_GetAvg0SampleReg(svc_adc_handler->adc_id);
            }
            else if( adc_chan_hdlr->buf_average_len == 1)
            {
              adc_chan_hdlr->out_data = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
            }
            if(adc_chan_hdlr->buf_ptr != NULL)
            {
              if( adc_chan_hdlr->buf_average_len == 1)
              {
                ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] =adc_chan_hdlr->out_data;
              }
              else if( adc_chan_hdlr->buf_average_len == 4)
              {
                ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
              }
              buf_pos++;
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD1Reg(svc_adc_handler->adc_id);
              buf_pos++;
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD2Reg(svc_adc_handler->adc_id);
              buf_pos++;
              ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD3Reg(svc_adc_handler->adc_id);
              buf_pos++;
            }
          }
          else if( chan_id == 1)
          {
            LLD_ADC_SetChanSel(svc_adc_handler->adc_id, svc_adc_handler->adc_configuration.sel);
            chan_sel = svc_adc_handler->adc_configuration.sel;
            if(chan_sel == LLD_ADC_ChanswitchSel_64)
            {
              cnt_chan1_mask = LLD_ADC_chan1_cntmask_64;
              chan_switch_mask = LLD_ADC_chanswitchmask_64;
            }
            else if(chan_sel == LLD_ADC_ChanswitchSel_128)
            {
              cnt_chan1_mask = LLD_ADC_chan1_cntmask_128;
              chan_switch_mask = LLD_ADC_chanswitchmask_128;
            }
            else if(chan_sel == LLD_ADC_ChanswitchSel_256)
            {
              cnt_chan1_mask = LLD_ADC_chan1_cntmask_256;
              chan_switch_mask = LLD_ADC_chanswitchmask_256;
            }
            if( adc_chan_hdlr->buf_average_len == 4)
            {
              if((LLD_ADC_GetCntReg(svc_adc_handler->adc_id) & cnt_chan1_mask) >= (chan_switch_mask + 20))
              {
                adc_chan_hdlr->out_data = LLD_ADC_GetAvg1SampleReg(svc_adc_handler->adc_id);
                if(adc_chan_hdlr->buf_ptr != NULL)
                {
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD1Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD2Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD3Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                }
              }
              else
              {
                LLD_ADC_switchChan1(svc_adc_handler->adc_id);
                while((LLD_ADC_GetCntReg(svc_adc_handler->adc_id) & cnt_chan1_mask) < (chan_switch_mask + 20));
                adc_chan_hdlr->out_data = LLD_ADC_GetAvg1SampleReg(svc_adc_handler->adc_id);
                if(adc_chan_hdlr->buf_ptr != NULL)
                {
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD1Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD2Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD3Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                }
              }
            }
            else if( adc_chan_hdlr->buf_average_len == 1)
            {
              if((LLD_ADC_GetCntReg(svc_adc_handler->adc_id) & cnt_chan1_mask) >= (chan_switch_mask + 13))
              {
                adc_chan_hdlr->out_data = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
                if(adc_chan_hdlr->buf_ptr != NULL)
                {
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = adc_chan_hdlr->out_data;
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD1Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD2Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD3Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                }
              }
              else
              {
                LLD_ADC_switchChan1(svc_adc_handler->adc_id);
                while((LLD_ADC_GetCntReg(svc_adc_handler->adc_id) & cnt_chan1_mask) < (chan_switch_mask + 13));
                adc_chan_hdlr->out_data = LLD_ADC_GetADCD0Reg(svc_adc_handler->adc_id);
                if(adc_chan_hdlr->buf_ptr != NULL)
                {
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = adc_chan_hdlr->out_data;
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD1Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD2Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                  ((tU32 *)adc_chan_hdlr->buf_ptr)[buf_pos] = LLD_ADC_GetADCD3Reg(svc_adc_handler->adc_id);
                  buf_pos++;
                }
              }
            }
          }
        }

      }

      adc_chan_hdlr->buf_ptr          = NULL;
      adc_chan_hdlr->buf_average_len  = 0;
      adc_chan_hdlr->buf_ptr_pos      = 0;

      *out_data = adc_chan_hdlr->out_data;

      gpOS_semaphore_signal( svc_adc_handler->access_sem);

      return gpOS_SUCCESS;
    }
    else
    {
      return gpOS_FAILURE;
    }
    #endif
  }
}

