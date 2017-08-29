/*
 * freeRTOSi.h
 *
 * Copyright (C) STMicroelectronics Ltd. 2000
 *
 */

#ifndef FREERTOSI_H
#define FREERTOSI_H

/*****************************************************************************
   includes
*****************************************************************************/
#include "freeRTOSConfig.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#if ( configGENERATE_RUN_TIME_STATSCustom == 1 )
#define FREERTOS_PROFILING  1
#endif

#define FREERTOS_IRQ_HOOKS

extern unsigned int interrupt_nested_count;           /**< Interrupt nesting counter */

#endif /* FREERTOS_TYPES_H */
