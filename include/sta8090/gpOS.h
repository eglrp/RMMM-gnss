/*****************************************************************************
   FILE:          gpOS.h
   PROJECT:       ARM GPS library
   SW PACKAGE:    Common Header
------------------------------------------------------------------------------
   DESCRIPTION:   Define with project scope.
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2014 STMicroelectronics, Le Mans (FRANCE)
------------------------------------------------------------------------------
   Developers:
      CS:   Fabrice Pointeau
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
-------------+------+---------------------------------------------------------
 2014.04.30  |  FP  | Original version
*****************************************************************************/

#ifndef GPOS_H
#define GPOS_H

#ifdef FREERTOS
#include "FreeRTOS.h"
#include "FR_utils.h"
#include "FR_memory.h"
#include "gpOS_common.h"
#else
#include "gpOS_common.h"
#endif

#endif  /* GPOS_H */

/* End of file */
