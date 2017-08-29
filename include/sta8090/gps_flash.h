/*****************************************************************************
   FILE:          flash.c
   PROJECT:       Cartesio SW
   SW PACKAGE:    NVM low level driver
------------------------------------------------------------------------------
   DESCRIPTION:   This is the low level driver to use NVM.
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Developers:
      CS:   Cosimo Stornaiuolo
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
-------------+------+---------------------------------------------------------
 2007.01.22  |  CS  | Original version
*****************************************************************************/

#ifndef GPS_FLASH_H
#define GPS_FLASH_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "typedefs.h"
#include "gpOS.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef void *gps_flash_config_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gpOS_error_t   flash_init                ( const gps_flash_config_t *, tU32 *, tU32);
extern gpOS_error_t   flash_erase_block         ( const tUInt *, const tUInt);
extern gpOS_error_t   flash_erase_block_check   ( const tUInt);
extern gpOS_error_t   flash_write_area          ( const tUInt*, const tUInt*, const tUInt);
extern gpOS_error_t   flash_write_word          ( const tUInt*, const tUInt);
extern void           flash_suspend             ( void);

#endif /* FLASH_H */
