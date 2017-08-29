/*****************************************************************************
   FILE:          errors.h
   PROJECT:       Emerald Kernel
   SW PACKAGE:    Common Header
------------------------------------------------------------------------------
   DESCRIPTION:   Definition of ERRORS Code with project scope.
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Developers:
      CS:   Cosimo Stornaiuolo
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
-------------+------+---------------------------------------------------------
 2005.08.09  |  CS  | Original version
*****************************************************************************/

#ifndef ERRORS_H
#define ERRORS_H

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************
| defines and macros (scope: module-local)
|-----------------------------------------------------------------------*/
/*    __ __ __ __ __ __ __ __ __ __ __ __ __ __ __ __
 MSB |__|__|__|__|__|__|__|__|__|__|__|__|__|__|__|__| LSB
     |  SW-Unit  | Type|       Error Number          |
     |__ 4 bit __|2 bit|__ __ __ _10 bit_ __ __ __ __|

SW-Unit:
      Used to identify the SW package
Type:
      Information (I)   0
      Warning     (W) 1
      Error       (E)   2
Error Number:
      It must belongs to the range reserved for the project
*/

/************************************************************************
| typedefs and structures (scope: module-local)
|-----------------------------------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif  /* ERRORS_H */

/* End of file */
