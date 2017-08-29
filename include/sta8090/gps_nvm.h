
#ifndef NVM_H
#define NVM_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gpOS.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define NVM_MAX_ITEMS       2048
#define NVM_MAX_COPIES      1024
#define NVM_MAX_ITEM_SIZE   4096

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum nvm_status_s
{
  NVM_NO_ERROR      = 0,
  NVM_ERROR         = 1,
  NVM_ITEM_VALID    = 2,
  NVM_ITEM_NOTVALID = 3,
  NVM_REC_BUSY      = 4,
  NVM_SWAP_ONGOING  = 5,
  NVM_SWAP_REQUIRED = 6
} nvm_status_t;

typedef struct nvm_swap_request_s
{
  boolean_t swap_required;
  tShort    id;
  tShort    item;
  tShort    current_copy;
  tShort    max_copy;
}nvm_swap_request_t;

typedef tUInt nvmid_t;
typedef void *nvm_address_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

extern nvm_status_t nvm_open                        ( const tUInt, nvm_address_t, nvm_address_t, const tUInt, const tU32);
extern nvm_status_t nvm_open_p                      ( gpOS_partition_t *, const tUInt, nvm_address_t, nvm_address_t, const tUInt, const tU32);
extern nvm_status_t nvm_create                      ( const nvmid_t, const tInt, const tInt, const tInt);
extern nvm_status_t nvm_write                       ( const nvmid_t, const tInt, const void *);
extern nvm_status_t nvm_write_conditioned           ( const nvmid_t, const tInt, const void *, const boolean_t);
extern nvm_status_t nvm_read                        ( const nvmid_t, const tInt, void * *);
extern nvm_status_t nvm_copy                        ( const nvmid_t, const tInt, void *);
extern nvm_status_t nvm_set_item_invalid            ( const nvmid_t, const tInt);
extern nvm_status_t nvm_set_item_invalid_conditioned( const nvmid_t, const tInt, const boolean_t);
extern nvm_status_t nvm_swap                        ( boolean_t);
extern nvm_status_t nvm_suspend                     ( void);
extern nvm_status_t nvm_restart                     ( void);
extern void         nvm_reading_lock                ( void);
extern void         nvm_reading_unlock              ( void);
extern nvm_status_t nvm_set_all_item_invalid        ( void);
extern nvm_status_t nvm_status_check                ( void);
extern void         nvm_wait_swap_end               ( void);
extern nvm_status_t nvm_get_record_info             ( const nvmid_t, tUInt *, tU16 *, tU16 *, tU16 *, tU16 *);
extern void         nvm_swap_triggering             ( void);
extern void         nvm_get_recovering_info         ( tU16 *, tU16 *, tUInt *);
#ifdef __cplusplus
}
#endif
#endif /* NVM_H */
