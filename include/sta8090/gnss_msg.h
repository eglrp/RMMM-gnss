/*****************************************************************************
   FILE:          <filename>
   PROJECT:       <project name>
   SW PACKAGE:    <software package name>
------------------------------------------------------------------------------
   DESCRIPTION:   <...>
------------------------------------------------------------------------------
   COPYRIGHT:     (c) <year> STMicroelectronics, (<group>) <site>
------------------------------------------------------------------------------
   Developers:
      AI:   Author Initials
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
   ----------+------+------------------------------------------------------
   yy.mm.dd  |  AI  | Original version
*****************************************************************************/

#ifndef GNSS_MSG_H
#define GNSS_MSG_H

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/


#define GNSS_MSG_MAX_READERS        8U           // MAX 32 readers
#define GNSS_MSG_MAX_IDS            32U          // 0 <= msg_id < GNSS_MSG_MAX_IDS
#define GNSS_MSG_MAX_IDS_MASK       0xFFFFFFFFU  // GNSS_MSG_MAX_IDS bits set
#define AMQ_LAST_BYTE               255U
/*lint -e961 */
#define GET_MSG_BODY_PTR(msg_ptr)   (((gnss_msg_ptr_t)(msg_ptr)) + sizeof(gnss_msg_header_t))
#define GET_MSG_HEADER_PTR(msg_ptr) (((gnss_msg_ptr_t)(msg_ptr)) - sizeof(gnss_msg_header_t))
/*lint +e961 */

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef tU16  gnss_msg_id_t;
typedef tU16  gnss_msg_size_t;
typedef tU32  gnss_msg_id_mask_t;
typedef tU32  gnss_msg_reader_mask_t;
typedef tU8*  gnss_msg_ptr_t;
typedef tU8   gnss_msg_reader_id_t;   // 1 <= gnss_msg_reader_id_t <= GNSS_MSG_MAX_READERS

typedef struct gnss_msg_header_t {
  gnss_msg_id_t           id;           // Message id
  gnss_msg_size_t         size;         // Message body size
  gnss_msg_reader_mask_t  reader_mask;  // Which readers are reading this message - depends on message id
} gnss_msg_header_t;

typedef struct gnss_msg_reader_handler_t {
  gnss_msg_ptr_t          read_ptr;   // local read pointer to message queue
  gnss_msg_reader_id_t    reader_id;  // reader id
  tU8                     wrap;       // wrap to global_read_ptr
  gnss_msg_id_mask_t      id_mask;    // specifies which message ids are read by this reader
  gpOS_semaphore_t*       reader_sem; // signals when there is a message to read for this reader
  //gpOS_semaphore_t*       reader_busy;
  void*                   next;       // points to the next reader in the list (or NULL for the last one)
} gnss_msg_reader_handler_t;

typedef struct gnss_msg_queue_t {
  gpOS_semaphore_t*   write_access_sem;
  gpOS_semaphore_t*   free_space_claim_sem;
  gpOS_semaphore_t*   global_ptr_sem;
  gnss_msg_ptr_t      begin_ptr;
  tU16                size;
  tU8                 waiting_free_space;
  tU8                 wrap;
  gnss_msg_ptr_t      write_ptr;
  gnss_msg_ptr_t      write_pending_ptr;
  gnss_msg_ptr_t      global_read_ptr;
} gnss_msg_queue_t;

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern gnss_error_t   gnss_msg_queue_init       ( gpOS_partition_t *partition, gnss_msg_ptr_t *mem_ptr, tU16 mem_size);
extern boolean_t      gnss_msg_queue_add_reader ( gnss_msg_reader_handler_t* reader_hnd_ptr);
extern void           gnss_msg_queue_add_msg_id ( gnss_msg_reader_handler_t* reader_hnd_ptr, gnss_msg_id_t msg_id);
extern gnss_msg_ptr_t gnss_msg_claim            ( gnss_msg_id_t msg_id, gnss_msg_size_t msg_size, const gpOS_clock_t *timeout);
extern void           gnss_msg_send             ( gnss_msg_ptr_t msg_ptr);
extern gnss_msg_ptr_t gnss_msg_receive          ( gnss_msg_reader_handler_t* reader_hnd_ptr, const gpOS_clock_t* timeout);
extern void           gnss_msg_release          ( gnss_msg_reader_handler_t * reader_hnd_ptr);

#endif
