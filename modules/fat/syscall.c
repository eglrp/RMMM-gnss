/*------------------------------------------------------------------------*/
/* Sample code of OS dependent controls for FatFs R0.08                   */
/* (C)ChaN, 2010                                                          */
/*------------------------------------------------------------------------*/

#include <string.h>		/* ANSI memory controls */

#include "ff.h"
#include "diskio.h"

#include "svc_sdi.h"
#include "lld_rtc.h"
#include "lld_gpio.h"

DSTATUS disk_initialize (
  BYTE Drive           /* Physical drive number */
)
{
  if( svc_sdi_open_port( Drive, gpOS_INTERRUPT_NOPRIORITY) == gpOS_FAILURE)
  {
    return STA_NOINIT;
  }

  return 0;
}

DSTATUS disk_status (
  BYTE Drive     /* Physical drive number */
)
{
  return 0;
}

DRESULT disk_read (
  BYTE Drive,          /* Physical drive number */
  BYTE* Buffer,        /* Pointer to the read data buffer */
  DWORD SectorNumber,  /* Start sector number */
  BYTE SectorCount     /* Number of sectros to read */
)
{
//  if (LLD_SDI_ReadBlocks( SD_SDIO_MMC_REG_START_ADDR, &sdi_system_context, 1,
//                          SectorNumber, (tU32 *)Buffer, 512, SectorCount ))
  if( svc_sdi_read( Drive, SectorNumber, SectorCount, 512, Buffer) == gpOS_FAILURE)
  {
    return RES_ERROR;
  }
  return RES_OK;
}

DRESULT disk_write (
  BYTE Drive,          /* Physical drive number */
  const BYTE* Buffer,  /* Pointer to the write data (may be non aligned) */
  DWORD SectorNumber,  /* Sector number to write */
  BYTE SectorCount     /* Number of sectors to write */
)
{
  tU32 i;
  tU8 temp_buffer[512];

  for (i=0; i<SectorCount; i++)
  {
    memcpy(temp_buffer, (Buffer + (i * 512)), 512);
//    if( LLD_SDI_WriteBlocks( SD_SDIO_MMC_REG_START_ADDR, &sdi_system_context, 1,
//                             (SectorNumber + i), temp_buffer, 512, 1 ))
    if( svc_sdi_write( Drive, SectorNumber + i, 1, 512, temp_buffer) == gpOS_FAILURE)
    {
      return RES_ERROR;
    }
  }
  return RES_OK;
}

DRESULT disk_ioctl (
  BYTE Drive,      /* Drive number */
  BYTE Command,    /* Control command code */
  void* Buffer     /* Parameter and data buffer */
)
{
  DRESULT res = RES_OK;

  switch (Command)
  {
    case CTRL_SYNC:
      if ( svc_sdi_is_port_busy( Drive) == TRUE)
        res = RES_NOTRDY;
      break;
    default:
      break;
  }
  return res;
}

#if _FS_REENTRANT
/*------------------------------------------------------------------------*/
/* Create a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount function to create a new
/  synchronization object, such as semaphore and mutex. When a FALSE is
/  returned, the f_mount function fails with FR_INT_ERR.
*/
tInt ff_cre_syncobj (	/* TRUE:Function succeeded, FALSE:Could not create due to any error */
	BYTE vol,			/* Corresponding logical drive being processed */
	_SYNC_t *sobj		/* Pointer to return the created sync object */
)
{
  *sobj = gpOS_semaphore_create( SEM_FIFO, 1);

  if( *sobj == NULL )
  {
    gpOS_semaphore_delete( *sobj);
    return FALSE;
  }
	return TRUE;
}

/*------------------------------------------------------------------------*/
/* Delete a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount function to delete a synchronization
/  object that created with ff_cre_syncobj function. When a FALSE is
/  returned, the f_mount function fails with FR_INT_ERR.
*/
tInt ff_del_syncobj (	/* TRUE:Function succeeded, FALSE:Could not delete due to any error */
	_SYNC_t sobj		/* Sync object tied to the logical drive to be deleted */
)
{
  gpOS_semaphore_delete( sobj);

	return TRUE;
}

/*------------------------------------------------------------------------*/
/* Request Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on entering file functions to lock the volume.
/  When a FALSE is returned, the file function fails with FR_TIMEOUT.
*/
tInt ff_req_grant (	/* TRUE:Got a grant to access the volume, FALSE:Could not get a grant */
	_SYNC_t sobj	/* Sync object to wait */
)
{
  gpOS_semaphore_wait( sobj);

	return TRUE;
}

/*------------------------------------------------------------------------*/
/* Release Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on leaving file functions to unlock the volume.
*/
void ff_rel_grant (
	_SYNC_t sobj	/* Sync object to be signaled */
)
{
  gpOS_semaphore_signal( sobj);
}

#endif

/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
DWORD get_fattime (void)
{
  return LLD_RTC_GetDataRegister();
}
