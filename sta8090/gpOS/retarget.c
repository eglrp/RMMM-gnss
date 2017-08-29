/*****************************************************************************
   FILE:          retarget.c
   PROJECT:       STA8090 GPS application
   SW PACKAGE:    STA8090 GPS library and application
------------------------------------------------------------------------------
   DESCRIPTION:   Module to implement C std library definitions for different
                  toolchains
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Created by : Fulvio boggia
           on : 2007.07.25
*****************************************************************************/

/*****************************************************************************
   includes
*****************************************************************************/

#if defined( __GNUC__ )

#include "typedefs.h"
#include "gpOS_crt.h"
#include "gpOS.h"

#include <errno.h>
#include <string.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <limits.h>

#if defined( _REENT )
#include <sys/stat.h>
#include <sys/reent.h>
#endif

#endif

/*****************************************************************************
   external declarations
*****************************************************************************/

#if defined( __GNUC__ )
extern int  _svfprintf_r (struct _reent *, FILE *f, const char *fmt, va_list ap); /*lint !e970 Directive 4.6, Use of modifier or type 'int' outside of a typedef */
extern void _exit( int);  /*lint !e970 Directive 4.6, Use of modifier or type 'int' outside of a typedef */
#endif

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

#if defined( __GNUC__ )
#ifndef __PTRDIFF_TYPE__
#define __PTRDIFF_TYPE__ long int
typedef __PTRDIFF_TYPE__ ptrdiff_t;
#endif
#endif

/*****************************************************************************
   global variable definitions  (scope: module-exported)
*****************************************************************************/

/*****************************************************************************
   global variable definitions (scope: module-local)
*****************************************************************************/

#if defined( __GNUC__ )
static gpOS_semaphore_t *crt_reent_sem = NULL;
#endif

/*****************************************************************************
   function prototypes (scope: module-local)
*****************************************************************************/

#if defined( __GNUC__ )
static void _crt_reent_init   ( void);
static void _crt_reent_lock   ( void);
static void _crt_reent_unlock ( void);
#endif

/*****************************************************************************
   function implementations (scope: module-local)
*****************************************************************************/

#if defined( __GNUC__ )
static void _crt_reent_init( void)
{
  crt_reent_sem = gpOS_semaphore_create(SEM_FIFO, 1);
}

static void _crt_reent_lock( void)
{
  gpOS_semaphore_wait( crt_reent_sem);
}

static void _crt_reent_unlock( void)
{
  gpOS_semaphore_signal( crt_reent_sem);
}
#endif

/*****************************************************************************
   function implementations (scope: module-exported)
*****************************************************************************/

#if defined( __ARMCC_VERSION)
void gpOS_crt_init( void)
{
}
#endif

#if defined( __GNUC__ )
#if defined( _REENT )
void gpOS_crt_init( void)
{
  _crt_reent_init();
}

// libc retargeting to share reent structure between tasks
int vsnprintf( char *str, size_t size, const char *fmt, va_list ap)
{
  int ret;
  FILE f;

  _crt_reent_lock();

  if (size > INT_MAX)
    {
      _REENT->_errno = EOVERFLOW;
      return EOF;
    }
  f._flags = __SWR | __SSTR;
  f._bf._base = f._p = (tUChar *) str;
  f._bf._size = f._w = (size > 0 ? size - 1 : 0);
  f._file = -1;  /* No file. */
  ret = _svfprintf_r (_REENT, &f, fmt, ap);
  if (ret < EOF)
    _REENT->_errno = EOVERFLOW;
  if (size > 0)
    *f._p = 0;

  _crt_reent_unlock();

  return ret;
}

int sprintf( char *s, const char *format, ...)
{
  tInt out_chars = 0;

  _crt_reent_lock();

  va_list ap;

  va_start( ap, format);

  out_chars = vsprintf( s, format, ap);

  va_end( ap);

  _crt_reent_unlock();

  return out_chars;
}

int sscanf( const char *s, const char *format, ...)
{
  tInt in_chars = 0;

  _crt_reent_lock();

  va_list ap;

  va_start( ap, format);

  in_chars = vsscanf( s, format, ap);

  va_end( ap);

  _crt_reent_unlock();

  return in_chars;
}

// libc retargeting needed for linking
void __malloc_lock( struct _reent *reent)
{
  gpOS_memory_lock();
}

void __malloc_unlock( struct _reent *reent)
{
  gpOS_memory_unlock();
}

void * _sbrk_r( struct _reent *r, ptrdiff_t incr)
{
  void *result;

  gpOS_memory_partition_extend_heap( incr, &result);

  if( result == NULL)
  {
    __errno_r(r) = ENOMEM;
    return (void *)-1;
  }

  return result;
}

int _read_r(struct _reent *r, int file, char * ptr, int len)
{
  (void)r;

  (void)file;
  (void)ptr;
  (void)len;

  __errno_r(r) = EINVAL;
  return -1;
}

/***************************************************************************/

int _lseek_r(struct _reent *r, int file, int ptr, int dir)
{
  (void)r;
  (void)file;
  (void)ptr;
  (void)dir;

  return 0;
}

/***************************************************************************/

int _write_r(struct _reent *r, int file, char * ptr, int len)
{
  (void)r;
  (void)file;
  (void)ptr;

  return len;
}

/***************************************************************************/

int _close_r(struct _reent *r, int file)
{
  (void)r;
  (void)file;

  return 0;
}

/***************************************************************************/

int _fstat_r(struct _reent *r, int file, struct stat * st)
{
  (void)r;
  (void)file;

  return 0;
}

/***************************************************************************/

int _isatty_r(struct _reent *r, int fd)
{
  (void)r;
  (void)fd;

  return 1;
}

int _kill( int pid, int sig)
{
  _exit( pid);
  return 0;
}

int _getpid( int n)
{
  _exit( n);
  return 0;
}
#else
void gpOS_crt_init( void)
{
  _crt_reent_init();
}

void * _sbrk( ptrdiff_t incr)
{
  void *result;

  gpOS_memory_partition_extend_heap( incr, &result);

  if( result == NULL)
  {
    return (void *)-1;
  }

  return result;
}
#endif
#endif
