/*-------------------------------------------*/
/* Integer type definitions for FatFs module */
/*-------------------------------------------*/

#ifndef _INTEGER
#define _INTEGER

#ifdef _WIN32	/* FatFs development platform */

#include <windows.h>
#include <tchar.h>

#else			/* Embedded platform */

#include "typedefs.h"

/* These types must be 16-bit, 32-bit or larger integer */
typedef tInt				INT;
typedef tUInt	UINT;

/* These types must be 8-bit integer */
typedef tChar			CHAR;
typedef tU8	UCHAR;
typedef tU8	BYTE;

/* These types must be 16-bit integer */
typedef tShort			SHORT;
typedef tU16	USHORT;
typedef tU16	WORD;
typedef tU16	WCHAR;

/* These types must be 32-bit integer */
typedef tLong			LONG;
typedef tULong	ULONG;
typedef tULong	DWORD;

#endif

#endif
