/*****************************************************************************
   FILE:          typedefs.h
   PROJECT:       ARM GPS library
   SW PACKAGE:    Common Header
------------------------------------------------------------------------------
   DESCRIPTION:   Define with project scope.
------------------------------------------------------------------------------
   COPYRIGHT:     (c) 2005 STMicroelectronics, (S2S - SWD) Napoli (ITALY)
------------------------------------------------------------------------------
   Developers:
      CS:   Fulvio Boggia
------------------------------------------------------------------------------
   HISTORY:

   Date      | A.I. | Description
-------------+------+---------------------------------------------------------
 2005.09.07  |  FB  | Original version
*****************************************************************************/

#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------
// defines (scope: global)
//-----------------------------------------------------------------------
// Description:
// Further keywords (section 5.1.2)

//#if defined( __ARMCC_VERSION)
 // #define INLINE         __inline  // specifies a routine to be in line
//#elif defined(__GNUC__)
//  #define INLINE         inline    // specifies a routine to be in line
//#endif

#define CONST          const    // defines a constant item

/*lint -e(9023,9024) Multiple use of '#/##' operators [MISRA 2012 Rule 20.10, advisory]*/
#define DEFINE_TYPES(name, base)        \
 typedef base              t ## name;    \
 typedef base *            tP  ## name;  \
 typedef volatile base     tV  ## name;  \
 typedef volatile base *   tVP ## name;

//-----------------------------------------------------------------------
// typedefs (scope: global)
//-----------------------------------------------------------------------

#if defined( __ARMCC_VERSION)
#if (__ARMCC_VERSION >= 300586)
#define tVoid           void
#else
typedef void            tVoid;
#endif
#else
typedef void            tVoid;
#endif

#if defined( __ARMCC_VERSION)
#ifndef __size_t
typedef unsigned int size_t;
#endif
#endif  //__ARMCC_VERSION

#if defined( __GNUC__)
#ifndef __SIZE_T_DEFINED
typedef unsigned int size_t;
#endif
#endif  //__GNUC__
#ifdef WIN32
#ifndef size_t
typedef unsigned int size_t;
#endif
#endif  //WIN32


DEFINE_TYPES(       Bool, unsigned char)
DEFINE_TYPES(         U8, unsigned char)
DEFINE_TYPES(         S8, signed char)
DEFINE_TYPES(        U16, unsigned short)
DEFINE_TYPES(        S16, signed short)
DEFINE_TYPES(        U32, unsigned long)
DEFINE_TYPES(        S32, signed long)
DEFINE_TYPES(       Char, char)
DEFINE_TYPES(      UChar, unsigned char)
DEFINE_TYPES(      SChar, signed char)
DEFINE_TYPES(     UShort, unsigned short)
DEFINE_TYPES(      Short, signed short)
DEFINE_TYPES(      ULong, unsigned long)
DEFINE_TYPES(       Long, signed long)
DEFINE_TYPES(  ULongLong, unsigned long long)
DEFINE_TYPES(   LongLong, long long)
DEFINE_TYPES(       UInt, unsigned int)
DEFINE_TYPES(        Int, signed int)
DEFINE_TYPES(      Float, float)
DEFINE_TYPES(     Double, double)
DEFINE_TYPES(    LDouble, long double)
DEFINE_TYPES(    Unicode, unsigned short)
DEFINE_TYPES(       Size, size_t)

DEFINE_TYPES(  UBitField, unsigned int)
DEFINE_TYPES(      SEnum, signed int)
DEFINE_TYPES(      UEnum, unsigned int)

typedef char *            tString;
typedef const char*       tCString;

#ifndef __linux__
typedef unsigned int      uint;
#endif
typedef tS8               int8;
typedef tS16              int16;
typedef tS32              int32;
typedef tU8               uint8;
typedef tU16              uint16;
typedef tU32              uint32;
typedef tS8               sint8;
typedef tS16              sint16;
typedef tS32              sint32;

typedef tBool             Bool;
typedef tBool             boolean_t;

#ifndef __boolean
typedef tBool             boolean;
#endif

/* -- Target-dependent types: -- */
/* WINDOWS / NUCLEUS */
#if defined (OS_WIN) || defined (OS_NUCLEUS) || defined (WIN32)
DEFINE_TYPES(       U64, unsigned __int64)
DEFINE_TYPES(       S64, signed __int64)
#elif defined (__ARMCC_VERSION) || defined (__GNUC__)
DEFINE_TYPES(       U64, unsigned long long)
DEFINE_TYPES(       S64, signed long long)
#else
#error The file typedefs.h is not prepared for the current target!
#endif

typedef tS64              sint64;

#ifdef __cplusplus
}
#endif

#endif  /* TYPEDEFS_H */
/* End of file */
