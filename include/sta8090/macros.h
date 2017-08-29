/*****************************************************************************
   FILE:          macros.h
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

#ifndef MACROS_H
#define MACROS_H

#include "defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************
| MACROS (scope: global)
|-----------------------------------------------------------------------*/

/*
   MCR_FIELD returns the field aligned of a value.
   Arguments:
   - p = position of field in the variable/register
   - m = mask for the field
   - v = value to set
*/
#define MCR_FIELD(p,m,v) \
   (((v) & (m)) << (p))

/*
   MCR_SETFIELD set a field in a variable/register.
   Arguments:
   - r = variable/register to modify
   - p = position of field in the variable/register
   - m = mask for the field
   - v = value to set
*/
#define MCR_SETFIELD(r,p,m,v) \
   ((r) = ((r) & ~((m) << (p))) | (((v) & (m)) << (p)))

/*
   MCR_GETFIELD get a field from a variable/register.
   Arguments:
   - r = variable/register to modify
   - p = position of field in the variable/register
   - m = mask for the field
*/
#define MCR_GETFIELD(r,p,m) \
   (((r) >> (p)) & (m))

/*
   GETBIT get a bit from a variable/register.
   Arguments:
   - r = variable/register to read
   - v = position of the bit
*/
#define MCR_GETBIT(r,v) \
   (((tU32)(r) >> (tU32)(v)) & 0x1U)

/*
   MCR_ISBITSET check if a bit in a variable/register is set.
   Arguments:
   - r = variable/register to check
   - v = position of the bit
*/
#define MCR_ISBITSET(r,v) \
   (MCR_GETBIT(r,v) == 0x1U)

/*
   MCR_SETBIT sets a bit in a variable/register.
   Arguments:
   - r = variable/register to write
   - v = position of the bit
*/
#define MCR_SETBIT(r,v) \
   ((r) |= BIT_x(v))

/*
   MCR_CLEARBIT clears a bit in a variable/register.
   Arguments:
   - r = variable/register to write
   - v = position of the bit
*/
#define MCR_CLEARBIT(r,v) \
   ((r) &= ~BIT_x(v))

/*
   MCR_BITMASK creates a bit mask of given bits.
   Arguments:
   - b = number of bits
*/
#define MCR_BITMASK(b) \
   (BIT_x(b) - 1)

/*
   MCR_GETREG32 gets a 32 bits register value.
   Arguments:
   - r = register to write
*/
#define MCR_GETREG32(r) \
   (*((volatile tUInt *)&(r)))

/*
   MCR_SETREG32 sets a 32 bits register to a value.
   Arguments:
   - r = register to write
   - v = position of the bit
*/
#define MCR_SETREG32(r,v) \
   (*((volatile tUInt *)&(r)) = (v))

/*
   MCR_ABS evaluates absolute value of a variable
   Arguments:
   - v = variable to convert
*/
#define MCR_ABS(v) \
  ((((tS32)(v)) > 0) ? ((tS32)(v)) : (-((tS32)(v))))


/*
   MCR_ABS_16 evaluates absolute value of a signed short variable
   Arguments:
   - v = variable to convert
*/
#define MCR_ABS_16(v) \
  ((((tS16)(v)) > 0) ? ((tS16)(v)) : (-((tS16)(v))))

/*
   MCR_SGN evaluates sign of a variable
   Arguments:
   - v = variable to convert
*/
#define MCR_SGN(v) \
  ((v) > 0 ? 1 : -1)

/*
   MCR_DOUBLE_TO_FP32
   Arguments:
   - v = variable to convert (must be < 1)
*/
#define MCR_DOUBLE_TO_FP32(v) \
  ((tInt)((tDouble)(v)*(tDouble)((tUInt)1U<<31U)))

/*
   MCR_DOUBLE_TO_FP32_QM
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 31)

   if fabs(v)>=max return either +/-max otherwise return v*(1<<(m))
*/
#define MCR_DOUBLE_TO_FP32_QM(v,m) \
((MCR_ABS(v) >= (tUInt)((1U<<(31U-(m)))))? ((((tInt)(v))>0)?((fp_s32_t)(FP_SCALE32)):(-((fp_s32_t)(FP_SCALE32)))):(tInt)((tDouble)(v)*(tDouble)((tUInt)1U<<(m))))

/*
   MCR_FP32_QM_TO_DOUBLE
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 31)
*/
#define MCR_FP32_QM_TO_DOUBLE(v,m) \
  (((tDouble)(v)/(tDouble)((tUInt)1U<<(m))))

/*
   MCR_DOUBLE_TO_FP16_QM
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 15)

      if abs(v)>=max return either +/-max otherwise return v*(1<<(m))
*/
#define MCR_DOUBLE_TO_FP16_QM(v,m) \
 ((MCR_ABS_16(v) >= (tS16)((1U<<(15U-(m)))))? ((((tInt)(v))>0)?((fp_s16_t)(FP_SCALE16)):(-((fp_s16_t)(FP_SCALE16)))):(fp_s16_t)((tDouble)(v)*(tDouble)((tUInt)1U<<(m))))

/*
   MCR_FP16_QM_TO_DOUBLE
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 15)
*/
#define MCR_FP16_QM_TO_DOUBLE(v,m) \
  (((tDouble)(v)/(tDouble)((tUInt)1U<<(m))))


/*
   MCR_DOUBLE_TO_UFP32_QM
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 32)
     if 0<v<max return v*(1<<(m)) otherwise return max if v>=max otherwise return 0 if v<0
*/
#define MCR_DOUBLE_TO_UFP32_QM(v,m) \
(((tInt)(v)>0)?((MCR_ABS(v)<(tUInt)((((tULongLong)(1ULL))<<(32ULL-(m)))))?((tUInt)((tDouble)(v)*(tDouble)((tUInt)1U<<(m)))):(tUInt)((((tULongLong)(1ULL))<<32ULL)-1ULL)):0)

/*
   MCR_FPU32_QM_TO_DOUBLE
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 32)
*/
#define MCR_FPU32_QM_TO_DOUBLE(v,m) \
  (((tDouble)(v)/(tDouble)((tUInt)1U<<(m))))

/*
   MCR_DOUBLE_TO_FP32_2_QM
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 62)
   This macro is for fixed point numbers with negative integer bits.
*/
#define MCR_DOUBLE_TO_FP32_2_QM(v,m) \
  ((tInt)((tDouble)(v)*(tDouble)((tULongLong)1ULL<<(m))))

/*
   MCR_FP32_2_QM_TO_DOUBLE
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 62)
   This macro is for fixed point numbers with negative integer bits.
*/
#define MCR_FP32_2_QM_TO_DOUBLE(v,m) \
  (((tDouble)(v)/(tDouble)((tULongLong)1ULL<<(m))))

/*
   MCR_DOUBLE_TO_FP64_QM
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 62)
*/
#define MCR_DOUBLE_TO_FP64_QM(v,m) \
  ((tLongLong)((tDouble)(v)*(tDouble)((tULongLong)1ULL<<(m))))

/*
   MCR_FP64_QM_TO_DOUBLE
   Arguments:
   - v = variable to convert
   - m = bit of the fractional part(must be < 62)
*/
#define MCR_FP64_QM_TO_DOUBLE(v,m) \
  (((tDouble)(v)/(tDouble)((tULongLong)1ULL<<(m))))

/*
   MCR_ALIGN_CHAR2LONG aligns a characters number to the minimum number of
   integer needed to store them.
   Arguments:
   - c = number of characters
*/
#define MCR_ALIGN_CHAR2LONG(c)  (((tU8)(((c)+sizeof(tU32))/sizeof(tU32)))*sizeof(tU32))

/*
   MCR_VERSION_STRING builds a version string
   Arguments:
   - c = component name
   - v = version number
*/
#define MCR_VERSION_STRING( c, v) \
  c "_" v "_" TOOLCHAIN_STRING

/*lint -e(9024) : '#/##' operator used in macro 'cat' [MISRA 2012 Rule 20.10, advisory]*/
#define MCR_MACROTOSTRING_BUILD(x)    #x
#define MCR_MACROTOSTRING(x)          MCR_MACROTOSTRING_BUILD(x)

/*lint -e(9024) : '#/##' operator used in macro 'cat' [MISRA 2012 Rule 20.10, advisory]*/
#define cat(x,y) x ## y
#define xcat(x,y) cat(x,y)

// Pheripherals regs manipulation
/*! @brief It reserves a memory space of 4 bytes (32 bits). It's used in the peripherals memory map definition */
#define gap32(x) tU32 xcat(UNUSED,__LINE__)[x]

/*! @brief It reserves a memory space of 2 bytes (16 bits). It's used in the peripherals memory map definition */
#define gap16(x) tU16 xcat(UNUSED,__LINE__)[x]

/*! @brief It reserves a memory space of 2 bytes (16 bits). It's used in the peripherals memory map definition */
#define gap(x) tU16 xcat(UNUSED,__LINE__)[x]

/*! @brief It reserves a memory space of 1 byte (8 bits). It's used in the peripherals memory map definition */
#define gap8(x) tU8 xcat(UNUSED,__LINE__)[x]

/*! @brief It reserves a memory space of 4 bytes (32 bits). It's used in the peripherals memory map definition
* It's an alias of @ref gap32 */
#define intra32 tU32 xcat(UNUSED,__LINE__)

/*! @brief It reserves a memory space of 2 bytes (16 bits). It's used in the peripherals memory map definition
* It's an alias of @ref gap16 and @ref gap*/
#define intra16 tU16 xcat(UNUSED,__LINE__)

/*! @brief It reserves a memory space of 1 byte (8 bits). It's used in the peripherals memory map definition
* It's an alias of @ref gap8 */
#define intra8 tU8 xcat(UNUSED,__LINE__)

/*! @brief It implements a simple infinite loop */
#define FOREVER { while (1) {}; }

/*! @brief It's the mask to extract the bit 0 */
#define BIT_0    ((tU32)1U<<0U)
/*! @brief It's the mask to extract the bit 1 */
#define BIT_1    ((tU32)1U<<1U)
/*! @brief It's the mask to extract the bit 2 */
#define BIT_2    ((tU32)1U<<2U)
/*! @brief It's the mask to extract the bit 3 */
#define BIT_3    ((tU32)1U<<3U)
/*! @brief It's the mask to extract the bit 4 */
#define BIT_4    ((tU32)1U<<4U)
/*! @brief It's the mask to extract the bit 5 */
#define BIT_5    ((tU32)1U<<5U)
/*! @brief It's the mask to extract the bit 6 */
#define BIT_6    ((tU32)1U<<6U)
/*! @brief It's the mask to extract the bit 7 */
#define BIT_7    ((tU32)1U<<7U)
/*! @brief It's the mask to extract the bit 8 */
#define BIT_8    ((tU32)1U<<8U)
/*! @brief It's the mask to extract the bit 9 */
#define BIT_9    ((tU32)1U<<9U)
/*! @brief It's the mask to extract the bit 10 */
#define BIT_10   ((tU32)1U<<10U)
/*! @brief It's the mask to extract the bit 11 */
#define BIT_11   ((tU32)1U<<11U)
/*! @brief It's the mask to extract the bit 12 */
#define BIT_12   ((tU32)1U<<12U)
/*! @brief It's the mask to extract the bit 13 */
#define BIT_13   ((tU32)1U<<13U)
/*! @brief It's the mask to extract the bit 14 */
#define BIT_14   ((tU32)1U<<14U)
/*! @brief It's the mask to extract the bit 15 */
#define BIT_15   ((tU32)1U<<15U)
/*! @brief It's the mask to extract the bit 16 */
#define BIT_16   ((tU32)1U<<16U)
/*! @brief It's the mask to extract the bit 17 */
#define BIT_17   ((tU32)1U<<17U)
/*! @brief It's the mask to extract the bit 18 */
#define BIT_18   ((tU32)1U<<18U)
/*! @brief It's the mask to extract the bit 19 */
#define BIT_19   ((tU32)1U<<19U)
/*! @brief It's the mask to extract the bit 20 */
#define BIT_20   ((tU32)1U<<20U)
/*! @brief It's the mask to extract the bit 21 */
#define BIT_21   ((tU32)1U<<21U)
/*! @brief It's the mask to extract the bit 22 */
#define BIT_22   ((tU32)1U<<22U)
/*! @brief It's the mask to extract the bit 23 */
#define BIT_23   ((tU32)1U<<23U)
/*! @brief It's the mask to extract the bit 24 */
#define BIT_24   ((tU32)1U<<24U)
/*! @brief It's the mask to extract the bit 25 */
#define BIT_25   ((tU32)1U<<25U)
/*! @brief It's the mask to extract the bit 26 */
#define BIT_26   ((tU32)1U<<26U)
/*! @brief It's the mask to extract the bit 27 */
#define BIT_27   ((tU32)1U<<27U)
/*! @brief It's the mask to extract the bit 28 */
#define BIT_28   ((tU32)1U<<28U)
/*! @brief It's the mask to extract the bit 39 */
#define BIT_29   ((tU32)1U<<29U)
/*! @brief It's the mask to extract the bit 30 */
#define BIT_30   ((tU32)1U<<30U)
/*! @brief It's the mask to extract the bit 31 */
#define BIT_31   ((tU32)1U<<31U)

/*! @brief It's the mask to extract the bit x */
#define BIT_x(x) ((tU32)1U<<(x))

/*! @brief Generic constant for accessing all bits in a 32 bit word */
#define BIT_ALL  ((tU32)0xFFFFFFFFU)

/*! @brief It's the mask to extract the bit 0 */
#define BIT64_0    ((tU64)1UL<<0UL)
/*! @brief It's the mask to extract the bit 1 */
#define BIT64_1    ((tU64)1UL<<1UL)
/*! @brief It's the mask to extract the bit 2 */
#define BIT64_2    ((tU64)1UL<<2UL)
/*! @brief It's the mask to extract the bit 3 */
#define BIT64_3    ((tU64)1UL<<3UL)
/*! @brief It's the mask to extract the bit 4 */
#define BIT64_4    ((tU64)1UL<<4UL)
/*! @brief It's the mask to extract the bit 5 */
#define BIT64_5    ((tU64)1UL<<5UL)
/*! @brief It's the mask to extract the bit 6 */
#define BIT64_6    ((tU64)1UL<<6UL)
/*! @brief It's the mask to extract the bit 7 */
#define BIT64_7    ((tU64)1UL<<7UL)
/*! @brief It's the mask to extract the bit 8 */
#define BIT64_8    ((tU64)1UL<<8UL)
/*! @brief It's the mask to extract the bit 9 */
#define BIT64_9    ((tU64)1UL<<9UL)
/*! @brief It's the mask to extract the bit 10 */
#define BIT64_10   ((tU64)1UL<<10UL)
/*! @brief It's the mask to extract the bit 11 */
#define BIT64_11   ((tU64)1UL<<11UL)
/*! @brief It's the mask to extract the bit 12 */
#define BIT64_12   ((tU64)1UL<<12UL))
/*! @brief It's the mask to extract the bit 13 */
#define BIT64_13   ((tU64)1UL<<13UL)
/*! @brief It's the mask to extract the bit 14 */
#define BIT64_14   ((tU64)1UL<<14UL)
/*! @brief It's the mask to extract the bit 15 */
#define BIT64_15   ((tU64)1UL<<15UL)
/*! @brief It's the mask to extract the bit 16 */
#define BIT64_16   ((tU64)1UL<<16UL)
/*! @brief It's the mask to extract the bit 17 */
#define BIT64_17   ((tU64)1UL<<17UL)
/*! @brief It's the mask to extract the bit 18 */
#define BIT64_18   ((tU64)1UL<<18UL)
/*! @brief It's the mask to extract the bit 19 */
#define BIT64_19   ((tU64)1UL<<19UL)
/*! @brief It's the mask to extract the bit 20 */
#define BIT64_20   ((tU64)1UL<<20UL)
/*! @brief It's the mask to extract the bit 21 */
#define BIT64_21   ((tU64)1UL<<21UL)
/*! @brief It's the mask to extract the bit 22 */
#define BIT64_22   ((tU64)1UL<<22UL)
/*! @brief It's the mask to extract the bit 23 */
#define BIT64_23   ((tU64)1UL<<23UL)
/*! @brief It's the mask to extract the bit 24 */
#define BIT64_24   ((tU64)1UL<<24UL)
/*! @brief It's the mask to extract the bit 25 */
#define BIT64_25   ((tU64)1UL<<25UL)
/*! @brief It's the mask to extract the bit 26 */
#define BIT64_26   ((tU64)1UL<<26UL)
/*! @brief It's the mask to extract the bit 27 */
#define BIT64_27   ((tU64)1UL<<27UL)
/*! @brief It's the mask to extract the bit 28 */
#define BIT64_28   ((tU64)1UL<<28UL)
/*! @brief It's the mask to extract the bit 39 */
#define BIT64_29   ((tU64)1UL<<29UL)
/*! @brief It's the mask to extract the bit 30 */
#define BIT64_30   ((tU64)1UL<<30UL)
/*! @brief It's the mask to extract the bit 31 */
#define BIT64_31   ((tU64)1UL<<31UL)
/*! @brief It's the mask to extract the bit 32 */
#define BIT64_32   ((tU64)1UL<<32UL)
/*! @brief It's the mask to extract the bit 33 */
#define BIT64_33   ((tU64)1UL<<33UL)
/*! @brief It's the mask to extract the bit 34 */
#define BIT64_34   ((tU64)1UL<<34UL)
/*! @brief It's the mask to extract the bit 35 */
#define BIT64_35   ((tU64)1UL<<35UL)
/*! @brief It's the mask to extract the bit 36 */
#define BIT64_36   ((tU64)1UL<<36UL)
/*! @brief It's the mask to extract the bit 37 */
#define BIT64_37   ((tU64)1UL<<37UL)
/*! @brief It's the mask to extract the bit 38 */
#define BIT64_38   ((tU64)1UL<<38UL)
/*! @brief It's the mask to extract the bit 39 */
#define BIT64_39   ((tU64)1UL<<39UL)
/*! @brief It's the mask to extract the bit 40 */
#define BIT64_40   ((tU64)1UL<<40UL)
/*! @brief It's the mask to extract the bit 41 */
#define BIT64_41   ((tU64)1UL<<41UL)
/*! @brief It's the mask to extract the bit 42 */
#define BIT64_42   ((tU64)1UL<<42UL)
/*! @brief It's the mask to extract the bit 43 */
#define BIT64_43   ((tU64)1UL<<43UL)
/*! @brief It's the mask to extract the bit 44 */
#define BIT64_44   ((tU64)1UL<<44UL)
/*! @brief It's the mask to extract the bit 45 */
#define BIT64_45   ((tU64)1UL<<45UL)
/*! @brief It's the mask to extract the bit 46 */
#define BIT64_46   ((tU64)1UL<<46UL)
/*! @brief It's the mask to extract the bit 47 */
#define BIT64_47   ((tU64)1UL<<47UL)
/*! @brief It's the mask to extract the bit 48 */
#define BIT64_48   ((tU64)1UL<<48UL)
/*! @brief It's the mask to extract the bit 49 */
#define BIT64_49   ((tU64)1UL<<49UL)
/*! @brief It's the mask to extract the bit 50 */
#define BIT64_50   ((tU64)1UL<<50UL)
/*! @brief It's the mask to extract the bit 51 */
#define BIT64_51   ((tU64)1UL<<51UL)
/*! @brief It's the mask to extract the bit 52 */
#define BIT64_52   ((tU64)1UL<<52UL)
/*! @brief It's the mask to extract the bit 53 */
#define BIT64_53   ((tU64)1UL<<53UL)
/*! @brief It's the mask to extract the bit 54 */
#define BIT64_54   ((tU64)1UL<<54UL)
/*! @brief It's the mask to extract the bit 55 */
#define BIT64_55   ((tU64)1UL<<55UL)
/*! @brief It's the mask to extract the bit 56 */
#define BIT64_56   ((tU64)1UL<<56UL)
/*! @brief It's the mask to extract the bit 57 */
#define BIT64_57   ((tU64)1UL<<57UL)
/*! @brief It's the mask to extract the bit 58 */
#define BIT64_58   ((tU64)1UL<<58UL)
/*! @brief It's the mask to extract the bit 59 */
#define BIT64_59   ((tU64)1UL<<59UL)
/*! @brief It's the mask to extract the bit 60 */
#define BIT64_60   ((tU64)1UL<<60UL)
/*! @brief It's the mask to extract the bit 61 */
#define BIT64_61   ((tU64)1UL<<61UL)
/*! @brief It's the mask to extract the bit 62 */
#define BIT64_62   ((tU64)1UL<<62UL)
/*! @brief It's the mask to extract the bit 63 */
#define BIT64_63   ((tU64)1UL<<63UL)

/*! @brief It's the mask to extract the bit x */
#define BIT64_x(x) (((tU64)1UL)<<(x))

// To be checked
#define GET8_BIT_FIELD(value,start,end)  (((value) >> (start)) << ((start) + (8  - (end))) >> (8  - (end)))
#define GET16_BIT_FIELD(value,start,end) (((value) >> (start)) << ((start) + (16 - (end))) >> (16 - (end)))
#define GET32_BIT_FIELD(value,start,end) (((value) >> (start)) << ((start) + (32 - (end))) >> (32 - (end)))

//----------------------------------------------------------------------
// 8 bits operations
//----------------------------------------------------------------------
/*! @brief This macro reads a 8 bits value */
#define READ8(reg)                      (*(tVPU8)(reg))
/*! @brief This macro writes a 8 bits value */
#define WRITE8(reg,val)                 (*(tVPU8)(reg) = (tU8)(val))
/*! @brief This macro sets the specified bits on a 8 bits value */
#define SET8_BIT(reg,bits)              (*(tVPU8)(reg) |= (tU8)(bits))
/*! @brief This macro clears the specified bits on a 8 bits value */
#define CLEAR8_BIT(reg,bits)            (*(tVPU8)(reg) &= ~(tU8)(bits))

//----------------------------------------------------------------------
// 16 bits operations
//----------------------------------------------------------------------
/*! @brief This macro reads a 16 bits value */
#define READ16(reg)                     (*(tVPU16)(reg))
/*! @brief This macro writes a 16 bits value */
#define WRITE16(reg,val)                (*(tVPU16)(reg) = (tU16)(val))
/*! @brief This macro sets the specified bits on a 16 bits value */
#define SET16_BIT(reg,bits)             (*(tVPU16)(reg) |= (tU16)(bits))
/*! @brief This macro clears the specified bits on a 16 bits value */
#define CLEAR16_BIT(reg,bits)           (*(tVPU16)(reg) &= ~(tU16)(bits))

//----------------------------------------------------------------------
// 32 bits operations
//----------------------------------------------------------------------
/*! @brief This macro reads a 32 bits value */
#define READ32(reg)                     (*(tVPU32)(reg))
/*! @brief This macro writes a 32 bits value */
#define WRITE32(reg,val)                (*(tVPU32)(reg) = (tU32)(val))
/*! @brief This macro sets the specified bits on a 32 bits value */
#define SET32_BIT(reg,bits)             (*(tVPU32)(reg) |= (tU32)(bits))
/*! @brief This macro clears the specified bits on a 32 bits value */
#define CLEAR32_BIT(reg,bits)           (*(tVPU32)(reg) &= ~(tU32)(bits))

// Description:
// Generic constant for accessing all bits in a 32 bit word
#define apBITS_ALL 0xFFFFFFFFU

// The larger of __x and __y
/*! @brief It returns the MAXIMUM value between __x and __y */
#define MAX(__x,__y) ((__x)>(__y)?(__x):(__y))

// The smaller of __x and __y
/*! @brief It returns the MINIMUM value between __x and __y */
#define MIN(__x,__y) ((__x)<(__y)?(__x):(__y))

// Description:
// Functions for converting a little-endian word of byte or halfword data to
// correct endian format.
//
// Implementation:
// These are required since the FIFO order for the Primecells will not be reversed
// to match a big endian system architecture
//
// These require the __BIG_ENDIAN constant to be defined for a big-endian system
// (set automatically by ADS)
//
// The parameter is a word variable to be byte- or halfword- reversed.
// The correct-endian data will be placed in the same variable
#if ( defined(__linux__) && (__BYTE_ORDER != __BIG_ENDIAN) ) || ( (! defined(__linux__)) && (! defined(__BIG_ENDIAN)) )
#define apBYTE_STREAM_ENDIAN_ADJUST(_w)
#define apHWD_STREAM_ENDIAN_ADJUST(_w)

#else
#ifndef __thumb
// ARM code version
#define apBYTE_STREAM_ENDIAN_ADJUST(_w) {tU32 temp;\
                    const tU32 mask = 0x00FF00FF, shift = 8;\
                    __asm{MOV temp,_w,ROR (32-shift);AND temp,temp,mask;\
                    MOV _w,_w,ROR shift;AND _w,_w,mask;ORR _w,_w,temp;}}
#define apHWD_STREAM_ENDIAN_ADJUST(_w) {const tU32 shift = 16; __asm { MOV _w,_w,ROR shift);}

#else
// Thumb code version
#define apBYTE_STREAM_ENDIAN_ADJUST(_w) {tU32 temp;\
                    const tU32 mask = 0x00FF00FF, shift = 8;\
                    __asm{MOV temp,_w;ROR temp,(32-shift);AND temp,mask;\
                    ROR _w,shift;AND _w,mask;ORR _w,temp;}}
#define apHWD_STREAM_ENDIAN_ADJUST(_w)  {const tU32 shift = 16; __asm { ROR _w,shift);}
#endif
#endif

//------------------- Standard data access set ------------------------------
// This set of macros accesses data using a width/shift parameter.
// This assumes that constants bsXXX and bwXXX are defined,
//   where XXX is the value specified in the parameter.
// + bwXXX is the number of bits to be accessed.
// + bsXXX is the offset of the lowest bit.

// Description:
// Build a mask for the specified bits
//
// Implementation:
// __bws - a width/shift pair.
//
// Returns  a mask with the bits to be addressed set and all others cleared
//

/*lint -e(9023,9024) Multiple use of '#/##' operators [MISRA 2012 Rule 20.10, advisory]*/
#define apBIT_MASK(__bws) ((tU32)(((bw ## __bws)==32)?0xFFFFFFFFU:((1U << (bw ## __bws)) - 1)) << (bs ## __bws))

// Description:
// Clear the specified bits
//
// Implementation:
// __datum - the word of data to be modified
// __bws - a width/shift pair.
#define apBIT_CLEAR(__datum, __bws) ((__datum) = ((__datum) & ~((tU32) apBIT_MASK(__bws))))

// Description:
// Access the specified bits from a word of data at their original offset
//
// Implementation:
// __datum - the word of data to be accessed
// __bws - a width/shift pair.
//
// Returns  The relevant bits masked from the data word, still at their original offset
#define apBIT_GET_UNSHIFTED(__datum, __bws) (((__datum) & ((tU32) apBIT_MASK(__bws))))

// Description:
// Access the specified bits from a word of data as an integer value
//
// Implementation:
// __datum - the word of data to be accessed
// __bws - a width/shift pair.
//
// Returns  The relevant bits masked from the data word shifted to bit zero
#define apBIT_GET(__datum, __bws) ((tU32)(((__datum) & ((tU32) apBIT_MASK(__bws))) >> (bs ## __bws)))

// Description:
// Place the specified value into the specified bits of a word of data
//
// Implementation:
// __datum - the word of data to be accessed
// __bws - a width/shift pair.
// __val - the data value to be shifted into the specified bits.
#define apBIT_SET(__datum, __bws, __val) ((__datum) = ((tU32) (__datum) & (tU32)~(apBIT_MASK(__bws))) | \
                                        ((tU32) ((tU32)(__val) << (tU32)(bs ## __bws)) & (apBIT_MASK(__bws))))

// Description:
// Place the specified value into the specified bits of a word of data
// without reading first - for sensitive interrupt type registers
//
// Implementation:
// __datum - the word of data to be accessed
// __bws - a width/shift pair.
// __val - the data value to be shifted into the specified bits.
#define apBIT_SET_NOREAD(__datum, __bws, __val) (tU32) ((__datum) = (((__val) << (bs ## __bws)) & ((tU32) apBIT_MASK(__bws))))

// Description:
// Shift the specified value into the desired bits
//
// Implementation:
// __bws - a width/shift pair.
// __val - the data value to be shifted into the specified bits.
//
// Returns  The value shifted to the specified offset
#define apBIT_BUILD(__bws, __val) ((tU32)(((tU32)(__val) << (tU32)(bs ## __bws)) & ((tU32) apBIT_MASK(__bws))))

//------------------ 'Field' data access set ---------------------------
// This set of macros accesses data using separate width and
// shift parameters.
//
// Description:
// Equivalent to apBIT_MASK, but using separate width and
// shift parameters.
#define apBIT_MASK_FIELD(__bw, __bs) ((tU32)((((__bw)==32)?0xFFFFFFFFU:(tU32)((tU32)1U << (__bw)) - 1)) << (__bs))

// Description:
// Equivalent to apBIT_CLEAR, but using separate width and
// shift parameters.
#define apBIT_CLEAR_FIELD(__datum, __bw, __bs) ((__datum) = ((__datum) & ~((tU32) apBIT_MASK_FIELD(__bw,__bs))))

// Description:
// Equivalent to apBIT_GET, but using separate width and
// shift parameters.
#define apBIT_GET_FIELD(__datum, __bw, __bs) ((tU32)(((__datum) & ((tU32) apBIT_MASK_FIELD(__bw,__bs))) >> (__bs)))

// Description:
// Equivalent to apBIT_SET, but using separate width and
// shift parameters.
#define apBIT_SET_FIELD(__datum, __bw, __bs, __val) ((__datum) = ((__datum) & ~((tU32) apBIT_MASK_FIELD(__bw,__bs))) | \
                                                    ((tU32) ((tU32)(__val) << (__bs)) & ((tU32) apBIT_MASK_FIELD(__bw,__bs))))
// Description:
// Equivalent to apBIT_SET_NOREAD, but using separate width and
// shift parameters.
#define apBIT_SET_FIELD_NOREAD(__datum, __bw,__bs, __val) (tU32) ((__datum) = (((__val) << (__bs)) & ((tU32) apBIT_MASK_FIELD(__bw,__bs))))

// Description:
// Equivalent to apBIT_BUILD, but using separate width and
// shift parameters.
#define apBIT_BUILD_FIELD(__bw, __bs, __val) ((tU32)(((__val) << (__bs)) & ((tU32) apBIT_MASK_FIELD(__bw,__bs))))

// Description:
// Equivalent to apBIT_GET_UNSHIFTED, but using separate width and
// shift parameters.
#define apBIT_GET_FIELD_UNSHIFTED(__datum, __bw, __bs) ((tU32)((__datum) & ((tU32) apBIT_MASK_FIELD(__bw,__bs))))

// Description:
// Equivalent to apBIT_SET_FIELD, but with BYTE width parameters.
#define apBYTE_SET_FIELD(__datum, __bw, __bs, __val) ((__datum) = (tS8) (((__datum) & ~(apBIT_MASK_FIELD(__bw,__bs))) | (((__val) << (__bs)) & (apBIT_MASK_FIELD(__bw,__bs)))))

// Description:
// NOP macro
#if defined( __ARMCC_VERSION)
#define MCR_NOP() __nop()
#endif
#if defined( __GNUC__)
#if defined( __thumb__)
#define MCR_NOP() { __asm volatile ( ".thumb\nnop;"); }
#else
#define MCR_NOP() { __asm volatile ( ".arm\nnop;"); }
#endif
#endif

// Description:
// This macros must be used before and after any inline assembly code for GCC (enclosed in quoteS)
// to allow it to be executed if compiled either in ARM or THUMB mode
#if defined( __GNUC__)
#if defined( __thumb__)
#define INLINE_ASM_ENTER \
    "\
    .balign 4\n                   ;\
    mov   r3, pc                  ;\
    bx    r3                      ;\
    .arm                          ;\
    "
#define INLINE_ASM_EXIT \
    "\
    .arm\n                            ;\
    add   r3, pc, #1                  ;\
    bx    r3                          ;\
    .thumb                            ;\
    "
#else
#define INLINE_ASM_ENTER
#define INLINE_ASM_EXIT
#endif
#endif

// Description:
// This macros are used to tag specific sections for GNSS application
#if (__ARMCC_VERSION >= 300586) || defined( __GNUC__)
#define GENERIC_CODE_ISR      __attribute__((section ("GENERIC_CODE_ISR_REGION")))
#define GENERIC_DATA_ISR      __attribute__((section ("GENERIC_DATA_ISR_REGION")))
#define GENERIC_NORELOC       __attribute__((section ("GENERIC_NORELOC_REGION")))
#define GNSS_FAST             __attribute__((section ("GNSS_FAST_REGION")))
#define STAGPS_FAST           __attribute__((section ("STAGPS_FAST_REGION")))
#define BIN_IMAGE_FAST        __attribute__((section ("BIN_IMAGE_FAST_REGION")))
#define SRAM2_DATA            __attribute__((section("SRAM2_AREA")))
#define SRAM_DATA             __attribute__((section("SRAM_AREA")))
#if defined(__GNUC__)
#define SRAM_STDBY_DATA       __attribute__ ((section("SRAM_STDBY_AREA")))
#else
#define SRAM_STDBY_DATA       __attribute__ ((zero_init))
#endif
#else
#define GENERIC_CODE_ISR
#define GENERIC_DATA_ISR
#define GENERIC_NORELOC
#define GNSS_FAST
#define STAGPS_FAST
#define BIN_IMAGE_FAST
#define SRAM2_DATA
#define SRAM_DATA
#define SRAM_STDBY_DATA
#endif

#ifdef __cplusplus
}
#endif


#define RCIF_DATA_ALIGN


#endif  /* MACROS_H */

/* End of file */
