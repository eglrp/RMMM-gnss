/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/

/********************************************//**
 * \file fixpoint.h
 * \brief This header provides fixpoint arithmetic.
 ***********************************************/

#ifndef FIXPOINT_H
#define FIXPOINT_H

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   includes
*****************************************************************************/

#include "macros.h"
#include "mathfunc.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

#define FP_PLUSONE32            0x7fffffffU
#define FP_MINUSONE32           0x80000001U
#define FP_PLUS_ONE_HALF        0x3fffffffU
#define FP_PLUS_ONE_QUARTER     0x1fffffffU

#define FP_SCALE32              (((fp_u32_t)1U<<31U) - 1U)
#define FP_SCALE64              (((fp_u64_t)1L<<62L) - 1L)

#define FP_ISCALE(x)            ((tDouble)((tULong)1<<(x)))
#define FP_LSCALE(x)            ((tDouble)((tULongLong)1<<(x)))

#define FP_DTOFP32_DS( n, s)    ((fp_s32_t)(((tDouble)(n) / (s)) * FP_SCALE32))
#define FP_FP32TOD_DS( n, s)    (((tDouble)(n) * (s)) / FP_SCALE32)
#define FP_DTOFP64_DS( n, s)    ((fp_s64_t)(((tDouble)(n) / (s)) * FP_SCALE64))
#define FP_FP64TOD_DS( n, s)    (((tDouble)(n) * (s)) / FP_SCALE64)
#define FP_DTOFP32_US( n, s)    ((fp_s32_t)(((tDouble)(n) * (s)) * FP_SCALE32))
#define FP_FP32TOD_US( n, s)    ((tDouble)(n) / (s) / FP_SCALE32)
#define FP_DTOFP64_US( n, s)    ((fp_s64_t)(((tDouble)(n) * (s) * FP_SCALE64))
#define FP_FP64TOD_US( n, s)    ((tDouble)(n) / (s) / FP_SCALE64)

#define FP_DTOFPU32_DS( n, s)    ((fp_u32_t)(((tDouble)(n) / (s)) * (((fp_u64_t)1<<32UL) - 1UL)))
#define FP_FPU32TOD_DS( n, s)    (((tDouble)(n) * (s)) / (((fp_u64_t)1<<32UL) - 1UL))
#define FP_SCALE16              (((fp_u16_t)1<<15U) - 1U)
#define FP_DTOFP16_DS( n, s)    ((fp_s16_t)(((tDouble)(n) / (s)) * FP_SCALE16))
#define FP_FP16TOD_DS( n, s)    (((tDouble)(n) * (s)) / FP_SCALE16)

#define FP_FP64TO32(x)          ((fp_s32_t)((fp_s64_t)(x) >> 31UL))
#define FP_FP32TO64(x)          ((fp_s64_t)((fp_s64_t)(x) << 31UL))

#define FP_ABS32(x)             MCR_ABS(x)

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/
typedef tU16        fp_u16_t;
typedef tS16        fp_s16_t;
typedef tU32        fp_u32_t;
typedef tS32        fp_s32_t;
typedef tULongLong  fp_u64_t;
typedef tLongLong   fp_s64_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

__inline static fp_s32_t fp_add32( fp_s32_t a, fp_s32_t b)
{
  return a + b;
}
__inline static fp_s32_t fp_sub32( fp_s32_t a, fp_s32_t b)
{
  return a - b;
}
__inline static fp_s64_t fp_add64( fp_s64_t a, fp_s64_t b)
{
  return a + b;
}
__inline static fp_s64_t fp_sub64( fp_s64_t a, fp_s64_t b)
{
  return a - b;
}
__inline static fp_s64_t fp_smul64( const fp_s32_t num1, const fp_s32_t num2)
{
  return ((fp_s64_t)num1 * num2);
}
__inline static fp_u64_t fp_umul64( const fp_u32_t num1, const fp_u32_t num2)
{
  return ((fp_u64_t)num1 * num2);
}
__inline static fp_u32_t fp_sqrt64( fp_u64_t num)
{
  return sqrt_long( num);
}

extern fp_u32_t     fp_umul32               ( const fp_u32_t a, const fp_u32_t b);
extern fp_s32_t     fp_smul32               ( const fp_s32_t a, const fp_s32_t b);
extern tInt         fp_sdiv32               ( const tInt, const tInt);
extern fp_u64_t     fp_udiv64               ( fp_u64_t, fp_u64_t);
extern fp_u32_t     fp_udiv64to32           ( fp_u32_t, fp_u64_t);
extern tUInt        fp_udiv32               ( const tUInt, const tUInt);
extern fp_s32_t     fp_sdiv32_shift         ( const fp_s32_t, const fp_s32_t, const tInt);
extern fp_s64_t     fp_rem64                ( const fp_s64_t, const fp_s64_t);
extern fp_s64_t     fp_rem64_with_division  ( const fp_s64_t, const fp_s64_t, const tInt);
//extern fp_s64_t     fp_rem64_modulo20       ( const fp_s64_t, tInt *);

#ifdef __cplusplus
}
#endif

#endif /* FIXPOINT_H */
