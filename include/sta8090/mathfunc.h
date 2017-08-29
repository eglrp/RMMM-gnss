/*******************************************************************************
 *                            (C) 2009 STMicroelectronics
 *    Reproduction and Communication of this document is strictly prohibited
 *      unless specifically authorized in writing by STMicroelectronics.
 *-----------------------------------------------------------------------------
 *                                  APG / CRM / SA&PD
 *                   Software Development Group - SW platform & HW Specific
 ******************************************************************************/

/********************************************//**
 * \file mathfunc.h
 * \brief This header provides maths API.
 ***********************************************/

#ifndef MATHFUNC_H
#define MATHFUNC_H

#include "typedefs.h"

/*****************************************************************************
   includes
*****************************************************************************/

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef struct math_ellipse_s
{
	tDouble semiaxis_e;
	tDouble semiaxis_n;
	tDouble theta;
} math_ellipse_t;

typedef struct euler_angles_s
{
	tDouble phi;   // roll y vs y
	tDouble theta; // pitch x vs x
	tDouble psi;   // yaw  z vs z
} euler_angles_t;

typedef struct three_d_vector_s
{
	tDouble x;
	tDouble y;
	tDouble z;
} three_d_vector_t;

/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

// Integer math procedures
extern void           parabolic_interp            ( tInt, tInt, tInt, tInt, tInt, tInt, tInt, tInt, tInt *, tInt *);
extern tInt           sign_extend                 ( const tUInt, const tUInt);
extern tInt           sign_extend_1c              ( const tUInt, const tUInt);
extern tUInt          sign_embed_1c               ( const tInt, const tUInt);
extern tInt           abs_int                     ( const tInt);
extern tInt           rem_int                     ( const tInt, const tInt);
extern tInt           pow_int                     ( const tShort, const tU16);
extern tInt           count_bit_ones              ( tInt);
extern tInt           select_bit_with_rank        (tUInt, tInt);

extern tU16           sqrt_int                    ( tUInt);
extern tULong         sqrt_long                   ( tULongLong);
extern tInt           ten_log10_int               ( tInt, tUInt);

// Double math procedures
extern tDouble        rem                         ( const tDouble, const tDouble);
extern tDouble        sqr                         ( const tDouble);
extern tInt           round_int                   ( const tDouble);
extern tDouble        calc_mean                   ( tInt no_samples, tDouble *data);
extern tUInt          parity                      ( tUInt num);
extern tDouble        calc_normal_p               ( const tDouble, const tDouble);
extern tDouble        calc_uniform_p              ( const tDouble, const tDouble);
extern tDouble        calc_uniform_linear_p       ( const tDouble, const tDouble, const tDouble, const tDouble, const tDouble);
extern tDouble        calc_multipath_model_p      ( const tDouble, const tDouble, const tDouble, const tDouble, const tDouble,const tU16, tU8);
extern tDouble        calc_cn0_model_p            ( const tU16);
extern tDouble        calc_standard_model_p       ( const tDouble , const tDouble ,const tDouble , const tDouble );
extern void           IIR_alpha_filter            ( tDouble * output, const tDouble input, const tDouble alpha);
extern void           IIR_hp_filter               ( tDouble * output, const tDouble input, const tDouble prev_input, const tDouble alpha);
extern boolean_t      calc_error_ellipse          ( const tDouble, const tDouble, const tDouble, math_ellipse_t *);
extern void           body_to_nav_axes_transform  ( const euler_angles_t, three_d_vector_t *, const boolean_t);

// CRC procedures
extern tU32           mathfunc_crc32              ( const tU32 , const tU32 *, const tU32);
extern tU16           mathfunc_crc16              ( const tU16 , const tU32 *, const tU32);
extern tUInt          mathfunc_crc24q             ( const tU8 *buff, const tInt len);
#endif /* MATHFUNC_H */
