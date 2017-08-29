//!
//!   \file       datum.h
//!   \brief      <i><b> datum module, header file</b></i>
//!   \author     Fulvio Boggia
//!   \authors    Many
//!   \version    1.0
//!   \date       2010.09.01
//!   \bug        Unknown
//!   \warning    None
//!   \addtogroup modules
//!   \{
//!

#ifndef DATUM_H
#define DATUM_H

/*****************************************************************************
   includes
*****************************************************************************/

#include "gnss_api.h"
#include "gnss_const.h"

/*****************************************************************************
   defines and macros (scope: module-local)
*****************************************************************************/

/*****************************************************************************
   typedefs and structures (scope: module-local)
*****************************************************************************/

typedef enum
{
  DATUM_ELLIPSOID_ID_AA,
  DATUM_ELLIPSOID_ID_AN,
  DATUM_ELLIPSOID_ID_BR,
  DATUM_ELLIPSOID_ID_BN,
  DATUM_ELLIPSOID_ID_CC,
  DATUM_ELLIPSOID_ID_CD,
  DATUM_ELLIPSOID_ID_EB,
  DATUM_ELLIPSOID_ID_EA,
  DATUM_ELLIPSOID_ID_EC,
  DATUM_ELLIPSOID_ID_EF,
  DATUM_ELLIPSOID_ID_EE,
  DATUM_ELLIPSOID_ID_ED,
  DATUM_ELLIPSOID_ID_RF,
  DATUM_ELLIPSOID_ID_HE,
  DATUM_ELLIPSOID_ID_HO,
  DATUM_ELLIPSOID_ID_ID,
  DATUM_ELLIPSOID_ID_IN,
  DATUM_ELLIPSOID_ID_KA,
  DATUM_ELLIPSOID_ID_AM,
  DATUM_ELLIPSOID_ID_FA,
  DATUM_ELLIPSOID_ID_SA,
  DATUM_ELLIPSOID_ID_WD,
  DATUM_ELLIPSOID_ID_PZ,
  DATUM_ELLIPSOID_ID_WE
} datum_ellipsoid_id_t;

typedef struct ellipsoid_par_s
{
  tDouble delta_a;
  tDouble delta_f;
} datum_ellipsoid_par_t;

#define _WGS84 228
#define CODE_USER (_WGS84+1)
#define MAX_DATUM_ID 255
typedef enum
{
  /*AFRICA*/
  DATUM_CODE_ADI_M = 0,
  DATUM_CODE_ADI_E = 1,
  DATUM_CODE_ADI_F = 2,
  DATUM_CODE_ADI_A = 3,
  DATUM_CODE_ADI_C = 4,
  DATUM_CODE_ADI_D = 5,
  DATUM_CODE_ADI_B = 6,
  DATUM_CODE_AFG   = 7,
  DATUM_CODE_ARF_M = 8,
  DATUM_CODE_ARF_A = 9,
  DATUM_CODE_ARF_H = 10,
  DATUM_CODE_ARF_B = 11,
  DATUM_CODE_ARF_C = 12,
  DATUM_CODE_ARF_D = 13,
  DATUM_CODE_ARF_E = 14,
  DATUM_CODE_ARF_F = 15,
  DATUM_CODE_ARF_G = 16,
  DATUM_CODE_ARS_M = 17,
  DATUM_CODE_ARS_A = 18,
  DATUM_CODE_ARS_B = 19,
  DATUM_CODE_PHA   = 20,
  DATUM_CODE_BID   = 21,
  DATUM_CODE_CAP   = 22,
  DATUM_CODE_CGE   = 23,
  DATUM_CODE_DAL   = 24,
  /*EUR_F, already defined */
  /*EUR_T,already defined */
  DATUM_CODE_LEH   = 25,
  DATUM_CODE_LIB   = 26,
  DATUM_CODE_MAS   = 27,
  DATUM_CODE_MER   = 28,
  DATUM_CODE_MIN_A = 29,
  DATUM_CODE_MIN_B = 30,
  DATUM_CODE_MPO   = 31,
  DATUM_CODE_NSD   = 32,
  DATUM_CODE_OEG   = 33,
  DATUM_CODE_PTB   = 34,
  DATUM_CODE_PTN   = 35,
  DATUM_CODE_SCK   = 36,
  DATUM_CODE_SRL   = 37,
  DATUM_CODE_VOR   = 38,
  /*ASIA*/
  DATUM_CODE_AIN_A = 39,
  DATUM_CODE_AIN_B = 40,
  DATUM_CODE_BAT   = 41,
  /*EUR_H,already defined */
  DATUM_CODE_HKD   = 42,
  DATUM_CODE_HTN   = 43,
  DATUM_CODE_IND_B = 44,
  DATUM_CODE_IND_I = 45,
  DATUM_CODE_INF   = 46,
  DATUM_CODE_ING_A = 47,
  DATUM_CODE_ING_B = 48,
  DATUM_CODE_INH_A = 49,
  DATUM_CODE_INH_A1= 50,
  DATUM_CODE_IDN   = 51,
  DATUM_CODE_KAN   = 52,
  DATUM_CODE_KEA   = 53,
  DATUM_CODE_KGS   = 54,
  DATUM_CODE_NAH_A = 55,
  DATUM_CODE_NAH_B = 56,
  DATUM_CODE_NAH_C = 57,
  DATUM_CODE_FAH   = 58,
  DATUM_CODE_QAT   = 59,
  DATUM_CODE_SOA   = 60,
  DATUM_CODE_TIL   = 61,
  DATUM_CODE_TOY_M = 62,
  DATUM_CODE_TOY_A = 63,
  DATUM_CODE_TOY_C = 64,
  DATUM_CODE_TOY_B = 65,
  DATUM_CODE_TOY_B1= 66,
  /*AUSTRALIA*/
  DATUM_CODE_AUA   = 67,
  DATUM_CODE_AUG   = 68,
  /*EUROPA*/
  DATUM_CODE_EST   = 69,
  DATUM_CODE_EUR_M = 70,
  DATUM_CODE_EUR_A = 71,
  DATUM_CODE_EUR_E = 72,
  DATUM_CODE_EUR_F = 73,
  DATUM_CODE_EUR_G = 74,
  DATUM_CODE_EUR_K = 75,
  DATUM_CODE_EUR_B = 76,
  DATUM_CODE_EUR_H = 77,
  DATUM_CODE_EUR_I = 78,
  DATUM_CODE_EUR_J = 79,
  DATUM_CODE_EUR_L = 80,
  DATUM_CODE_EUR_C = 81,
  DATUM_CODE_EUR_D = 82,
  DATUM_CODE_EUR_T = 83,
  DATUM_CODE_EUS   = 84,
  DATUM_CODE_HJO   = 85,
  DATUM_CODE_IRL   = 86,
  DATUM_CODE_OGB_M = 87,
  DATUM_CODE_OGB_A = 88,
  DATUM_CODE_OGB_B = 89,
  DATUM_CODE_OGB_C = 90,
  DATUM_CODE_OGB_D = 91,
  DATUM_CODE_MOD   = 92,
  DATUM_CODE_SPK_A = 93,
  DATUM_CODE_SPK_B = 94,
  DATUM_CODE_SPK_C = 95,
  DATUM_CODE_SPK_D = 96,
  DATUM_CODE_SPK_E = 97,
  DATUM_CODE_SPK_F = 98,
  DATUM_CODE_SPK_G = 99,
  DATUM_CODE_CCD   = 100,
  /*NORTH AMERICA*/
  DATUM_CODE_CAC   = 101,
  DATUM_CODE_NAS_C = 102,
  DATUM_CODE_NAS_B = 103,
  DATUM_CODE_NAS_A = 104,
  DATUM_CODE_NAS_D = 105,
  DATUM_CODE_NAS_V = 106,
  DATUM_CODE_NAS_W = 107,
  DATUM_CODE_NAS_Q = 108,
  DATUM_CODE_NAS_R = 109,
  DATUM_CODE_NAS_E = 110,
  DATUM_CODE_NAS_F = 111,
  DATUM_CODE_NAS_G = 112,
  DATUM_CODE_NAS_H = 113,
  DATUM_CODE_NAS_I = 114,
  DATUM_CODE_NAS_J = 115,
  DATUM_CODE_NAS_O = 116,
  DATUM_CODE_NAS_P = 117,
  DATUM_CODE_NAS_N = 118,
  DATUM_CODE_NAS_T = 119,
  DATUM_CODE_NAS_U = 120,
  DATUM_CODE_NAS_L = 121,
  DATUM_CODE_NAR_A = 122,
  DATUM_CODE_NAR_E = 123,
  DATUM_CODE_NAR_B = 124,
  DATUM_CODE_NAR_C = 125,
  DATUM_CODE_NAR_H = 126,
  DATUM_CODE_NAR_D = 127,
  /*SOUTH AMERICA*/
  DATUM_CODE_BDO   = 128,
  DATUM_CODE_CAI   = 129,
  DATUM_CODE_CHU   = 130,
  DATUM_CODE_COA   = 131,
  DATUM_CODE_PRP_M = 132,
  DATUM_CODE_PRP_A = 133,
  DATUM_CODE_PRP_B = 134,
  DATUM_CODE_PRP_C = 135,
  DATUM_CODE_PRP_D = 136,
  DATUM_CODE_PRP_E = 137,
  DATUM_CODE_PRP_F = 138,
  DATUM_CODE_PRP_G = 139,
  DATUM_CODE_PRP_H = 140,
  DATUM_CODE_HIT   = 141,
  DATUM_CODE_SAN_M = 142,
  DATUM_CODE_SAN_A = 143,
  DATUM_CODE_SAN_B = 144,
  DATUM_CODE_SAN_C = 145,
  DATUM_CODE_SAN_D = 146,
  DATUM_CODE_SAN_E = 147,
  DATUM_CODE_SAN_F = 148,
  DATUM_CODE_SAN_J = 149,
  DATUM_CODE_SAN_G = 150,
  DATUM_CODE_SAN_H = 151,
  DATUM_CODE_SAN_I = 152,
  DATUM_CODE_SAN_K = 153,
  DATUM_CODE_SAN_L = 154,
  DATUM_CODE_SIR   = 155,
  DATUM_CODE_ZAN   = 156,
  /*ATLANTIC OCEAN*/
  DATUM_CODE_AIA   = 157,
  DATUM_CODE_ASC   = 158,
  DATUM_CODE_SHB   = 159,
  DATUM_CODE_BER   = 160,
  /*CAC, already defined in NorthAmerica*/
  DATUM_CODE_DID   = 161,
  DATUM_CODE_FOT   = 162,
  DATUM_CODE_GRA   = 163,
  /*HJO, already  defined in Europe*/
  DATUM_CODE_ISG   = 164,
  DATUM_CODE_LCF   = 165,
  DATUM_CODE_ASM   = 166,
  DATUM_CODE_NAP   = 167,
  DATUM_CODE_FLO   = 168,
  DATUM_CODE_PLN   = 169,
  DATUM_CODE_POS   = 170,
  DATUM_CODE_PUR   = 171,
  DATUM_CODE_QUO   = 172,
  DATUM_CODE_SAO   = 173,
  DATUM_CODE_SAP   = 174,
  DATUM_CODE_SGM   = 175,
  DATUM_CODE_TDC   = 176,
  /*INDIAN OCEAN*/
  DATUM_CODE_ANO   = 177,
  DATUM_CODE_GAA   = 178,
  DATUM_CODE_IST   = 179,
  DATUM_CODE_KEG   = 180,
  DATUM_CODE_MIK   = 181,
  DATUM_CODE_REU   = 182,
  /*PACIFIC OCEAN*/
  DATUM_CODE_AMA   = 183,
  DATUM_CODE_ATF   = 184,
  DATUM_CODE_TRN   = 185,
  DATUM_CODE_ASQ   = 186,
  DATUM_CODE_IBE   = 187,
  DATUM_CODE_CAO   = 188,
  DATUM_CODE_CHI   = 189,
  DATUM_CODE_GIZ   = 190,
  DATUM_CODE_EAS   = 191,
  DATUM_CODE_GEO   = 192,
  DATUM_CODE_GUA   = 193,
  DATUM_CODE_DOB   = 194,
  /*IDN, already defined in Europe*/
  DATUM_CODE_JOH   = 195,
  DATUM_CODE_KUS   = 196,
  DATUM_CODE_LUZ_A = 197,
  DATUM_CODE_LUZ_B = 198,
  DATUM_CODE_MID_A = 199,
  DATUM_CODE_MID_B = 200,
  DATUM_CODE_OHA_M = 201,
  DATUM_CODE_OHA_A = 202,
  DATUM_CODE_OHA_B = 203,
  DATUM_CODE_OHA_C = 204,
  DATUM_CODE_OHA_D = 205,
  DATUM_CODE_OHI_M = 206,
  DATUM_CODE_OHI_A = 207,
  DATUM_CODE_OHI_B = 208,
  DATUM_CODE_OHI_C = 209,
  DATUM_CODE_OHI_D = 210,
  DATUM_CODE_PIT   = 211,
  DATUM_CODE_SAE   = 212,
  DATUM_CODE_MVS   = 213,
  DATUM_CODE_ENW   = 214,
  DATUM_CODE_WAK   = 215,
  DATUM_CODE_BUR   = 216,
  DATUM_CODE_CAZ   = 217,
  DATUM_CODE_EUR_S = 218,
  DATUM_CODE_GSE   = 219,
  DATUM_CODE_HEN   = 220,
  DATUM_CODE_HER   = 221,
  DATUM_CODE_IND_P = 222,
  DATUM_CODE_PUK   = 223,
  DATUM_CODE_TAN   = 224,
  DATUM_CODE_VOI   = 225,
  DATUM_CODE_YAC   = 226,
  /*RUSSIA AREA - GLONASS REFERENCE*/
  DATUM_CODE_PZ90_2= 227,
  //insert new datum here - you can modify number of WGS84 (please change _WGS84 value)  and USER_D
  //Warning: Geodetic datum number cannot exceed 254 due to current implementation and SW config "DATUM_ID"
  DATUM_CODE_WGS84 = _WGS84,
  DATUM_CODE_USER_D = CODE_USER,
  DATUM_ROTATED_ID  = 252,
  DATUM_CODE_PZ90   = 253,
  DATUM_CODE_PZ90_11= 254,
  DATUM_DEFAULT     = MAX_DATUM_ID
 } datum_code_t;

typedef struct datum_trasformation_par_s
{
  datum_ellipsoid_id_t  reference_ellipsoid;
  tShort                 delta_x;
  tShort                 delta_y;
  tShort                 delta_z;
} datum_trasformation_par_t;

typedef struct datum_rotated_trasformation_par_s
{
  datum_ellipsoid_id_t  reference_ellipsoid;
  tInt                 delta_x;
  tInt                 delta_y;
  tInt                 delta_z;
  tInt                 d;
  tInt                 r1;
  tInt                 r2;
  tInt                 r3;
} datum_rotated_trasformation_par_t;

typedef struct local_to_WGS84_param_s
{
  tDouble delta_x;
  tDouble delta_y;
  tDouble delta_z;
  tDouble delta_a;
  tDouble delta_f;
  tDouble d;
  tDouble r1;
  tDouble r2;
  tDouble r3;
} datum_local_to_WGS84_param_t;
/*****************************************************************************
   exported variables
*****************************************************************************/

/*****************************************************************************
   exported function prototypes
*****************************************************************************/

extern  gnss_error_t  datum_init             (tUInt);
extern  gnss_error_t  datum_select           (datum_code_t);
extern  gnss_error_t  datum_set_by_user      (datum_local_to_WGS84_param_t);
extern  datum_code_t  datum_get_current_code (void);
extern  void          datum_apply_ajustment  (position_t*, tDouble*);
extern  void          datum_convert_position_wgs84_to_local_datum(ECEF_pos_t *, position_t *, tDouble *);

#endif  /* DATUM_H */

//!   \}

