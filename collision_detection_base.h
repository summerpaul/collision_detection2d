/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-13 11:10:38
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-13 14:00:15
 */

#ifndef __COLLISION_DETECTION_BASE_H__
#define __COLLISION_DETECTION_BASE_H__

#ifdef __cplusplus
extern "C"
{
#endif

    typedef signed char CD_S08;
    typedef unsigned char CD_U08;
    typedef signed short CD_S16;
    typedef unsigned short CD_U16;
    typedef signed int CD_S32;
    typedef unsigned int CD_U32;
    typedef float CD_F32;
    typedef double CD_F64;
    typedef void CD_VOID;
    typedef int CD_RET;

#if defined(_WIN32) || defined(_WIN64)
    typedef signed __int64 CD_S64;
    typedef unsigned __int64 CD_U64;
#else
typedef signed long long CD_S64;
typedef unsigned long long CD_U64;
#endif

    typedef CD_U08 CD_BOOL;

#define CD_NULL (0)
#define CD_TRUE (1)
#define CD_FALSE (0)
#define CD_EPS (0.0000001f)
#define CD_EPS_F (1e-07f)
#define CD_EPS_D (1e-15)
#define CD_RET_OK (0)

#define CD_MIN_U08 (0)
#define CD_MAX_U08 (0xFF)
#define CD_MIN_U16 (0)
#define CD_MAX_U16 (0xFFFF)
#define CD_MIN_U32 (0)
#define CD_MAX_U32 (0xFFFFFFFF)
#define CD_MIN_S08 (-128)
#define CD_MAX_S08 (127)
#define CD_MIN_S16 (-32768)
#define CD_MAX_S16 (32767)
#define CD_MIN_S32 (-2147483647 - 1)
#define CD_MAX_S32 (2147483647)

#if defined(_WIN32) || defined(_WIN64)
#define CD_MAX_S64 (9223372036854775807i64)
#define CD_MIN_S64 (-9223372036854775807i64 - 1)
#else
#define CD_MAX_S64 (9223372036854775807LL)
#define CD_MIN_S64 (-9223372036854775807LL - 1)
#endif

#define CD_MINABS_F (1.175494351e-38f)
#define CD_MAXABS_F (3.402823466e+38f)
#define CD_MINABS_D (2.2250738585072014e-308)
#define CD_MAXABS_D (1.7976931348623158e+308)

// float
#define CD_PI (3.1415926f)    // pi
#define CD_2PI (6.2831853f)   // 2*pi
#define CD_PI2 (1.5707963f)   // pi/2
#define CD_PI4 (0.7853981f)   // pi/4
#define CD_RPI (0.3183099f)   // 1/pi
#define CD_PI180 (0.0174533f) // pi/180
#define CD_SQRT2 (1.4142136f) // sqrt(2)
#define CD_SQRT3 (1.7320508f) // sqrt(3)
#define CD_LN2 (0.6931472f)   // ln(2)
#define CD_LN3 (1.0986123f)   // ln(3)
#define CD_E (2.7182818f)     // e
#define CD_RE (0.3678794f)    // 1/e

// double
#define CD_PI_D (3.141592653589793)         // pi
#define CD_2PI_D (6.283185307179586)        // 2*pi
#define CD_PI2_D (1.570796326794897)        // pi/2
#define CD_PI4_D (0.78539816339744830961)   // pi/4
#define CD_RPI_D (0.3183098861837907)       // 1/pi
#define CD_PI180_D (0.01745329251994329577) // pi/180
#define CD_SQRT2_D (1.41421356237309504880) // sqrt(2)
#define CD_SQRT3_D (1.73205080756887729353) // sqrt(3)
#define CD_LN2_D (0.69314718055994530942)   // ln(2)
#define CD_LN3_D (1.09861228866810969139)   // ln(3)
#define CD_E_D (2.71828182845904523536)     // e
#define CD_RE_D (0.36787944117144232159)    // 1/e

#define CD_ANG2RAD(x) ((x) * 0.017453293f)
#define CD_RAD2ANG(x) ((x) * 57.29578f)

#define CD_ANG2RAD_D(x) ((x) * 0.0174532925199433)
#define CD_RAD2ANG_D(x) ((x) * 57.29577951308233)

#define CD_M2MM(x) ((CD_S32)((x) * 1000))
#define CD_MM2M(x) ((x) * 0.001f)

#define CD_S2MS(x) ((CD_S32)((x) * 1000))
#define CD_MS2S(x) ((x) * 0.001f)

#define CD_RAD2MRAD(x) ((CD_S32)((x) * 1000))
#define CD_MRAD2RAD(x) ((x) * 0.001f)

#define CD_KPH2MPS(x) ((x) * 0.27778f)
#define CD_MPS2KPH(x) ((x) * 3.6f)

#define CD_MUL_THO(x) ((CD_S32)((x) * 1000))
#define CD_DIV_THO(x) ((x) * 0.001f)

#define CD_POW_2_31 (2147483648) /
#define CD_LL_D2I(ll) ((CD_S32)((ll > (180 - CD_EPS_D) ? (180 - CD_EPS_D) : ll) * CD_POW_2_31 / 180))
#define CD_LL_I2D(ll) ((CD_F64)ll * 180 / CD_POW_2_31)

#define CD_ABS(x) (((CD_S32)(x) < 0) ? (-(CD_S32)(x)) : (CD_S32)(x))
#define CD_IABS(x) (((CD_S32)(x) < 0) ? (-(CD_S32)(x)) : (CD_S32)(x))
#define CD_CABS(x) (((CD_S08)(x) < 0) ? (-(CD_S08)(x)) : (CD_S08)(x))
#define CD_SABS(x) (((CD_S16)(x) < 0) ? (-(CD_S16)(x)) : (CD_S16)(x))
#define CD_LABS(x) (((CD_S64)(x) < 0) ? (-(CD_S64)(x)) : (CD_S64)(x))
#define CD_FABS(x) (((CD_F32)(x) < 0) ? (-(CD_F32)(x)) : (CD_F32)(x))
#define CD_DABS(x) (((CD_F64)(x) < 0) ? (-(CD_F64)(x)) : (CD_F64)(x))

#define CD_SQUARE(x) ((x) * (x))

#define CD_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CD_MIN(a, b) (((a) < (b)) ? (a) : (b))

#define CD_CLIP(x, mini, maxi) CD_MIN(CD_MAX(x, mini), maxi)

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_BASE_H__ */
