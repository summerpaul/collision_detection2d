/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-13 11:10:38
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-25 19:59:55
 */

#ifndef __COLLISION_DETECTION_BASE_H__
#define __COLLISION_DETECTION_BASE_H__

#ifdef __cplusplus
extern "C" {
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
#define CD_PI (3.1415926f)  // pi
#define CD_2PI (6.2831853f)  // 2*pi
#define CD_PI2 (1.5707963f)  // pi/2
#define CD_PI4 (0.7853981f)  // pi/4
#define CD_RPI (0.3183099f)  // 1/pi
#define CD_PI180 (0.0174533f)  // pi/180
#define CD_SQRT2 (1.4142136f)  // sqrt(2)
#define CD_SQRT3 (1.7320508f)  // sqrt(3)
#define CD_LN2 (0.6931472f)  // ln(2)
#define CD_LN3 (1.0986123f)  // ln(3)
#define CD_E (2.7182818f)  // e
#define CD_RE (0.3678794f)  // 1/e

// double
#define CD_PI_D (3.141592653589793)  // pi
#define CD_2PI_D (6.283185307179586)  // 2*pi
#define CD_PI2_D (1.570796326794897)  // pi/2
#define CD_PI4_D (0.78539816339744830961)  // pi/4
#define CD_RPI_D (0.3183098861837907)  // 1/pi
#define CD_PI180_D (0.01745329251994329577)  // pi/180
#define CD_SQRT2_D (1.41421356237309504880)  // sqrt(2)
#define CD_SQRT3_D (1.73205080756887729353)  // sqrt(3)
#define CD_LN2_D (0.69314718055994530942)  // ln(2)
#define CD_LN3_D (1.09861228866810969139)  // ln(3)
#define CD_E_D (2.71828182845904523536)  // e
#define CD_RE_D (0.36787944117144232159)  // 1/e

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_BASE_H__ */
