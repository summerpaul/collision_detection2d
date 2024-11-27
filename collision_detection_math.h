/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-15 11:07:27
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-26 20:33:44
 */

#ifndef __COLLISION_DETECTION_MATH_H__
#define __COLLISION_DETECTION_MATH_H__
#include <math.h>
#include "collision_detection_type.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define CD_ANG2RAD(x) ((x)*0.017453293f)         // 角度转弧度，转完后float型
#define CD_RAD2ANG(x) ((x)*57.29578f)            // 弧度转角度，转完后float型
#define CD_ANG2RAD_D(x) ((x)*0.0174532925199433) // 角度转弧度，转完后double型
#define CD_RAD2ANG_D(x) ((x)*57.29577951308233)  // 弧度转角度，转完后double型
#define CD_M2MM(x) ((CD_S32)((x)*1000))          // 米转毫米，转完后变成整型
#define CD_MM2M(x) ((x)*0.001f)                  // 毫米转米，转完后变成浮点型
#define CD_S2MS(x) ((CD_S32)((x)*1000))          // 秒转毫秒，转完后变成整型
#define CD_MS2S(x) ((x)*0.001f)                  // 毫秒转秒，转完后变成浮点型
#define CD_RAD2MRAD(x) ((CD_S32)((x)*1000))      // 弧度转毫弧度，转完后变成整型
#define CD_MRAD2RAD(x) ((x)*0.001f)              // 毫弧度转弧度，转完后变成浮点型
#define CD_KPH2MPS(x) ((x)*0.27778f)             // 公里每小时转米每秒
#define CD_MPS2KPH(x) ((x)*3.6f)                 // 米每秒转公里每小时

#define CD_MUL_THO(x) ((CD_S32)((x)*1000)) // 放大1000倍，转为整形
#define CD_DIV_THO(x) ((x)*0.001f)         // 缩小1000倍，转为浮点

#define CD_POW_2_31 (2147483648)
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

#define CD_MAX(a, b) (((a) > (b)) ? (a) : (b))               // 取大
#define CD_MIN(a, b) (((a) < (b)) ? (a) : (b))               // 取小
#define CD_CLIP(x, mini, maxi) CD_MIN(CD_MAX(x, mini), maxi) // 截断

#define CD_GET_EU_DIST_SQR(x1, y1, x2, y2) ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) // 欧式距离平方
#define CD_GET_EU_DIST(x1, y1, x2, y2) (sqrtf(CD_GET_EU_DIST_SQR(x1, y1, x2, y2)))         // 欧式距离
#define CD__ALMOST_EQUAL(v1, v2, eps) (((v1 - v2) <= (eps)) && ((v2 - v1) <= (eps)))

    /**
     * @brief  [-2*pi, 2*pi] 归一化到 [-pi, pi]
     * @param angle
     * @return 归一化的角度
     */
    CD_INLINE CD_F32 cd_unwind_anglle(CD_F32 angle)
    {
        if (angle < -CD_PI)
        {
            return angle + 2.0f * CD_PI;
        }
        else if (angle > CD_PI)
        {
            return angle - 2.0f * CD_PI;
        }

        return angle;
    }

    /**
     * @brief  任意角度归一化到  [-pi, pi]
     * @param angle
     * @return 归一化的角度
     */
    CD_INLINE CD_F32 cd_unwind_large_anglle(CD_F32 angle)
    {
        while (angle > CD_PI)
        {
            angle -= 2.0f * CD_PI;
        }

        while (angle < -CD_PI)
        {
            angle += 2.0f * CD_PI;
        }

        return angle;
    }
    /**
     * @brief 归一化角度，将角度归一化到[0, 2pi]
     * @param angle
     * @return
     */
    CD_INLINE CD_F32 cd_wrap_angle(CD_F32 angle)
    {
        while (angle < 0)
        {
            angle += 2.0f * CD_PI;
        }
        while (angle >= 2.0f * CD_PI)
        {
            angle -= 2.0f * CD_PI;
        }
        return angle;
    }

    CD_INLINE CD_BOOL cd_is_with_in(CD_F32 val, CD_F32 bound1, CD_F32 bound2)
    {
        return (val >= CD_MIN(bound1, bound2)) && (val <= CD_MAX(bound1, bound2));
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_MATH_H__ */
