/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-15 11:08:40
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-19 14:17:43
 */

#ifndef __COLLISION_DETECTION_CIRCLE_H__
#define __COLLISION_DETECTION_CIRCLE_H__

#include "collision_detection_type.h"
#include "collision_detection_vec2.h"
#include "collision_detection_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 圆
    typedef struct _CD_CIRCLE_
    {
        CD_VEC2 center; // 中心
        CD_F32 radius;  // 半径
    } CD_CIRCLE;

    CD_INLINE CD_RET cd_point_in_circle(CD_VEC2 *point, CD_CIRCLE *circle, CD_BOOL *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(point == CD_NULL || circle == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 dist_sqr = 0;
        ret = cd_vec2_dis_sqr(point, &circle->center, &dist_sqr);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        *result = (dist_sqr <= CD_SQUARE(circle->radius));
        return ret;
    }
#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_CIRCLE_H__ */
