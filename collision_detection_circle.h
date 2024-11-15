/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-15 11:08:40
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-15 15:16:33
 */

#ifndef __COLLISION_DETECTION_CIRCLE_H__
#define __COLLISION_DETECTION_CIRCLE_H__

#include "collision_detection_type.h"
#include "collision_detection_vec2.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 圆
    typedef struct _CD_CIRCLE_
    {
        CD_VEC2 center; // 中心
        CD_F32 radius;             // 半径
    } CD_CIRCLE;

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_CIRCLE_H__ */
