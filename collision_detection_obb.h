/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-15 11:06:54
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-20 22:35:41
 */

#ifndef __COLLISION_DETECTION_OBB_H__
#define __COLLISION_DETECTION_OBB_H__

#include "collision_detection_vec2.h"
#include "collision_detection_transform.h"
#include "collision_detection_segment.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 方向包围盒
    typedef struct _CD_OBB_
    {
        CD_VEC2 center; // 中心
        CD_F32 length;  // 长
        CD_F32 width;   // 宽
        CD_ROT q;       // 旋转量

    } CD_OBB;

    CD_INLINE CD_RET cd_create_obb(CD_VEC2 *center, CD_F32 length, CD_F32 width, CD_F32 heading, CD_OBB *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(center == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->center = *center;
        result->length = length;
        result->width = width;
        ret = cd_rot_from_angle(&result->q, heading);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_OBB_H__ */
