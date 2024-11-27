/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-15 11:06:54
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-27 16:31:40
 */

#ifndef __COLLISION_DETECTION_OBB_H__
#define __COLLISION_DETECTION_OBB_H__

#include "collision_detection_vec2.h"
#include "collision_detection_transform.h"
#include "collision_detection_segment.h"
#include "collision_detection_aabb.h"

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

    /**
 * @brief 构建obb
 * @param center obb中心
 * @param length obb长
 * @param width obb宽
 * @param heading obb朝向
 * @param result obb结构
 * @return ok / 参数异常  
 */
    CD_INLINE CD_RET cd_create_obb(const CD_VEC2 *center, CD_F32 length, CD_F32 width, CD_F32 heading, CD_OBB *result)
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

    /**
 * @brief 将obb转换成aabb
 * @param obb obb
 * @param result aabb
 * @return ok / 参数异常   
 */
    CD_INLINE CD_RET cd_obb_to_aabb(const CD_OBB *obb, CD_AABB *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(obb == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 half_length = obb->length * 0.5f;
        CD_F32 half_width = obb->width * 0.5f;
        CD_VEC2 half_extents = {half_length, half_width};
        ret = cd_vec2_add(&obb->center, &half_extents, &result->lowerBound);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        ret = cd_vec2_sub(&obb->center, &half_extents, &result->upperBound);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

    /**
 * @brief 获取obb的heading
 * @param obb 
 * @param result 
 * @return ok / 参数异常   
 */
    CD_INLINE CD_RET cd_obb_heading(const CD_OBB *obb, CD_F32 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(obb == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        ret = cd_rot_to_angle(&obb->q, result);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

    /**
 * @brief 判断点是否在obb内
 * @param obb 
 * @param point 
 * @param result 
 * @return ok / 参数异常    
 */
    CD_INLINE CD_RET cd_is_point_in_obb(const CD_OBB *obb, const CD_VEC2 *point, CD_BOOL *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(obb == CD_NULL || point == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        const CD_F32 x0 = point->x - obb->center.x;
        const CD_F32 y0 = point->y - obb->center.y;
        const CD_F32 dx = CD_FABS(x0 * obb->q.c + y0 * obb->q.s);
        const CD_F32 dy = CD_FABS(y0 * obb->q.c - x0 * obb->q.s);
        *result = (dx <= obb->length * 0.5f + CD_EPS) && (dy <= obb->width * 0.5f + CD_EPS);
        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_OBB_H__ */
