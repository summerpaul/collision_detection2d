/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-15 11:06:42
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-22 16:32:28
 */

#ifndef __COLLISION_DETECTION_AABB_H__
#define __COLLISION_DETECTION_AABB_H__

#include "collision_detection_vec2.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 轴对齐包围盒
    typedef struct _CD_AABB_
    {
        CD_VEC2 lowerBound; // 坐标轴最小值
        CD_VEC2 upperBound; // 坐标轴最大值
    } CD_AABB;

    CD_INLINE CD_RET cd_create_aabb(const CD_VEC2 *center, CD_F32 length, CD_F32 width, CD_AABB *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(center == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->lowerBound.x = center->x - length * 0.5f;
        result->lowerBound.y = center->y - width * 0.5f;
        result->upperBound.x = center->x + length * 0.5f;
        result->upperBound.y = center->y + width * 0.5f;
        return ret;
    }
    CD_INLINE CD_RET cd_aabb_contains(const CD_AABB *a, const CD_AABB *b, CD_BOOL *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = (a->lowerBound.x <= b->lowerBound.x && a->lowerBound.y <= b->lowerBound.y &&
                   a->upperBound.x >= b->upperBound.x && a->upperBound.y >= b->upperBound.y);
        return ret;
    }

    CD_INLINE CD_RET cd_aabb_overlap(const CD_AABB *a, const CD_AABB *b, CD_BOOL *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = (a->lowerBound.x <= b->upperBound.x && a->upperBound.x >= b->lowerBound.x &&
                   a->lowerBound.y <= b->upperBound.y && a->upperBound.y >= b->lowerBound.y);
        return ret;
    }

    CD_INLINE CD_RET cd_aabb_center(const CD_AABB *a, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = (a->lowerBound.x + a->upperBound.x) * 0.5f;
        result->y = (a->lowerBound.y + a->upperBound.y) * 0.5f;
        return ret;
    }

    CD_INLINE CD_RET cd_aabb_union(const CD_AABB *a, const CD_AABB *b, CD_AABB *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->lowerBound.x = CD_MIN(a->lowerBound.x, b->lowerBound.x);
        result->lowerBound.y = CD_MIN(a->lowerBound.y, b->lowerBound.y);
        result->upperBound.x = CD_MAX(a->upperBound.x, b->upperBound.x);
        result->upperBound.y = CD_MAX(a->upperBound.y, b->upperBound.y);
        return ret;
    }

    CD_INLINE CD_RET cd_is_point_in_aabb(const CD_AABB *a, const CD_VEC2 *point, CD_BOOL *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || point == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = (point->x >= a->lowerBound.x && point->x <= a->upperBound.x &&
                   point->y >= a->lowerBound.y && point->y <= a->upperBound.y);
        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_AABB_H__ */
