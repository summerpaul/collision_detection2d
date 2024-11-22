/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-15 11:07:44
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-22 17:05:42
 */

#ifndef __COLLISION_DETECTION_POLYGON_H__
#define __COLLISION_DETECTION_POLYGON_H__

#include "collision_detection_type.h"
#include "collision_detection_vec2.h"
#include "collision_detection_distance.h"
#include "collision_detection_transform.h"
#include "collision_detection_aabb.h"
#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct _CD_POLYGON_
    {
        /// The polygon vertices
        CD_VEC2 vertices[MAX_POLYGON_VERTICES];

        /// The outward normal vectors of the polygon sides
        CD_VEC2 normals[MAX_POLYGON_VERTICES];

        /// The centroid of the polygon
        CD_VEC2 centroid;

        /// The external radius for rounded polygons
        CD_F32 radius;

        /// The number of polygon vertices
        CD_S32 count;
    } CD_POLYGON;

    /**
     * @brief 判断点是否在多边形内
     * @param point
     * @param polygon
     * @param result
     * @return
     */
    CD_INLINE CD_RET cd_point_in_polygon(CD_VEC2 *point, CD_POLYGON *polygon, CD_BOOL *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(point == CD_NULL || polygon == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_DISTANCE_INPUT input = {0};
        ret = cd_make_proxy(polygon->vertices, polygon->count, 0.0f, &input.proxyA);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        ret = cd_make_proxy(point, 1, 0.0f, &input.proxyB);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        input.transformA = TRANSFORM_IDENTITY;
        input.transformB = TRANSFORM_IDENTITY;
        input.useRadii = CD_FALSE;
        CD_DISTANCE_CACHE cache = {0};
        CD_DISTANCE_OUTPUT output = {0};
        return ret;
    }

    CD_INLINE CD_RET cd_polygon_to_aabb(const CD_POLYGON *polygon, CD_AABB *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(polygon == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_CHECK_ERROR(polygon->count < 3, COLLISION_DETECTION_E_PARAM_NULL);

        result->lowerBound = polygon->vertices[0];
        result->upperBound = polygon->vertices[0];

        for (CD_S32 i = 0; i < polygon->count; ++i)
        {
            ret = cd_vec2_min(&result->lowerBound, &polygon->vertices[i], &result->lowerBound);
            CD_CHECK_ERROR(ret != CD_RET_OK, ret);
            ret = cd_vec2_max(&result->upperBound, &polygon->vertices[i], &result->upperBound);
            CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        }
        return ret;
    }

#ifdef __cplusplus
}
#endif
#endif /* __COLLISION_DETECTION_POLYGON_H__ */
