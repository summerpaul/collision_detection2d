/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-19 13:35:45
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-20 22:36:48
 */
#include <stdint.h>

#ifndef __COLLISION_DETECTION_DISTANCE_H__
#define __COLLISION_DETECTION_DISTANCE_H__
#include "collision_detection_type.h"
#include "collision_detection_vec2.h"
#include "collision_detection_transform.h"
#include "collision_detection_math.h"
#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct _CD_DISTANCE_PROXY_
    {
        /// The point cloud
        CD_VEC2 points[MAX_POLYGON_VERTICES];

        /// The number of points
        CD_S32 count;

        /// The external radius of the point cloud
        CD_F32 radius;
    } CD_DISTANCE_PROXY;

    typedef struct _CD_DISTANCE_CACHE_
    {
        /// The number of stored simplex points
        CD_U16 count;

        /// The cached simplex indices on shape A
        CD_U08 indexA[3];

        /// The cached simplex indices on shape B
        CD_U08 indexB[3];
    } CD_DISTANCE_CACHE;

    static const CD_DISTANCE_CACHE emptyDistanceCache = {0};

    typedef struct _CD_DISTANCE_INPUT_
    {
        /// The proxy for shape A
        CD_DISTANCE_PROXY proxyA;

        /// The proxy for shape B
        CD_DISTANCE_PROXY proxyB;

        /// The world transform for shape A
        CD_TRANSFORM transformA;

        /// The world transform for shape B
        CD_TRANSFORM transformB;

        /// Should the proxy radius be considered?
        CD_BOOL useRadii;
    } CD_DISTANCE_INPUT;

    typedef struct _CD_DISTANCE_OUTPUT_
    {
        CD_VEC2 pointA;      ///< Closest point on shapeA
        CD_VEC2 pointB;      ///< Closest point on shapeB
        CD_F32 distance;     ///< The final distance, zero if overlapped
        CD_S32 iterations;   ///< Number of GJK iterations used
        CD_S32 simplexCount; ///< The number of simplexes stored in the simplex array
    } CD_DISTANCE_OUTPUT;

    /// Simplex vertex for debugging the GJK algorithm
    typedef struct _CD_SIMPLEX_VERTEX_
    {
        CD_VEC2 wA;    ///< support point in proxyA
        CD_VEC2 wB;    ///< support point in proxyB
        CD_VEC2 w;     ///< wB - wA
        CD_F32 a;      ///< barycentric coordinate for closest point
        CD_S32 indexA; ///< wA index
        CD_S32 indexB; ///< wB index
    } CD_SIMPLEX_VERTEX;

    /// Simplex from the GJK algorithm
    typedef struct _CD_SIMPLEX_
    {
        CD_SIMPLEX_VERTEX v1, v2, v3; ///< vertices
        CD_S32 count;                 ///< number of valid vertices
    } CD_SIMPLEX;

    CD_INLINE CD_RET cd_make_proxy(CD_VEC2 *vertices, CD_S16 count, CD_F32 radius, CD_DISTANCE_PROXY *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(vertices == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        count = CD_MIN(count, MAX_POLYGON_VERTICES);
        for (CD_S32 i = 0; i < count; i++)
        {
            result->points[i] = vertices[i];
        }
        result->count = count;
        result->radius = radius;
        return ret;
    }

    CD_INLINE CD_RET cd_shape_distance(CD_DISTANCE_CACHE *cache,
                                       CD_DISTANCE_INPUT *input,
                                       CD_SIMPLEX *simplexes,
                                       CD_S32 simplexCapacity,
                                       CD_DISTANCE_OUTPUT *output)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(cache == CD_NULL || input == CD_NULL || simplexes == CD_NULL || output == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_DISTANCE_H__ */
