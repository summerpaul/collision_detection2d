/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-13 15:01:49
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-13 15:24:08
 */

#ifndef __COLLISION_DETECTION_SEGMENT_H__
#define __COLLISION_DETECTION_SEGMENT_H__
#include "collision_detection_vec2.h"
#include "collision_detection_transform.h"
#ifdef __cplusplus
extern "C"
{
#endif

    // 线段
    typedef struct _CD_SEGMENT_
    {
        CD_VEC2 point1; // 线段起点
        CD_VEC2 point2; // 线段终点
    } CD_SEGMENT;

    /**
     * @brief 计算线段的长
     * @param seg 线段
     * @param result_len 线段的长
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_segment_len(CD_SEGMENT *seg, CD_F32 *result_len)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || result_len == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        ret = cd_vec2_dis(&seg->point1, &seg->point2, result_len);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

    /**
     * @brief 计算线段的长的平方
     * @param seg 线段
     * @param result_len_sqr 线段的长的平方
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_segment_len_sqr(CD_SEGMENT *seg, CD_F32 *result_len_sqr)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || result_len_sqr == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        ret = cd_vec2_dis_sqr(&seg->point1, &seg->point2, result_len_sqr);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

    /**
     * @brief 计算线段的单位向量
     * @param seg 线段
     * @param result_v 线段的的单位向量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_segment_unit_dir(CD_SEGMENT *seg, CD_VEC2 *result_v)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || result_v == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        ret = cd_vec2_sub(&seg->point1, &seg->point2, result_v);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        ret = cd_vec2_norm(result_v, result_v);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

    /**
     * @brief 获取线段的朝向
     * @param seg 线段
     * @param result_heading 线段的朝向
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_segment_heading(CD_SEGMENT *seg, CD_F32 *result_heading)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || result_heading == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_VEC2 unit_direction;
        ret = cd_segment_unit_dir(seg, &unit_direction);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        *result_heading = atan2f(unit_direction.y, unit_direction.x);
        return ret;
    }

    /**
     * @brief 获取线段的中点
     * @param seg 线段
     * @param result_center 线段的中点
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_segment_center(CD_SEGMENT *seg, CD_VEC2 *result_center)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || result_center == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result_center->x = (seg->point1.x + seg->point2.x) * 0.5f;
        result_center->y = (seg->point1.y + seg->point2.y) * 0.5f;
        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_SEGMENT_H__ */
