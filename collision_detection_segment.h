/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-13 15:01:49
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-27 16:10:53
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
    CD_INLINE CD_RET cd_segment_len(const CD_SEGMENT *seg, CD_F32 *result_len)
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
    CD_INLINE CD_RET cd_segment_len_sqr(const CD_SEGMENT *seg, CD_F32 *result_len_sqr)
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
    CD_INLINE CD_RET cd_segment_unit_dir(const CD_SEGMENT *seg, CD_VEC2 *result_v)
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
    CD_INLINE CD_RET cd_segment_heading(const CD_SEGMENT *seg, CD_F32 *result_heading)
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
    CD_INLINE CD_RET cd_segment_center(const CD_SEGMENT *seg, CD_VEC2 *result_center)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || result_center == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result_center->x = (seg->point1.x + seg->point2.x) * 0.5f;
        result_center->y = (seg->point1.y + seg->point2.y) * 0.5f;
        return ret;
    }

    /**
 * @brief 绕线段的起点进行旋转
 * @param seg  旋转前线段
 * @param angle 旋转的角度
 * @param result 旋转后的线段
 * @return  ok / 参数异常 
 */
    CD_INLINE CD_RET cd_segment_rotate(const CD_SEGMENT *seg, CD_F32 angle, CD_SEGMENT *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_VEC2 diff_vec;
        ret = cd_vec2_sub(&seg->point2, &seg->point1, &diff_vec);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        CD_ROT rot;
        ret = cd_rot_from_angle(&rot, angle);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        ret = cd_rot_vector(&rot, &diff_vec, &diff_vec);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        result->point1 = seg->point1;
        ret = cd_vec2_add(&result->point1, &diff_vec, &result->point2);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

    /**
 * @brief 计算点到线段的距离，并找到最近点
 * @param seg 线段
 * @param point 点
 * @param nearest_pt 最近点，可为null 
 * @param distance 距离，可为null
 * @return  ok / 参数异常 
 */
    CD_INLINE CD_RET cd_segment_dis_to_point(const CD_SEGMENT *seg, const CD_VEC2 *point, CD_VEC2 *nearest_pt, CD_F32 *distance)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || point == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        // 计算线段的长度
        CD_F32 length;
        ret = cd_segment_len(seg, &length);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (length <= CD_EPS)
        {
            if (distance != CD_NULL)
            {
                ret = cd_vec2_dis(point, &seg->point1, distance);
                CD_CHECK_ERROR(ret != CD_RET_OK, ret);
            }
            if (nearest_pt != CD_NULL)
            {
                *nearest_pt = seg->point1;
            }
            return ret;
        }
        const CD_F32 x0 = point->x - seg->point1.x;
        const CD_F32 y0 = point->y - seg->point1.y;
        CD_VEC2 unit_direction;
        ret = cd_segment_unit_dir(seg, &unit_direction);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        // 计算点在线段上的投影
        const CD_F32 proj = x0 * unit_direction.x + y0 * unit_direction.y;
        if (proj < 0)
        {
            if (distance != CD_NULL)
            {
                ret = cd_vec2_dis(point, &seg->point1, distance);
                CD_CHECK_ERROR(ret != CD_RET_OK, ret);
            }
            if (nearest_pt != CD_NULL)
            {
                *nearest_pt = seg->point1;
            }
            return ret;
        }
        if (proj > length)
        {
            if (distance != CD_NULL)
            {
                ret = cd_vec2_dis(point, &seg->point2, distance);
                CD_CHECK_ERROR(ret != CD_RET_OK, ret);
            }
            if (nearest_pt != CD_NULL)
            {
                *nearest_pt = seg->point2;
            }
            return ret;
        }
        if (nearest_pt != CD_NULL)
        {
            ret = cd_vec2_mul_add(&seg->point1, proj, &unit_direction, nearest_pt);
            CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        }
        if (distance != CD_NULL)
        {
            *distance = CD_FABS(x0 * unit_direction.y - y0 * unit_direction.x);
        }
        return ret;
    }

    /**
     * @brief 判断点是否在线段上
     * @param seg 线段
     * @param point 点
     * @param result 1 在线段上，0 不在线段上
     * @return  ok / 参数异常 
     */
    CD_INLINE CD_RET cd_is_point_in_seg(const CD_SEGMENT *seg, const CD_VEC2 *point, CD_BOOL *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg == CD_NULL || point == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 length;
        ret = cd_segment_len(seg, &length);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (length <= CD_EPS)
        {
            *result = (CD_FABS(point->x - seg->point1.x) <= CD_EPS) && (CD_FABS(point->y - seg->point1.y) <= CD_EPS);
        }
        CD_F32 prod;
        ret = cd_points_cross(point, &seg->point1, &seg->point2, &prod);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (CD_FABS(prod) > CD_EPS)
        {
            *result = CD_FALSE;
            return ret;
        }
        *result = cd_is_with_in(point->x, seg->point1.x, seg->point2.x) && cd_is_with_in(point->y, seg->point1.y, seg->point2.y);
        return ret;
    }

    /**
     * @brief 判断两线段是否相交
     * @param seg1 线段1
     * @param seg2 线段2
     * @param result 1 相交，0 不相交
     * @return  ok / 参数异常 
     */
    CD_INLINE CD_RET cd_segments_intersect(const CD_SEGMENT *seg1, const CD_SEGMENT *seg2, CD_VEC2 *point, CD_BOOL *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(seg1 == CD_NULL || seg2 == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_BOOL point_in_seg = CD_FALSE;
        ret = cd_is_point_in_seg(seg1, &seg2->point1, &point_in_seg);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (point_in_seg)
        {
            *result = CD_TRUE;
            if (point != CD_NULL)
            {
                *point = seg2->point1;
            }
            return ret;
        }
        point_in_seg = CD_FALSE;
        ret = cd_is_point_in_seg(seg1, &seg2->point2, &point_in_seg);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (point_in_seg)
        {
            *result = CD_TRUE;
            if (point != CD_NULL)
            {
                *point = seg2->point2;
            }
            return ret;
        }
        point_in_seg = CD_FALSE;
        ret = cd_is_point_in_seg(seg2, &seg1->point1, &point_in_seg);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (point_in_seg)
        {
            *result = CD_TRUE;
            if (point != CD_NULL)
            {
                *point = seg1->point1;
            }
            return ret;
        }
        point_in_seg = CD_FALSE;
        ret = cd_is_point_in_seg(seg2, &seg1->point2, &point_in_seg);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (point_in_seg)
        {
            *result = CD_TRUE;
            if (point != CD_NULL)
            {
                *point = seg1->point2;
            }
            return ret;
        }

        CD_F32 seg_1_length;
        CD_F32 seg_2_length;
        ret = cd_segment_len(seg1, &seg_1_length);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        ret = cd_segment_len(seg2, &seg_2_length);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (seg_1_length <= CD_EPS || seg_2_length <= CD_EPS)
        {
            *result = CD_FALSE;
            return ret;
        }
        CD_F32 cc1;
        ret = cd_points_cross(&seg1->point1, &seg1->point2, &seg2->point1, &cc1);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        CD_F32 cc2;
        ret = cd_points_cross(&seg1->point1, &seg1->point2, &seg2->point2, &cc2);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (cc1 * cc2 >= -CD_EPS)
        {
            *result = CD_FALSE;
            return ret;
        }
        *result = true;
        if (point != CD_NULL)
        {
            CD_F32 cc3;
            ret = cd_points_cross(&seg2->point1, &seg2->point2, &seg1->point1, &cc3);
            CD_CHECK_ERROR(ret != CD_RET_OK, ret);
            CD_F32 cc4;
            ret = cd_points_cross(&seg2->point1, &seg2->point2, &seg1->point2, &cc4);
            CD_CHECK_ERROR(ret != CD_RET_OK, ret);
            const CD_F32 ratio = cc4 / (cc4 - cc3);
            point->x = seg1->point1.x * ratio + seg1->point2.x * (1 - ratio);
            point->y = seg1->point1.y * ratio + seg1->point2.y * (1 - ratio);
        }
        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_SEGMENT_H__ */
