/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-13 13:51:36
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-15 21:27:11
 */

#ifndef __COLLISION_DETECTION_VEC2_H__
#define __COLLISION_DETECTION_VEC2_H__

#include "collision_detection_type.h"
#include "collision_detection_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct _CD_VEC2_
    {
        CD_F32 x;
        CD_F32 y;
    } CD_VEC2;

    static const CD_VEC2 Vec2_Zero = {0.0f, 0.0f}; // 0向量

    /**
     * @brief 根据朝向构建单位向量
     * @param heading 向量朝向
     * @param result 单位向量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_create_unit_vec2(CD_F32 heading, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = cosf(heading);
        result->y = sinf(heading);
        return ret;
    }

    /**
     * @brief 计算向量的长度
     * @param v 向量
     * @param result 向量的长度
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_len(CD_VEC2 *v, CD_F32 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(v == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = sqrtf(v->x * v->x + v->y * v->y);
        return ret;
    }

    /**
     * @brief 计算向量长度的平方
     * @param v 向量
     * @param result 向量长度的平方
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_len_sqr(CD_VEC2 *v, CD_F32 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(v == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = v->x * v->x + v->y * v->y;
        return ret;
    }

    /**
     * @brief 向量点积
     * @param a 向量a
     * @param b 向量b
     * @param result 点积结果
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_dot(CD_VEC2 *a, CD_VEC2 *b, CD_F32 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = a->x * b->x + a->y * b->y;
        return ret;
    }

    /**
     * @brief 向量叉积
     * @param a 向量a
     * @param b 向量b
     * @param result 叉积结果
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_cross(CD_VEC2 *a, CD_VEC2 *b, CD_F32 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = a->x * b->y - a->y * b->x;
        return ret;
    }

    /**
     * @brief 求向量的朝向
     * @param v 向量
     * @param result 向量朝向
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_heading(CD_VEC2 *v, CD_F32 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(v == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = atan2f(v->y, v->x);
        return ret;
    }

    /**
     * @brief 求向左的垂直向量
     * @param v 向量
     * @param result 向左的垂直向量
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_left_perp(CD_VEC2 *v, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(v == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = -v->y;
        result->y = v->x;
        return ret;
    }

    /**
     * @brief 求向右的垂直向量
     * @param v 向量
     * @param result 向右的垂直向量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_right_perp(CD_VEC2 *v, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(v == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = v->y;
        result->y = -v->x;
        return ret;
    }

    /**
     * @brief 向量加法
     * @param a 向量a
     * @param b 向量b
     * @param result 向量a + 向量b
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_add(CD_VEC2 *a, CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = a->x + b->x;
        result->y = a->y + b->y;
        return ret;
    }

    /**
     * @brief 向量减法
     * @param a 向量a
     * @param b 向量b
     * @param result 向量a - 向量b
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_sub(CD_VEC2 *a, CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = a->x - b->x;
        result->y = a->y - b->y;
        return ret;
    }

    /**
     * @brief 求向量的负数
     * @param a 向量
     * @param result 向量的负数
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_neg(CD_VEC2 *a, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = -a->x;
        result->y = -a->y;
        return ret;
    }

    /**
     * @brief 从向量a到向量b,进行线性插值
     * @param a 向量a
     * @param b 向量b
     * @param t 插值距离
     * @param result  插值的结构
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_nlerp(CD_VEC2 *a, CD_VEC2 *b, CD_F32 t, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = a->x + t * (b->x - a->x);
        result->y = a->y + t * (b->y - a->y);
        return ret;
    }

    /**
     * @brief 向量的对应元素乘积
     * @param a 向量a
     * @param b 向量b
     * @param result 乘积结果
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_mul(CD_VEC2 *a, CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = a->x * b->x;
        result->y = a->y * b->y;
        return ret;
    }

    /**
     * @brief 向量的缩放
     * @param a 向量a
     * @param scale 缩放因子
     * @param result 缩放结果
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_scale(CD_VEC2 *a, CD_F32 scale, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = a->x * scale;
        result->y = a->y * scale;
        return ret;
    }

    /**
     * @brief 向量加上一个缩放的向量
     * @param a 向量a
     * @param scale 向量b缩放因子
     * @param b 向量b
     * @param result
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_mul_add(CD_VEC2 *a, CD_F32 scale, CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = a->x + b->x * scale;
        result->y = a->y + b->y * scale;
        return ret;
    }

    /**
     * @brief 向量减去一个缩放的向量
     * @param a 向量a
     * @param scale 向量b缩放因子
     * @param b 向量b
     * @param result
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_mul_sub(CD_VEC2 *a, CD_F32 scale, CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = a->x - b->x * scale;
        result->y = a->y - b->y * scale;
        return ret;
    }

    /**
     * @brief 向量的单位化
     * @param a 向量a
     * @param result 单位化结果
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_norm(CD_VEC2 *a, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 len;
        ret = cd_vec2_len(a, &len);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        if (len < CD_EPS)
        {
            result->x = 0.0f;
            result->y = 0.0f;
            return CD_RET_OK;
        }
        result->x = a->x / len;
        result->y = a->y / len;
        return ret;
    }

    /**
     * @brief 向量绝对值
     * @param a 向量a
     * @param result 绝对值向量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_abs(CD_VEC2 *a, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = CD_FABS(a->x);
        result->y = CD_FABS(a->y);
        return ret;
    }

    /**
     * @brief 求两个向量的每个元素的大值
     * @param a 向量a
     * @param b 向量b
     * @param result
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_max(CD_VEC2 *a, CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = CD_MAX(a->x, b->x);
        result->y = CD_MAX(a->y, b->y);
        return ret;
    }

    /**
     * @brief 求两个向量的每个元素的小值
     * @param a 向量a
     * @param b 向量b
     * @param result
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_min(CD_VEC2 *a, CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = CD_MIN(a->x, b->x);
        result->y = CD_MIN(a->y, b->y);
        return ret;
    }

    /**
     * @brief 向量v的值限制在向量a b之间
     * @param v 向量v
     * @param a 向量a
     * @param b 向量b
     * @param result
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_clamp(CD_VEC2 *v, CD_VEC2 *a, CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(v == CD_NULL || a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = CD_CLIP(v->x, a->x, b->x);
        result->y = CD_CLIP(v->y, a->y, b->y);
        return ret;
    }

    /**
     * @brief 求两个点之间的距离
     * @param a 点a
     * @param b 点b
     * @param result 两个点之间的距离
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_dis(CD_VEC2 *a, CD_VEC2 *b, CD_F32 *result)
    {

        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == NULL || b == NULL || result == NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 dx = a->x - b->x;
        CD_F32 dy = a->y - b->y;
        *result = sqrtf(dx * dx + dy * dy);
        return ret;
    }

    /**
     * @brief 求两个点之间的距离平方
     * @param a 点a
     * @param b 点b
     * @param result 两个点之间的距离平方
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_dis_sqr(CD_VEC2 *a, CD_VEC2 *b, CD_F32 *result)
    {

        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == NULL || b == NULL || result == NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 dx = a->x - b->x;
        CD_F32 dy = a->y - b->y;
        *result = dx * dx + dy * dy;
        return ret;
    }

    /**
     * @brief 求向量a的长度并归一化
     * @param a 向量a
     * @param result_len 向量长度
     * @param result_v 单位向量
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_vec2_get_len_and_norm(CD_VEC2 *a, CD_F32 *result_len, CD_VEC2 *result_v)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == NULL || result_len == NULL || result_v == NULL, COLLISION_DETECTION_E_PARAM_NULL);
        ret = cd_vec2_len(a, result_len);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        ret = cd_vec2_norm(a, result_v);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);

        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_VEC2_H__ */
