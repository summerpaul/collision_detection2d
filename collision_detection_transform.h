/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-13 15:20:49
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-19 14:13:01
 */

#ifndef __COLLISION_DETECTION_TRANSFORM_H__
#define __COLLISION_DETECTION_TRANSFORM_H__
#include "collision_detection_vec2.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 2d平面旋转量,包含三角函数的cos与sin
    typedef struct _CD_ROT_
    {
        CD_F32 c; // 三角函数cos值
        CD_F32 s; // 三角函数sin值
    } CD_ROT;

    // 坐标变换，旋转+平移
    typedef struct _CD_TRANSFORM_
    {
        CD_VEC2 p; // 平移量
        CD_ROT q;  // 旋转量
    } CD_TRANSFORM;

    static const CD_TRANSFORM TRANSFORM_IDENTITY = {{0.0f, 0.0f}, {1.0f, 0.0f}};

    /**
     * @brief 通过角度计算旋转量
     * @param q 旋转量
     * @param angle 角度
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_from_angle(CD_ROT *q, CD_F32 angle)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        q->s = sinf(angle);
        q->c = cosf(angle);
        return ret;
    }

    /**
     * @brief 通过旋转量,计算角度
     * @param q 旋转量
     * @param angle 角度
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_to_angle(CD_ROT *q, CD_F32 *angle)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q == CD_NULL || angle == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *angle = atan2f(q->s, q->c);
        return ret;
    }

    /**
     * @brief 旋转量相乘  q * r
     * @param q 旋转量q
     * @param r 旋转量r
     * @param result
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_mul(CD_ROT *q, CD_ROT *r, CD_ROT *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q == CD_NULL || r == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->c = q->c * r->c - q->s * r->s;
        result->s = q->c * r->s + q->s * r->c;
        return ret;
    }

    /**
     * @brief 旋转量逆相乘 qT * r
     * @param q 旋转量q
     * @param r 旋转量r
     * @param result
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_inv_mul(CD_ROT *q, CD_ROT *r, CD_ROT *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q == CD_NULL || r == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->s = q->c * r->s - q->s * r->c;
        result->c = q->c * r->c + q->s * r->s;
        return ret;
    }

    /**
     * @brief 旋转量归一化
     * @param q 旋转量
     * @param result 归一化后的旋转量
     * @return ok / 参数异常
     */

    CD_INLINE CD_RET cd_rot_norm(CD_ROT *q, CD_ROT *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 mag = sqrtf(q->c * q->c + q->s * q->s);
        CD_F32 invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
        result->c = q->c * invMag;
        result->s = q->s * invMag;
        return ret;
    }

    /**
     * @brief 根据角速度,计算增量后的旋转量
     * @param q 增量前的旋转量
     * @param deltaAngle 角速增加量(弧度)
     * @param result 增量后的旋转量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_integrate_angle(CD_ROT *q, CD_F32 deltaAngle, CD_ROT *result)
    {
        // dc/dt = -omega * sin(t)
        // ds/dt = omega * cos(t)
        // c2 = c1 - omega * h * s1
        // s2 = s1 + omega * h * c1
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_ROT q2;
        q2.c = q->c - deltaAngle * q->s;
        q2.s = q->s + deltaAngle * q->c;
        CD_F32 mag = sqrtf(q2.s * q2.s + q2.c * q2.c);
        CD_F32 invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
        result->c = q2.c * invMag;
        result->s = q2.s * invMag;
        return ret;
    }

    /**
     * @brief 旋转量进行线性插值
     * @param q1 旋转量 q1
     * @param q2 旋转量 q2
     * @param t 偏移量
     * @param result 插值结果
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_nlerp(CD_ROT *q1, CD_ROT *q2, CD_F32 t, CD_ROT *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q1 == CD_NULL || q2 == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 omt = 1.0f - t;
        result->c = omt * q1->s + t * q2->c;
        result->s = omt * q1->c + t * q2->s;
        ret = cd_rot_norm(result, result);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

    /**
     * @brief 根据两个旋转量和时间倒数,计算角速度
     * @param q1 旋转量q1
     * @param q2 旋转量q2
     * @param inv_h 时间倒数
     * @param result 角速度
     * @return  ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_compute_angular_velocity(CD_ROT *q1, CD_ROT *q2, CD_F32 inv_h, CD_F32 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q1 == CD_NULL || q2 == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        *result = inv_h * (q2->s * q1->c - q2->c * q1->s);
        return ret;
    }

    /**
     * @brief 计算旋转量x轴单位向量
     * @param q1 旋转量
     * @param result x轴单位向量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_get_x_axis(CD_ROT *q1, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q1 == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = q1->c;
        result->y = q1->s;
        return ret;
    }

    /**
     * @brief 计算旋转量y轴单位向量
     * @param q1 旋转量
     * @param result y轴单位向量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_get_y_axis(CD_ROT *q1, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q1 == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = -q1->s;
        result->y = q1->c;
        return ret;
    }

    /**
     * @brief 旋转量b到旋转量a的夹角 (rot_b * inv(rot_a))
     * @param b 旋转量b
     * @param a 旋转量a
     * @param result 两个旋转量之间的夹角
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_relative_angle(CD_ROT *b, CD_ROT *a, CD_F32 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(b == CD_NULL || a == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 s = b->s * a->c - b->c * a->s;
        CD_F32 c = b->c * a->c + b->s * a->s;
        *result = atan2f(s, c);
        return ret;
    }

    /**
     * @brief 旋转一个向量
     * @param b 旋转量
     * @param v 向量
     * @param result 旋转后的结果
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_rot_vector(CD_ROT *q, CD_VEC2 *v, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q == CD_NULL || v == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = q->c * v->x - q->s * v->y;
        result->y = q->s * v->x + q->c * v->y;
        return ret;
    }

    /**
     * @brief 求向量旋转前的结果
     * @param b 旋转量
     * @param v 旋转后的向量
     * @param result 旋转前的向量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_inv_rot_vector(CD_ROT *q, CD_VEC2 *v, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(q == CD_NULL || v == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        result->x = q->c * v->x + q->s * v->y;
        result->y = -q->s * v->x + q->c * v->y;
        return ret;
    }

    /**
     * @brief 对一个点进行旋转+平移
     * @param t 旋转平移量
     * @param p 转换前的点
     * @param result 转换后的点
     * @return ok / 参数异常
     */

    CD_INLINE CD_RET cd_transforms_point(CD_TRANSFORM *t, CD_VEC2 *p, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        // CD_CHECK_ERROR(t == CD_NULL || p == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        // result->x = (t->q.c * p->x - t->q.s * p->y) + t->p.x;
        // result->y = (t->q.s * p->x + t->q.c * p->y) + t->p.y;
        return ret;
    }

    /**
     * @brief 求一个点旋转平移前的结果
     * @param t 旋转平移量
     * @param p 转换的点
     * @param result 转换前的点
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_inv_transforms_point(CD_TRANSFORM *t, CD_VEC2 *p, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(t == CD_NULL || p == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 vx = p->x - t->p.x;
        CD_F32 vy = p->y - t->p.y;
        result->x = (t->q.c * vx + t->q.s * vy);
        result->y = (-t->q.s * vx + t->q.c * vy);
        return ret;
    }

    /**
     * @brief a 叠加转换b
     * @param a  转换量a
     * @param b 转换量b
     * @param result 叠加后的转换量
     * @return ok / 参数异常
     */
    CD_INLINE CD_RET cd_transforms_mul(CD_TRANSFORM *a, CD_TRANSFORM *b, CD_TRANSFORM *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        ret = cd_rot_mul(&a->q, &b->q, &result->q);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        CD_VEC2 add_vec;
        ret = cd_rot_vector(&a->q, &b->p, &add_vec);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        ret = cd_vec2_add(&add_vec, &b->p, &result->p);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        return ret;
    }

    CD_INLINE CD_RET cd_transforms_inv_mul(CD_TRANSFORM *a, CD_TRANSFORM *b, CD_TRANSFORM *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(a == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        ret = cd_rot_inv_mul(&a->q, &b->q, &result->q);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        CD_VEC2 sub_vec;
        ret = cd_vec2_sub(&b->p, &a->p, &sub_vec);
        CD_CHECK_ERROR(ret != CD_RET_OK, ret);
        ret = cd_inv_rot_vector(&a->q, &sub_vec, &result->p);
        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_TRANSFORM_H__ */
