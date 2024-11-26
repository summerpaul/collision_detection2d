/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-26 13:45:18
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-26 13:58:35
 */

#ifndef __COLLISION_DETECTION_MAT22_H__
#define __COLLISION_DETECTION_MAT22_H__

#include "collision_detection_vec2.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct _CD_MAT22_
    {
        CD_VEC2 cx;
        CD_VEC2 cy;
    } CD_MAT22;

    /**
 * @brief 2*2矩阵与2*1向量相乘
 * 
 * @param A 矩阵
 * @param v 向量
 * @param result 结果向量
 * @return ok / 参数异常 
 */
    CD_INLINE CD_RET cd_mul_Mat_vec2(const CD_MAT22 *A, const CD_VEC2 *v, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(A == CD_NULL || v == CD_NULL || result == CD_NULL,
                       COLLISION_DETECTION_E_PARAM_NULL);

        result->x = A->cx.x * v->x + A->cy.x * v->y;
        result->y = A->cx.y * v->x + A->cy.y * v->y;
        return ret;
    }

    /**
 * @brief 求2*2矩阵的逆
 * 
 * @param A 
 * @param result 
 * @return ok / 参数异常  
 */
    CD_INLINE CD_RET cd_get_inv_mat22(const CD_MAT22 *A, CD_MAT22 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(A == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 a = A->cx.x;
        CD_F32 b = A->cx.y;
        CD_F32 c = A->cy.x;
        CD_F32 d = A->cy.y;
        CD_F32 det = a * d - b * c;
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }
        result->cx.x = d * det;
        result->cx.y = -b * det;
        result->cy.x = -c * det;
        result->cy.y = a * det;
        return ret;
    }

    /**
 * @brief 求解 A * x = b
 * 
 * @param A 
 * @param b 
 * @param result 
 * @return CD_INLINE 
 */
    CD_INLINE CD_RET cd_solve22(const CD_MAT22 *A, const CD_VEC2 *b, CD_VEC2 *result)
    {
        CD_RET ret = CD_RET_OK;
        CD_CHECK_ERROR(A == CD_NULL || b == CD_NULL || result == CD_NULL, COLLISION_DETECTION_E_PARAM_NULL);
        CD_F32 a11 = A->cx.x;
        CD_F32 a21 = A->cx.y;
        CD_F32 a12 = A->cy.x;
        CD_F32 a22 = A->cy.y;
        CD_F32 det = a11 * a22 - a12 * a21;
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }
        result->x = (a22 * b->x - a12 * b->y) * det;
        result->y = (a11 * b->y - a21 * b->x) * det;
        return ret;
    }

#ifdef __cplusplus
}
#endif

#endif /* __COLLISION_DETECTION_MAT22_H__ */
