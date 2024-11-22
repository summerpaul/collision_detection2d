/**
 * @Author: Xia Yunkai
 * @Date:   2024-11-13 11:09:22
 * @Last Modified by:   Xia Yunkai
 * @Last Modified time: 2024-11-22 16:38:54
 */

#ifndef __COLLISION_DETECTION_TYPE_H__
#define __COLLISION_DETECTION_TYPE_H__
#include "collision_detection_base.h"

#define COLLISION_DETECTION_VERSION 24111313 // 当前版本号

#ifdef __cplusplus
#define CD_INLINE inline
#else
#define CD_INLINE static inline
#endif

#define CD_CHECK_ERROR(state, error_code) \
    if (state)                            \
    {                                     \
        return error_code;                \
    }

#define COLLISION_DETECTION_E_MEM_NULL 0x0000   // 内存地址为空
#define COLLISION_DETECTION_E_MEM_ALIGN 0x0001  // 内存对齐错误
#define COLLISION_DETECTION_E_CALC_ERROR 0x0002 // 内存不足
#define COLLISION_DETECTION_E_PARAM_NULL 0x0010 // 输入参数为空
#define COLLISION_DETECTION_E_ZERO_NUM   0x0020 // 输入参数为空

#define MAX_POLYGON_VERTICES 8

#endif /* __COLLISION_DETECTION_TYPE_H__ */
