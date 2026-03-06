/*
 * Debug 扩展：编译信息导出（Build Info）、SLAM 回调
 * 命名与数据结构参考 ss_vio_err.h / ss_vio.h
 */
#ifndef SS_VIO_DEBUG_H
#define SS_VIO_DEBUG_H

#include "stddef.h"
#include "ss_vio.h"
#include "../system/ov_core_export.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========== SLAM 回调 ========== */
/** 单次回调中 SLAM 3D 点数量上限 */
#define OT_VIO_SLAM_POINTS_MAX 512

/** SLAM 回调数据：当前帧位姿 + SLAM 特征点集（指针仅在回调内有效，不得保存） */
typedef struct {
  double timestamp;                    /**< 时间戳（秒） */
  OT_VIO_PoseData pose;                /**< 当前位姿 */
  int pointNum;                        /**< SLAM 3D 点数量，0 表示无或未初始化 */
  const OT_VIO_3DPoint* pointSets;     /**< 3D 点数组，仅回调内有效 */
} OT_VIO_SlamCallbackData_t;

/** SLAM 回调函数类型 */
typedef void (*OT_VIO_SlamCallback_t)(const OT_VIO_SlamCallbackData_t* data, void* user_data);

/**
 * 注册 SLAM 回调（每帧状态更新时在内部 state 回调之后调用）
 * @param callback 回调函数，传 NULL 表示仅注销
 * @param user_data 透传给 callback 的 user_data
 * @return 0 成功，OT_VIOALG_ERR_CALLBACK_NULL 表示 callback 为 NULL 且未注销
 */
SS_VIO_API int SS_VIO_RegisterSlamCallback(OT_VIO_SlamCallback_t callback, void* user_data);

/** 注销 SLAM 回调 */
SS_VIO_API void SS_VIO_UnregisterSlamCallback(void);

/* ========== Build Info ========== */

/** 单条编译信息字段长度上限 */
#define SS_VIO_BUILD_INFO_STR_MAX 128

/** 编译信息结构（与 Build Info 打印一一对应） */
typedef struct {
  char build_time[SS_VIO_BUILD_INFO_STR_MAX];   /**< Build Time，如 "Mar 06 2026 18:35:22" */
  char build_timezone[SS_VIO_BUILD_INFO_STR_MAX]; /**< 如 "CST (UTC+8)" */
  char git_commit[SS_VIO_BUILD_INFO_STR_MAX];   /**< Git Commit 短哈希 */
  char git_hash[SS_VIO_BUILD_INFO_STR_MAX];     /**< Git 完整哈希 */
  char git_branch[SS_VIO_BUILD_INFO_STR_MAX];  /**< Git Branch */
  char git_tag[SS_VIO_BUILD_INFO_STR_MAX];     /**< Git Tag，无则 "none" */
  char git_status[SS_VIO_BUILD_INFO_STR_MAX];  /**< Git Status，"dirty" / "clean" */
  char git_user[SS_VIO_BUILD_INFO_STR_MAX];    /**< Git User Name */
  char git_email[SS_VIO_BUILD_INFO_STR_MAX];   /**< Git User Email */
} SS_VIO_BuildInfo;

/**
 * 获取当前 ov_core 编译信息（编译时生成的 build_info.h）
 * @param out 输出结构体，由调用方分配，不得为 NULL
 * @return 0 成功，非 0 失败
 */
SS_VIO_API int SS_VIO_GetBuildInfo(SS_VIO_BuildInfo* out);

/**
 * 将编译信息格式化为与终端一致的多行字符串，写入 buffer
 * @param buffer 输出缓冲区
 * @param size   buffer 字节数（含结尾 '\0'）
 * @return 写入的字节数（不含 '\0'），若 buffer 不足则返回所需长度；失败返回 <0
 */
SS_VIO_API int SS_VIO_FormatBuildInfo(char* buffer, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* SS_VIO_DEBUG_H */
