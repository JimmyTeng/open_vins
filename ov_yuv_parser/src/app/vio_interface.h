//
// Created by tengz on 2025/12/19.
//

#ifndef VIO_INTERFACE_H
#define VIO_INTERFACE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================
 * 1. 基础数据类型 (Basic Types)
 * ========================================== */

// 时间戳使用 uint64_t
// 【重要约定】单位：纳秒 (Nanoseconds, 1e-9 sec)
// 这样可以避免长时间运行后的浮点精度抖动
typedef uint64_t vio_timestamp_t;

// 3D 向量 (x, y, z) - 物理量仍建议使用 double 保持精度
typedef struct {
    double data[3];
} vio_vec3_t;

// 四元数 (x, y, z, w) - Hamilton Convention (w 在最后)
typedef struct {
    double data[4];
} vio_quat_t;

// 图像格式
typedef enum {
    VIO_PIXEL_FMT_GRAY8 = 0,
    VIO_PIXEL_FMT_RGB888 = 1,
    VIO_PIXEL_FMT_RGBA8888 = 2
} vio_pixel_format_e;

/* ==========================================
 * 2. 核心状态枚举 (System Status)
 * ========================================== */

typedef enum {
    // 系统刚启动，没有任何数据，尚未准备好
    VIO_STATUS_NOT_READY = 0,

    // 正在初始化 (如：静止对齐、重力方向估计、SfM 地图构建)
    // 此时 Pose 数据不可用或不可信
    VIO_STATUS_INITIALIZING = 1,

    // 正常跟踪状态，Pose 数据可用
    VIO_STATUS_TRACKING = 2,

    // 跟踪丢失 (特征点不足、剧烈运动)，需要重定位
    VIO_STATUS_LOST = 3
} vio_system_status_e;

/* ==========================================
 * 3. 输入数据协议 (Input Payloads)
 * ========================================== */

// IMU 数据包
typedef struct {
    vio_timestamp_t timestamp; // ns
    vio_vec3_t acc;            // m/s^2
    vio_vec3_t gyro;           // rad/s
} vio_imu_msg_t;

// 图像数据包
typedef struct {
    vio_timestamp_t timestamp; // ns
    int32_t camera_id;         // 多目支持

    int32_t width;
    int32_t height;
    int32_t stride;            // Step (bytes per row)

    vio_pixel_format_e format;
    const uint8_t* buffer;     // 原始数据指针 (调用者持有内存)
} vio_image_msg_t;

/* ==========================================
 * 4. 输出数据协议 (Output State)
 * ========================================== */

typedef struct {
    vio_timestamp_t timestamp; // ns (与对应的图像帧时间戳对齐)

    // 系统当前状态
    vio_system_status_e status;

    // 位姿 (T_world_body)
    vio_vec3_t position;
    vio_quat_t orientation;

    // 速度 (World frame)
    vio_vec3_t velocity;

    // 传感器零偏 (用于监控)
    vio_vec3_t gyro_bias;
    vio_vec3_t acc_bias;

    // 可选：置信度/协方差对角线 (用于下游融合)
    // 如果算法不支持，可设为 -1
    double position_covariance[3];

} vio_state_t;

/* ==========================================
 * 5. 配置结构 (Configuration)
 * ========================================== */

typedef struct {
    // IMU 噪声参数 (通常单位是 continuous time, 需要算法内部处理离散化)
    double acc_noise_density;
    double acc_random_walk;
    double gyr_noise_density;
    double gyr_random_walk;

    // 外参 T_bc (Body -> Camera)
    vio_vec3_t t_bc;
    vio_quat_t r_bc;

    // 初始化相关配置
    bool enable_online_calibration; // 是否在线精修外参
    int32_t init_window_size;       // 初始化需要的帧数/时间

    // 调试/日志等级 (0=None, 1=Info, 2=Debug)
    int32_t log_level;
} vio_config_t;

/* ==========================================
 * 6. API 函数 (Functions)
 * ========================================== */

// 不透明句柄
typedef struct vio_instance_t* vio_handle_t;

// 回调定义
typedef void (*vio_state_callback_t)(const vio_state_t* state, void* user_data);


// --- 生命周期 ---
vio_handle_t vio_create(const char* yaml_path);
int          vio_init(vio_handle_t handle);
void         vio_destroy(vio_handle_t handle);
void         vio_reset(vio_handle_t handle); // 强制重置算法

// --- 数据输入 ---
// 注意：timestamp 单位必须是纳秒
void vio_push_imu(vio_handle_t handle, const vio_imu_msg_t* imu);
void vio_push_image(vio_handle_t handle, const vio_image_msg_t* img);

// --- 数据输出 ---
void vio_register_state_callback(vio_handle_t handle,
                                 vio_state_callback_t callback,
                                 void* user_data);

#ifdef __cplusplus
}
#endif

#endif // VIO_INTERFACE_H
