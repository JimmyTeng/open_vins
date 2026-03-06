//
// VIO C API - 对外稳定接口，仅含类型与函数声明，无 C++ 实现细节
//

#ifndef VIO_INTERFACE_API_H
#define VIO_INTERFACE_API_H

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
typedef uint64_t vio_timestamp_t;

// 3D 向量 (x, y, z)
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
    VIO_STATUS_NOT_READY = 0,
    VIO_STATUS_INITIALIZING = 1,
    VIO_STATUS_TRACKING = 2,
    VIO_STATUS_LOST = 3
} vio_system_status_e;

/* ==========================================
 * 3. 输入数据协议 (Input Payloads)
 * ========================================== */

typedef struct {
    vio_timestamp_t timestamp; // ns
    vio_vec3_t acc;            // m/s^2
    vio_vec3_t gyro;           // rad/s
} vio_imu_msg_t;

typedef struct {
    vio_timestamp_t timestamp; // ns
    int32_t camera_id;
    int32_t width;
    int32_t height;
    int32_t stride;
    vio_pixel_format_e format;
    const uint8_t* buffer;
} vio_image_msg_t;

/* ==========================================
 * 4. 输出数据协议 (Output State)
 * ========================================== */

#define VIO_DEBUG_MAX_FEATURES 512

typedef struct {
    int32_t num_msckf;
    int32_t num_slam;
    int32_t num_aruco;
    const double* points_msckf;
    const double* points_slam;
    const double* points_aruco;
    const uint8_t* track_image;
    int32_t track_image_width;
    int32_t track_image_height;
    int32_t track_image_step;
    int32_t track_image_channels;
} vio_debug_info_t;

typedef struct {
    vio_timestamp_t timestamp;
    vio_system_status_e status;
    vio_vec3_t position;
    vio_quat_t orientation;
    vio_vec3_t velocity;
    vio_vec3_t gyro_bias;
    vio_vec3_t acc_bias;
    double position_covariance[3];
    const vio_debug_info_t* debug;
} vio_state_t;

/* ==========================================
 * 5. 配置结构 (Configuration)
 * ========================================== */

typedef struct {
    double acc_noise_density;
    double acc_random_walk;
    double gyr_noise_density;
    double gyr_random_walk;
    vio_vec3_t t_bc;
    vio_quat_t r_bc;
    bool enable_online_calibration;
    int32_t init_window_size;
    int32_t log_level;
} vio_config_t;

/* ==========================================
 * 6. API 函数 (Functions)
 * ========================================== */

typedef struct vio_instance_t* vio_handle_t;
typedef void (*vio_state_callback_t)(const vio_state_t* state, void* user_data);

vio_handle_t vio_create(const char* yaml_path);
int          vio_init(vio_handle_t handle);
void         vio_reset(vio_handle_t handle);
void         vio_destroy(vio_handle_t handle);

void vio_push_imu(vio_handle_t handle, const vio_imu_msg_t* imu);
void vio_push_image(vio_handle_t handle, const vio_image_msg_t* img);

void vio_register_state_callback(vio_handle_t handle,
                                 vio_state_callback_t callback,
                                 void* user_data);

#ifdef __cplusplus
}
#endif

#endif // VIO_INTERFACE_API_H
