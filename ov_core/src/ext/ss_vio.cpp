/*
 * SS_VIO 接口实现 - 直接调用 VioInterface (C++ 实现)
 */
#include "ext/ss_vio.h"
#include "ext/ss_vio_err.h"
#include "system/vio_interface.h"

#include <mutex>
#include <new>

namespace {

constexpr double SEC_TO_NS = 1e9;
constexpr double NS_TO_SEC = 1e-9;

VioInterface* g_handle = nullptr;
std::mutex g_state_mtx;

// 最新状态缓存（回调内指针仅短期有效，故缓存标量）
struct CachedState {
    bool has_data = false;
    double timestamp_ns = 0.0;
    vio_system_status_e status = VIO_STATUS_NOT_READY;
    double position[3] = {0, 0, 0};
    double orientation[4] = {0, 0, 0, 1};  // x,y,z,w
    double velocity[3] = {0, 0, 0};
    double gyro_bias[3] = {0, 0, 0};
    double acc_bias[3] = {0, 0, 0};
} g_cached;

void state_callback(const vio_state_t* state, void* /*user_data*/) {
    if (!state) return;
    std::lock_guard<std::mutex> lk(g_state_mtx);
    g_cached.has_data = true;
    g_cached.timestamp_ns = static_cast<double>(state->timestamp);
    g_cached.status = state->status;
    for (int i = 0; i < 3; ++i) {
        g_cached.position[i] = state->position.data[i];
        g_cached.velocity[i] = state->velocity.data[i];
        g_cached.gyro_bias[i] = state->gyro_bias.data[i];
        g_cached.acc_bias[i] = state->acc_bias.data[i];
    }
    for (int i = 0; i < 4; ++i) {
        g_cached.orientation[i] = state->orientation.data[i];
    }
}

OT_VIO_PoseValidStatus map_status(vio_system_status_e s) {
    switch (s) {
        case VIO_STATUS_NOT_READY:    return VIO_POSE_VALID_STATUS_NONE;
        case VIO_STATUS_INITIALIZING: return VIO_POSE_VALID_STATUS_NOT_INIT;
        case VIO_STATUS_LOST:         return VIO_POSE_VALID_STATUS_LOST;
        case VIO_STATUS_TRACKING:      return VIO_POSE_VALID_STATUS_SUCCESS;
        default:                      return VIO_POSE_VALID_STATUS_NONE;
    }
}

}  // namespace

int SS_VIO_Init(const OT_VIO_Param* param) {
    if (!param) {
        return OT_VIOALG_ERR_INIT_NULL;
    }
    if (!param->calibParamPath[0]) {
        return OT_VIOALG_ERR_INIT_CHECK_CALIBPATH;
    }
    if (g_handle) {
        SS_VIO_DeInit();
    }
    g_handle = new (std::nothrow) VioInterface(param->calibParamPath);
    if (g_handle == nullptr) {
        return OT_VIOALG_ERR_INIT;
    }
    g_handle->RegisterStateCallback(state_callback, nullptr);
    int ret = g_handle->init();
    if (ret != 0) {
        delete g_handle;
        g_handle = nullptr;
        return OT_VIOALG_ERR_INIT_START;
    }
    {
        std::lock_guard<std::mutex> lk(g_state_mtx);
        g_cached.has_data = false;
    }
    return 0;
}

int SS_VIO_GetData(const double /*timestamp*/, OT_VIO_PoseData* data) {
    if (!data) {
        return OT_VIOALG_ERR_GETDATA_NULL;
    }
    if (!g_handle) {
        return OT_VIOALG_ERR_GET_POSE;
    }
    std::lock_guard<std::mutex> lk(g_state_mtx);
    if (!g_cached.has_data) {
        data->ValidFlag = VIO_POSE_VALID_STATUS_NONE;
        data->timestamp = 0.0;
        return 0;
    }
    data->timestamp = g_cached.timestamp_ns * NS_TO_SEC;
    data->x = static_cast<float>(g_cached.position[0]);
    data->y = static_cast<float>(g_cached.position[1]);
    data->z = static_cast<float>(g_cached.position[2]);
    data->qx = static_cast<float>(g_cached.orientation[0]);
    data->qy = static_cast<float>(g_cached.orientation[1]);
    data->qz = static_cast<float>(g_cached.orientation[2]);
    data->qw = static_cast<float>(g_cached.orientation[3]);
    data->vx = static_cast<float>(g_cached.velocity[0]);
    data->vy = static_cast<float>(g_cached.velocity[1]);
    data->vz = static_cast<float>(g_cached.velocity[2]);
    data->ox = static_cast<float>(g_cached.gyro_bias[0]);
    data->oy = static_cast<float>(g_cached.gyro_bias[1]);
    data->oz = static_cast<float>(g_cached.gyro_bias[2]);
    data->ValidFlag = map_status(g_cached.status);
    return 0;
}

int SS_VIO_DeInit(void) {
    if (g_handle) {
        delete g_handle;
        g_handle = nullptr;
    }
    return 0;
}

int SS_VIO_PushImageData(const OT_VIO_CameraData* camData) {
    if (!g_handle) {
        return OT_VIOALG_ERR_PUSH_IMAGE;
    }
    if (!camData) {
        return OT_VIOALG_ERR_IMAGE_NULL;
    }
    const OT_VIO_ImageData* img = &camData->leftImage;
    if (!img->virtAddr) {
        return OT_VIOALG_ERR_IMAGE_NULL;
    }
    if (img->width <= 0 || img->height <= 0) {
        return OT_VIOALG_ERR_IMAGE_WIDTH;  // or HEIGHT
    }
    if (img->stride <= 0) {
        return OT_VIOALG_ERR_IMAGE_STRIDE;
    }
    vio_image_msg_t vio_img = {};
    vio_img.timestamp = static_cast<vio_timestamp_t>(img->timestamp * SEC_TO_NS);
    vio_img.camera_id = 0;
    vio_img.width = static_cast<int32_t>(img->width);
    vio_img.height = static_cast<int32_t>(img->height);
    vio_img.stride = static_cast<int32_t>(img->stride);
    vio_img.format = VIO_PIXEL_FMT_GRAY8;
    vio_img.buffer = img->virtAddr;
    g_handle->OnImage(vio_img);
    return 0;
}

int SS_VIO_PushImuData(const OT_VIO_ImuDataInfo* imuDataInfo) {
    if (!g_handle) {
        return OT_VIOALG_ERR_PUSH_IMU_ACC;  // generic push error
    }
    if (!imuDataInfo) {
        return OT_VIOALG_ERR_IMU_NULL;
    }
    if (imuDataInfo->num > OT_VIO_IMU_DATA_NUM_MAX) {
        return OT_VIOALG_ERR_IMU_TIME;
    }
    vio_imu_msg_t vio_imu = {};
    for (unsigned int i = 0; i < imuDataInfo->num; ++i) {
        const OT_VIO_ImuData* src = &imuDataInfo->imuDatas[i];
        vio_imu.timestamp = static_cast<vio_timestamp_t>(src->timestamp * SEC_TO_NS);
        vio_imu.acc.data[0] = static_cast<double>(src->accX);
        vio_imu.acc.data[1] = static_cast<double>(src->accY);
        vio_imu.acc.data[2] = static_cast<double>(src->accZ);
        vio_imu.gyro.data[0] = static_cast<double>(src->gyroX);
        vio_imu.gyro.data[1] = static_cast<double>(src->gyroY);
        vio_imu.gyro.data[2] = static_cast<double>(src->gyroZ);
        g_handle->OnIMU(vio_imu);
    }
    return 0;
}
