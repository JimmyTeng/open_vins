/*
 * SS_VIO 接口实现 - 直接调用 VioInterface (C++ 实现)
 * 同时实现 ss_vio debug.h 的编译信息导出（Build Info）
 */
#include "ext/ss_vio.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <new>

#include "build_info.h"
#include "ext/ss_vio debug.h"
#include "ext/ss_vio_err.h"
#include "system/vio_interface.h"
#include "system/vio_interface_api.h"

namespace {

constexpr double SEC_TO_NS = 1e9;
constexpr double NS_TO_SEC = 1e-9;

VioInterface* g_handle = nullptr;
std::mutex g_state_mtx;

// SLAM 回调（ss_vio debug.h）
OT_VIO_SlamCallback_t g_slam_cb = nullptr;
void* g_slam_user = nullptr;
std::mutex g_slam_mtx;
static OT_VIO_PoseData g_slam_pose;
static OT_VIO_3DPoint g_slam_points[OT_VIO_SLAM_POINTS_MAX];

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

/** 在销毁 VioInterface 后清空模块级缓存，避免 DeInit/重入 Init 后残留旧位姿或 SLAM 缓冲。 */
static void reset_module_globals_after_handle_destroyed() {
  {
    std::lock_guard<std::mutex> lk(g_state_mtx);
    g_cached = CachedState{};
  }
  g_slam_pose = OT_VIO_PoseData{};
  std::memset(g_slam_points, 0, sizeof(g_slam_points));
}

static OT_VIO_PoseValidStatus map_status(vio_system_status_e s) {
  switch (s) {
    case VIO_STATUS_NOT_READY:
      return VIO_POSE_VALID_STATUS_NONE;
    case VIO_STATUS_INITIALIZING:
      return VIO_POSE_VALID_STATUS_NOT_INIT;
    case VIO_STATUS_LOST:
      return VIO_POSE_VALID_STATUS_LOST;
    case VIO_STATUS_TRACKING:
      return VIO_POSE_VALID_STATUS_SUCCESS;
    default:
      return VIO_POSE_VALID_STATUS_NONE;
  }
}

void state_callback(const vio_state_t* state, void* /*user_data*/) {
  if (!state) return;
  {
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
  /* 调用 SLAM 回调：仅在回调内使用 data，不得保存指针 */
  OT_VIO_SlamCallback_t cb = nullptr;
  void* user = nullptr;
  {
    std::lock_guard<std::mutex> lk(g_slam_mtx);
    cb = g_slam_cb;
    user = g_slam_user;
  }
  if (!cb) return;

  g_slam_pose.timestamp = static_cast<double>(state->timestamp) * NS_TO_SEC;
  g_slam_pose.x = static_cast<float>(state->position.data[0]);
  g_slam_pose.y = static_cast<float>(state->position.data[1]);
  g_slam_pose.z = static_cast<float>(state->position.data[2]);
  g_slam_pose.qx = static_cast<float>(state->orientation.data[0]);
  g_slam_pose.qy = static_cast<float>(state->orientation.data[1]);
  g_slam_pose.qz = static_cast<float>(state->orientation.data[2]);
  g_slam_pose.qw = static_cast<float>(state->orientation.data[3]);
  g_slam_pose.vx = static_cast<float>(state->velocity.data[0]);
  g_slam_pose.vy = static_cast<float>(state->velocity.data[1]);
  g_slam_pose.vz = static_cast<float>(state->velocity.data[2]);
  g_slam_pose.ox = static_cast<float>(state->gyro_bias.data[0]);
  g_slam_pose.oy = static_cast<float>(state->gyro_bias.data[1]);
  g_slam_pose.oz = static_cast<float>(state->gyro_bias.data[2]);
  g_slam_pose.ValidFlag = map_status(state->status);

  int pointNum = 0;
  if (state->debug && state->debug->points_slam && state->debug->num_slam > 0) {
    const int n = std::min(state->debug->num_slam,
                           static_cast<int32_t>(OT_VIO_SLAM_POINTS_MAX));
    const double* ps = state->debug->points_slam;
    for (int i = 0; i < n; ++i) {
      g_slam_points[i].x = static_cast<float>(ps[i * 3 + 0]);
      g_slam_points[i].y = static_cast<float>(ps[i * 3 + 1]);
      g_slam_points[i].z = static_cast<float>(ps[i * 3 + 2]);
      g_slam_points[i].ids = i;
    }
    pointNum = n;
  }

  OT_VIO_SlamCallbackData_t data = {};
  data.timestamp = g_slam_pose.timestamp;
  data.pose = g_slam_pose;
  data.pointNum = pointNum;
  data.pointSets = pointNum > 0 ? g_slam_points : nullptr;
  cb(&data, user);
}

}  // namespace

int SS_VIO_Init(const OT_VIO_Param* param) {
  if (!param) {
    return OT_VIOALG_ERR_INIT_NULL;
  }
  if (!param->calibParamPath[0]) {
    return OT_VIOALG_ERR_INIT_CHECK_CALIBPATH;
  }
  /* 已成功初始化则禁止重复 Init，须先 SS_VIO_DeInit */
  if (g_handle) {
    return OT_VIOALG_ERR_INIT_ALREADY;
  }
  reset_module_globals_after_handle_destroyed();
  g_handle = new (std::nothrow) VioInterface(param->calibParamPath);
  if (g_handle == nullptr) {
    return OT_VIOALG_ERR_INIT;
  }
  g_handle->RegisterStateCallback(state_callback, nullptr);
  int ret = g_handle->init();
  if (ret != 0) {
    delete g_handle;
    g_handle = nullptr;
    reset_module_globals_after_handle_destroyed();
    return OT_VIOALG_ERR_INIT_START;
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
  SS_VIO_UnregisterSlamCallback();
  if (g_handle) {
    delete g_handle;
    g_handle = nullptr;
  }
  reset_module_globals_after_handle_destroyed();
  return 0;
}

int SS_VIO_RegisterSlamCallback(OT_VIO_SlamCallback_t callback,
                                void* user_data) {
  std::lock_guard<std::mutex> lk(g_slam_mtx);
  g_slam_cb = callback;
  g_slam_user = user_data;
  return 0;
}

void SS_VIO_UnregisterSlamCallback(void) {
  std::lock_guard<std::mutex> lk(g_slam_mtx);
  g_slam_cb = nullptr;
  g_slam_user = nullptr;
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
    vio_imu.timestamp =
        static_cast<vio_timestamp_t>(src->timestamp * SEC_TO_NS);
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

// -----------------------------------------------------------------------------
// Build Info（ss_vio debug.h）
// -----------------------------------------------------------------------------
namespace {

void build_info_copy_str(char* dst, size_t dst_size, const char* src) {
  if (!dst || dst_size == 0) return;
  size_t n = strlen(src);
  if (n >= dst_size) n = dst_size - 1;
  memcpy(dst, src, n);
  dst[n] = '\0';
}

}  // namespace

extern "C" {

int SS_VIO_GetBuildInfo(SS_VIO_BuildInfo* out) {
  if (!out) return -1;
  build_info_copy_str(out->build_time, sizeof(out->build_time),
                      BUILD_TIMESTAMP);
  build_info_copy_str(out->build_timezone, sizeof(out->build_timezone),
                      BUILD_TIMEZONE);
  build_info_copy_str(out->git_commit, sizeof(out->git_commit),
                      BUILD_GIT_COMMIT);
  build_info_copy_str(out->git_hash, sizeof(out->git_hash), BUILD_GIT_HASH);
  build_info_copy_str(out->git_branch, sizeof(out->git_branch),
                      BUILD_GIT_BRANCH);
  build_info_copy_str(out->git_tag, sizeof(out->git_tag), BUILD_GIT_TAG);
  build_info_copy_str(out->git_status, sizeof(out->git_status),
                      BUILD_GIT_DIRTY);
  build_info_copy_str(out->git_user, sizeof(out->git_user), BUILD_GIT_USER);
  build_info_copy_str(out->git_email, sizeof(out->git_email), BUILD_GIT_EMAIL);
  return 0;
}

int SS_VIO_FormatBuildInfo(char* buffer, size_t size) {
  char tmp[512];
  int n = snprintf(tmp, sizeof(tmp),
                   "\n========== Build Info ==========\n"
                   "  Build Time:   %s %s\n"
                   "  Git Commit:   %s (%s)\n"
                   "  Git Branch:   %s\n"
                   "  Git Tag:      %s\n"
                   "  Git Status:   %s\n"
                   "  Git User:     %s <%s>\n"
                   "=================================\n",
                   BUILD_TIMESTAMP, BUILD_TIMEZONE, BUILD_GIT_COMMIT,
                   BUILD_GIT_HASH, BUILD_GIT_BRANCH, BUILD_GIT_TAG,
                   BUILD_GIT_DIRTY, BUILD_GIT_USER, BUILD_GIT_EMAIL);
  if (n < 0) return -1;
  size_t len = static_cast<size_t>(n);
  if (buffer && size > 0) {
    size_t copy = len < size ? len : size - 1;
    memcpy(buffer, tmp, copy);
    buffer[copy] = '\0';
  }
  return static_cast<int>(len);
}

}  // extern "C"
