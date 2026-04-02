//
// VioInterface 类实现
//
#include "system/vio_interface.h"

#include "core/VioManager.h"
#include "sim/Simulator.h"
#include "state/State.h"
#include "system/data_bridge.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/sensor_data.h"

// 调试窗口（highgui）仅在 64 位 x86 上启用；ARM 等架构编译为无操作
#if defined(__x86_64__) || defined(_M_X64) || defined(__amd64__)
#define OV_VIO_DEBUG_DISPLAY_X64 1
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#else
#define OV_VIO_DEBUG_DISPLAY_X64 0
#endif

namespace {

constexpr size_t kMaxCameraQueueSize = 30;
constexpr double SEC_TO_NS = 1e9;

}  // namespace

VioInterface::VioInterface(const std::string& yaml_path) : run_flg_(false) {
  parser_ = std::make_shared<ov_core::YamlParser>(yaml_path);
  vio_manager_options_ = std::make_shared<ov_msckf::VioManagerOptions>();
  vio_manager_options_->print_and_load(parser_);
  app_ = std::make_shared<ov_msckf::VioManager>(*vio_manager_options_);
  sim_ = nullptr;
}

VioInterface::~VioInterface() {
  run_flg_.store(false);
  camera_queue_cond_.notify_all();
  if (run_thread_.joinable()) {
    run_thread_.join();
  }
}

void VioInterface::RegisterStateCallback(vio_state_callback_t callback,
                                         void* user_data) {
  std::lock_guard<std::mutex> lck(callback_mtx_);
  state_callback_ = callback;
  state_callback_user_data_ = user_data;
}

void VioInterface::OnIMU(const vio_imu_msg_t& imu) {
  auto imu_data = ov_core::ToImuData(imu);
  {
    std::lock_guard<std::mutex> lck(vio_mtx_);
    app_->feed_measurement_imu(imu_data);
  }
  {
    std::lock_guard<std::mutex> lck(camera_queue_mtx_);
    last_imu_time_ = imu_data.timestamp;
    camera_queue_cond_.notify_one();
  }
}

void VioInterface::OnImage(const vio_image_msg_t& image) {
  auto camera_data = ov_core::ToCameraData(image);
  {
    std::lock_guard<std::mutex> lck(camera_queue_mtx_);
    // 按 track_frequency 限频，并设置容差避免边界抖动导致误丢帧。
    if (vio_manager_options_ &&
        vio_manager_options_->track_frequency > 0.0 &&
        last_accepted_cam_time_ >= 0.0) {
      const double min_dt = 1.0 / vio_manager_options_->track_frequency;
      const double tol = std::max(0.001, 0.05 * min_dt);  // max(1ms, 5%)
      const double dt = camera_data.timestamp - last_accepted_cam_time_;
      if (dt <= 0.0 || dt < (min_dt - tol)) {
        return;
      }
    }
    last_accepted_cam_time_ = camera_data.timestamp;
    camera_queue_.push_back(std::move(camera_data));
    if (camera_queue_.size() > kMaxCameraQueueSize) {
      camera_queue_.pop_front();
    }
    camera_queue_cond_.notify_one();
  }
  // OpenCV highgui：imshow/waitKey 须在主线程；由调用方保证 OnImage 在主线程执行
#if OV_VIO_DEBUG_DISPLAY_X64
  cv::Mat to_show;
  {
    std::lock_guard<std::mutex> lock(display_mtx_);
    if (!debug_display_combo_.empty()) {
      to_show = debug_display_combo_.clone();
    }
  }
  if (!to_show.empty()) {
    static bool window_initialized = false;
    if (!window_initialized) {
      const std::string window_name = "VIO: prev raw | track";
      cv::namedWindow(window_name, cv::WINDOW_NORMAL);
      cv::resizeWindow(window_name, 1280, 480);
      cv::moveWindow(window_name, 50, 50);
      cv::setWindowProperty(window_name, cv::WND_PROP_TOPMOST, 1);
      window_initialized = true;
    }
    cv::imshow("VIO: prev raw | track", to_show);
    int waitkey_ms = 1;
    if (vio_manager_options_) {
      waitkey_ms = vio_manager_options_->debug_display_waitkey_ms;
      if (waitkey_ms < 0) {
        waitkey_ms = 1;
      }
    }
    cv::waitKey(waitkey_ms);
  }
#endif
}

int VioInterface::init() {
  if (run_flg_.load(std::memory_order_relaxed)) return -1;
  run_flg_.store(true);
  run_thread_ = std::thread(&VioInterface::Run, this);
  return 0;
}

int VioInterface::reset() {
  run_flg_.store(false);
  camera_queue_cond_.notify_all();
  if (run_thread_.joinable()) {
    run_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lck(camera_queue_mtx_);
    camera_queue_.clear();
    last_accepted_cam_time_ = -1.0;
  }
  return init();
}

void VioInterface::InvokeStateCallback(const ov_core::CameraData& cam_data) {
  vio_state_callback_t cb = nullptr;
  void* user = nullptr;
  {
    std::lock_guard<std::mutex> lck(callback_mtx_);
    cb = state_callback_;
    user = state_callback_user_data_;
  }
  if (!cb) return;

  vio_state_t state_out = {};
  state_out.timestamp =
      static_cast<vio_timestamp_t>(cam_data.timestamp * SEC_TO_NS);
  state_out.status =
      app_->initialized() ? VIO_STATUS_TRACKING : VIO_STATUS_INITIALIZING;
  for (int i = 0; i < 3; ++i) state_out.position_covariance[i] = -1.0;

  if (app_->initialized()) {
    auto state = app_->get_state();
    const auto& imu = state->_imu;
    state_out.position.data[0] = imu->pos()(0);
    state_out.position.data[1] = imu->pos()(1);
    state_out.position.data[2] = imu->pos()(2);
    state_out.orientation.data[0] = imu->quat()(0);
    state_out.orientation.data[1] = imu->quat()(1);
    state_out.orientation.data[2] = imu->quat()(2);
    state_out.orientation.data[3] = imu->quat()(3);
    state_out.velocity.data[0] = imu->vel()(0);
    state_out.velocity.data[1] = imu->vel()(1);
    state_out.velocity.data[2] = imu->vel()(2);
    state_out.gyro_bias.data[0] = imu->bias_g()(0);
    state_out.gyro_bias.data[1] = imu->bias_g()(1);
    state_out.gyro_bias.data[2] = imu->bias_g()(2);
    state_out.acc_bias.data[0] = imu->bias_a()(0);
    state_out.acc_bias.data[1] = imu->bias_a()(1);
    state_out.acc_bias.data[2] = imu->bias_a()(2);
  } else {
    state_out.position.data[0] = state_out.position.data[1] =
        state_out.position.data[2] = 0.0;
    state_out.orientation.data[0] = state_out.orientation.data[1] =
        state_out.orientation.data[2] = 0.0;
    state_out.orientation.data[3] = 1.0;
    state_out.velocity.data[0] = state_out.velocity.data[1] =
        state_out.velocity.data[2] = 0.0;
    state_out.gyro_bias.data[0] = state_out.gyro_bias.data[1] =
        state_out.gyro_bias.data[2] = 0.0;
    state_out.acc_bias.data[0] = state_out.acc_bias.data[1] =
        state_out.acc_bias.data[2] = 0.0;
  }

  if (app_->initialized()) {
    FillDebugInfo(debug_info_);
    state_out.debug = &debug_info_;
  } else {
    state_out.debug = nullptr;
  }
  cb(&state_out, user);
}

void VioInterface::FillDebugInfo(vio_debug_info_t& out) {
  out = {};
  out.points_msckf = nullptr;
  out.points_slam = nullptr;
  out.points_aruco = nullptr;
  out.prev_raw_image = nullptr;
  out.prev_raw_image_width = 0;
  out.prev_raw_image_height = 0;
  out.prev_raw_image_step = 0;
  out.prev_raw_image_channels = 0;
  out.track_image = nullptr;

  auto vec_to_flat = [](const std::vector<Eigen::Vector3d>& pts,
                        std::vector<double>& buf, size_t max_n) {
    buf.clear();
    size_t n = std::min(pts.size(), max_n);
    buf.reserve(n * 3);
    for (size_t i = 0; i < n; ++i) {
      buf.push_back(pts[i](0));
      buf.push_back(pts[i](1));
      buf.push_back(pts[i](2));
    }
  };
  const size_t max_f = static_cast<size_t>(VIO_DEBUG_MAX_FEATURES);
  std::vector<Eigen::Vector3d> feats_msckf = app_->get_good_features_MSCKF();
  vec_to_flat(feats_msckf, debug_msckf_, max_f);
  out.num_msckf = static_cast<int32_t>(debug_msckf_.size() / 3);
  out.points_msckf = debug_msckf_.empty() ? nullptr : debug_msckf_.data();

  std::vector<Eigen::Vector3d> feats_slam = app_->get_features_SLAM();
  vec_to_flat(feats_slam, debug_slam_, max_f);
  out.num_slam = static_cast<int32_t>(debug_slam_.size() / 3);
  out.points_slam = debug_slam_.empty() ? nullptr : debug_slam_.data();

  std::vector<Eigen::Vector3d> feats_aruco = app_->get_features_ARUCO();
  vec_to_flat(feats_aruco, debug_aruco_, max_f);
  out.num_aruco = static_cast<int32_t>(debug_aruco_.size() / 3);
  out.points_aruco = debug_aruco_.empty() ? nullptr : debug_aruco_.data();

  if (!prev_raw_gray_.empty()) {
    out.prev_raw_image = prev_raw_gray_.data;
    out.prev_raw_image_width = prev_raw_gray_.cols;
    out.prev_raw_image_height = prev_raw_gray_.rows;
    out.prev_raw_image_step = static_cast<int32_t>(prev_raw_gray_.step[0]);
    out.prev_raw_image_channels = prev_raw_gray_.channels();
  }

  debug_track_image_ = app_->get_historical_viz_image();
  if (!debug_track_image_.empty()) {
    out.track_image = debug_track_image_.data;
    out.track_image_width = debug_track_image_.cols;
    out.track_image_height = debug_track_image_.rows;
    out.track_image_step = static_cast<int32_t>(debug_track_image_.step[0]);
    out.track_image_channels = debug_track_image_.channels();
  }
}

void VioInterface::UpdateDebugDisplayBuffer() {
#if OV_VIO_DEBUG_DISPLAY_X64
  // 在 cb() 之后调用，可安全刷新；初始化阶段也需可视化时由此取图
  debug_track_image_ = app_->get_historical_viz_image();
  cv::Mat left_bgr;
  cv::Mat right_bgr;
  if (!prev_raw_gray_.empty()) {
    if (prev_raw_gray_.channels() == 1) {
      cv::cvtColor(prev_raw_gray_, left_bgr, cv::COLOR_GRAY2BGR);
    } else {
      left_bgr = prev_raw_gray_.clone();
    }
  }
  if (!debug_track_image_.empty()) {
    if (debug_track_image_.channels() == 1) {
      cv::cvtColor(debug_track_image_, right_bgr, cv::COLOR_GRAY2BGR);
    } else {
      right_bgr = debug_track_image_.clone();
    }
  }
  if (left_bgr.empty() && right_bgr.empty()) {
    std::lock_guard<std::mutex> lock(display_mtx_);
    debug_display_combo_.release();
    return;
  }
  const int target_h = 480;
  auto resize_to_height = [target_h](cv::Mat& m) {
    if (m.empty() || m.rows == target_h) {
      return;
    }
    const double s = static_cast<double>(target_h) / static_cast<double>(m.rows);
    cv::resize(m, m, {}, s, s, cv::INTER_AREA);
  };
  resize_to_height(left_bgr);
  resize_to_height(right_bgr);
  cv::Mat combo;
  if (!left_bgr.empty() && !right_bgr.empty()) {
    cv::hconcat(left_bgr, right_bgr, combo);
  } else if (!left_bgr.empty()) {
    combo = std::move(left_bgr);
  } else {
    combo = std::move(right_bgr);
  }
  std::lock_guard<std::mutex> lock(display_mtx_);
  debug_display_combo_ = std::move(combo);
#endif
}

void VioInterface::Run() {
  while (run_flg_.load(std::memory_order_relaxed)) {
    ov_core::CameraData cam_data;
    {
      std::unique_lock<std::mutex> lck(camera_queue_mtx_);
      camera_queue_cond_.wait(lck, [this] {
        return !run_flg_.load(std::memory_order_relaxed) ||
               (!camera_queue_.empty() &&
                last_imu_time_ >= camera_queue_.front().timestamp);
      });
      if (!run_flg_.load(std::memory_order_relaxed)) {
        break;
      }
      cam_data = std::move(camera_queue_.front());
      camera_queue_.pop_front();
    }
    {
      std::lock_guard<std::mutex> lck(vio_mtx_);
      app_->feed_measurement_camera(cam_data);
      InvokeStateCallback(cam_data);
#if OV_VIO_DEBUG_DISPLAY_X64
      UpdateDebugDisplayBuffer();
#endif
      if (!cam_data.images.empty()) {
        prev_raw_gray_ = cam_data.images[0].clone();
      }
    }
  }
}
