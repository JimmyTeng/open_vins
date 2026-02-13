//
// Created by tengz on 2026/2/9.
//
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "vio_interface.h"
#include "data_bridge.h"
#include "utils/opencv_yaml_parse.h"
#include "core/VioManager.h"
#include "state/State.h"
#include "sim/Simulator.h"
#include "utils/sensor_data.h"

class VioInterface;
namespace {

constexpr size_t kMaxCameraQueueSize = 30;
constexpr double SEC_TO_NS = 1e9;

}  // namespace

class VioInterface {
public:
    explicit VioInterface(const std::string& yaml_path) : run_flg_(false) {
        parser_ = std::make_shared<ov_core::YamlParser>(yaml_path);
        vio_manager_options_ = std::make_shared<ov_msckf::VioManagerOptions>();
        vio_manager_options_->print_and_load(parser_);
        app_ = std::make_shared<ov_msckf::VioManager>(*vio_manager_options_);
        sim_ = std::make_shared<ov_msckf::Simulator>(*vio_manager_options_);

    }

    ~VioInterface() {
        run_flg_.store(false);
        camera_queue_cond_.notify_all();
        if (run_thread_.joinable()) {
            run_thread_.join();
        }
    }

    void RegisterStateCallback(vio_state_callback_t callback, void* user_data) {
        std::lock_guard<std::mutex> lck(callback_mtx_);
        state_callback_ = callback;
        state_callback_user_data_ = user_data;
    }

    void OnIMU(const vio_imu_msg_t& imu) {
        auto imu_data = ov_yuv_parser::ToImuData(imu);
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

    void OnImage(const vio_image_msg_t& image) {
        auto camera_data = ov_yuv_parser::ToCameraData(image);  // 耗时操作在锁外完成
        std::lock_guard<std::mutex> lck(camera_queue_mtx_);
        camera_queue_.push_back(std::move(camera_data));
        if (camera_queue_.size() > kMaxCameraQueueSize) {
            camera_queue_.pop_front();  // 队列溢出时丢弃最旧帧
        }
        camera_queue_cond_.notify_one();
    }

    int init() {
        if (run_flg_.load(std::memory_order_relaxed)) return -1;
        run_flg_.store(true);
        run_thread_ = std::thread(&VioInterface::Run, this);
        return 0;   
    }

    int reset() {
        run_flg_.store(false);
        camera_queue_cond_.notify_all();
        if (run_thread_.joinable()) {
            run_thread_.join();
        }
        return init();
    }
private:
    void InvokeStateCallback(const ov_core::CameraData& cam_data) {
        vio_state_callback_t cb = nullptr;
        void* user = nullptr;
        {
            std::lock_guard<std::mutex> lck(callback_mtx_);
            cb = state_callback_;
            user = state_callback_user_data_;
        }
        if (!cb) return;

        vio_state_t state_out = {};
        state_out.timestamp = static_cast<vio_timestamp_t>(cam_data.timestamp * SEC_TO_NS);
        state_out.status = app_->initialized() ? VIO_STATUS_TRACKING : VIO_STATUS_INITIALIZING;
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
            state_out.position.data[0] = state_out.position.data[1] = state_out.position.data[2] = 0.0;
            state_out.orientation.data[0] = state_out.orientation.data[1] = state_out.orientation.data[2] = 0.0;
            state_out.orientation.data[3] = 1.0;
            state_out.velocity.data[0] = state_out.velocity.data[1] = state_out.velocity.data[2] = 0.0;
            state_out.gyro_bias.data[0] = state_out.gyro_bias.data[1] = state_out.gyro_bias.data[2] = 0.0;
            state_out.acc_bias.data[0] = state_out.acc_bias.data[1] = state_out.acc_bias.data[2] = 0.0;
        }

        if (app_->initialized()) {
            FillDebugInfo(debug_info_);
            state_out.debug = &debug_info_;
        } else {
            state_out.debug = nullptr;
        }
        cb(&state_out, user);
    }

    void FillDebugInfo(vio_debug_info_t& out) {
        out = {};
        out.points_msckf = nullptr;
        out.points_slam = nullptr;
        out.points_aruco = nullptr;
        out.track_image = nullptr;

        auto vec_to_flat = [](const std::vector<Eigen::Vector3d>& pts, std::vector<double>& buf, size_t max_n) {
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

        debug_track_image_ = app_->get_historical_viz_image();
        if (!debug_track_image_.empty()) {
            out.track_image = debug_track_image_.data;
            out.track_image_width = debug_track_image_.cols;
            out.track_image_height = debug_track_image_.rows;
            out.track_image_step = static_cast<int32_t>(debug_track_image_.step[0]);
            out.track_image_channels = debug_track_image_.channels();
        }
    }

    void Run() {
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
            }
        }
    }

    std::atomic<bool> run_flg_;
    std::mutex callback_mtx_;
    vio_state_callback_t state_callback_{nullptr};
    void* state_callback_user_data_{nullptr};
    std::thread run_thread_;
    std::mutex vio_mtx_;
    std::mutex camera_queue_mtx_;
    std::condition_variable camera_queue_cond_;
    std::shared_ptr<ov_core::YamlParser> parser_;
    std::shared_ptr<ov_msckf::VioManagerOptions> vio_manager_options_;
    std::shared_ptr<ov_msckf::VioManager> app_;
    std::shared_ptr<ov_msckf::Simulator> sim_;
    std::deque<ov_core::CameraData> camera_queue_;
    double last_imu_time_{0.0};
    vio_debug_info_t debug_info_{};
    std::vector<double> debug_msckf_;
    std::vector<double> debug_slam_;
    std::vector<double> debug_aruco_;
    cv::Mat debug_track_image_;
};

vio_handle_t vio_create(const char* yaml_path) {
    auto* ptr = new VioInterface(yaml_path ? std::string(yaml_path) : std::string());
    return reinterpret_cast<vio_handle_t>(ptr);
}
int vio_init(vio_handle_t handle) {
    auto* ptr = reinterpret_cast<VioInterface*>(handle);
    return ptr->init();
}

void vio_reset(vio_handle_t handle) {
    auto* ptr = reinterpret_cast<VioInterface*>(handle);
    ptr->reset();
}

void vio_destroy(vio_handle_t handle) {
    auto* ptr = reinterpret_cast<VioInterface*>(handle);
    delete ptr;
}

void vio_push_imu(vio_handle_t handle, const vio_imu_msg_t* imu) {
    if (handle && imu) {
        reinterpret_cast<VioInterface*>(handle)->OnIMU(*imu);
    }
}

void vio_push_image(vio_handle_t handle, const vio_image_msg_t* img) {
    if (handle && img) {
        reinterpret_cast<VioInterface*>(handle)->OnImage(*img);
    }
}

void vio_register_state_callback(vio_handle_t handle,
                                 vio_state_callback_t callback,
                                 void* user_data) {
    if (handle && callback) {
        reinterpret_cast<VioInterface*>(handle)->RegisterStateCallback(callback, user_data);
    }
}
