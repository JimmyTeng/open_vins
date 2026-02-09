//
// Created by tengz on 2026/2/9.
//
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include "vio_interface.h"
#include "data_bridge.h"
#include "utils/opencv_yaml_parse.h"
#include "core/VioManager.h"
#include "sim/Simulator.h"
#include "utils/sensor_data.h"

class VioInterface;
namespace {

constexpr size_t kMaxCameraQueueSize = 30;

}  // namespace

class VioInterface {
public:
    explicit VioInterface(const std::string& yaml_path) {
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
            }
        }
    }

    std::atomic<bool> run_flg_;
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
    (void)handle;
    (void)callback;
    (void)user_data;
    // TODO: 集成到 VioInterface 的状态输出
}
