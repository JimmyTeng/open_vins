//
// VIO 接口头文件 - C API + 内部类声明（实现见 vio_interface.cpp / vio_interface_api.cpp）
//

#ifndef VIO_INTERFACE_H
#define VIO_INTERFACE_H

#include "system/vio_interface_api.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "utils/sensor_data.h"

namespace ov_core {
class YamlParser;
}
namespace ov_msckf {
class VioManager;
struct VioManagerOptions;
class Simulator;
}

/**
 * 内部 C++ 实现类，由 vio_interface_api.cpp 的 C API 包装调用。
 */
class VioInterface {
public:
    explicit VioInterface(const std::string& yaml_path);
    ~VioInterface();

    void RegisterStateCallback(vio_state_callback_t callback, void* user_data);
    void OnIMU(const vio_imu_msg_t& imu);
    /** 仅 x86_64 且配置 debug_display_imshow 时在此函数内 imshow（须由主线程调用） */
    void OnImage(const vio_image_msg_t& image);
    int init();
    int reset();

private:
    void InvokeStateCallback(const ov_core::CameraData& cam_data);
    void FillDebugInfo(vio_debug_info_t& out);
    /** 在工作线程中仅更新共享缓冲，不调用 imshow/waitKey */
    void UpdateDebugDisplayBuffer();
    void Run();

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
    double last_accepted_cam_time_{-1.0};
    vio_debug_info_t debug_info_{};
    std::vector<double> debug_msckf_;
    std::vector<double> debug_slam_;
    std::vector<double> debug_aruco_;
    cv::Mat debug_track_image_;
    /** 上一帧已送入 VioManager 的灰度图，供回调中 prev_raw 与调试显示 */
    cv::Mat prev_raw_gray_;
    std::mutex display_mtx_;
    cv::Mat debug_display_combo_;
};

#endif // VIO_INTERFACE_H
