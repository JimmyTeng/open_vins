/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H

#include <Eigen/StdVector>
#include <algorithm>
#include <atomic>
#include <boost/filesystem.hpp>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include "VioManagerOptions.h"

namespace ov_core {
struct ImuData;
struct CameraData;
class TrackBase;
class FeatureInitializer;
} // namespace ov_core
namespace ov_init {
class InertialInitializer;
} // namespace ov_init

namespace ov_msckf {

class State;
class StateHelper;
class UpdaterMSCKF;
class UpdaterSLAM;
class UpdaterZeroVelocity;
class Propagator;

/**
 * @brief 管理整个VIO系统的核心类
 * 
 * 该类包含MSCKF所需的状态和其他算法。
 * 我们将测量数据输入到该类中，并将它们发送到相应的算法。
 * 如果有需要传播或更新的测量数据，该类将调用状态对象来执行这些操作。
 * 
 * @brief Core class that manages the entire system
 *
 * This class contains the state and other algorithms needed for the MSCKF to work.
 * We feed in measurements into this class and send them to their respective algorithms.
 * If we have measurements to propagate or update with, this class will call on our state to do that.
 */
class VioManager {

public:
  /**
   * @brief 默认构造函数，将加载所有配置变量
   * @param params_ 从ROS或命令行加载的参数
   * 
   * @brief Default constructor, will load all configuration variables
   * @param params_ Parameters loaded from either ROS or CMDLINE
   */
  VioManager(VioManagerOptions &params_);

  /**
   * @brief 输入IMU惯性测量数据
   * @param message 包含时间戳和惯性信息的消息
   * 
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   */
  void feed_measurement_imu(const ov_core::ImuData &message);

  /**
   * @brief 输入相机测量数据
   * @param message 包含时间戳、图像和相机ID的消息
   * 
   * @brief Feed function for camera measurements
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_measurement_camera(const ov_core::CameraData &message) { track_image_and_update(message); }

  /**
   * @brief 输入同步仿真相机的测量数据
   * @param timestamp 图像采集的时间戳
   * @param camids 有仿真测量数据的相机ID列表
   * @param feats 原始UV坐标的仿真测量数据
   * 
   * @brief Feed function for a synchronized simulated cameras
   * @param timestamp Time that this image was collected
   * @param camids Camera ids that we have simulated measurements for
   * @param feats Raw uv simulated measurements
   */
  void feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                   const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

  /**
   * @brief 根据给定的状态初始化IMU状态（使用真值）
   * @param imustate MSCKF顺序的状态向量: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   *                 其中：time为时间戳，q_GtoI为全局到IMU的旋转四元数，p_IinG为IMU在全局坐标系中的位置，
   *                 v_IinG为IMU在全局坐标系中的速度，b_gyro为陀螺仪偏差，b_accel为加速度计偏差
   * 
   * @brief Given a state, this will initialize our IMU state.
   * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   */
  void initialize_with_gt(Eigen::Matrix<double, 17, 1> imustate);

  /// 检查系统是否已初始化
  /// If we are initialized or not
  bool initialized() { return is_initialized_vio && timelastupdate != -1; }

  /// 返回系统初始化的时间戳
  /// Timestamp that the system was initialized at
  double initialized_time() { return startup_time; }

  /// 获取当前系统参数的访问器
  /// Accessor for current system parameters
  VioManagerOptions get_params() { return params; }

  /// 获取当前状态的访问器
  /// Accessor to get the current state
  std::shared_ptr<State> get_state() { return state; }

  /// 获取当前状态传播器的访问器
  /// Accessor to get the current propagator
  std::shared_ptr<Propagator> get_propagator() { return propagator; }

  /// 获取跟踪特征的可视化图像
  /// Get a nice visualization image of what tracks we have
  cv::Mat get_historical_viz_image();

  /// 返回全局坐标系中的3D SLAM特征点
  /// Returns 3d SLAM features in the global frame
  std::vector<Eigen::Vector3d> get_features_SLAM();

  /// 返回全局坐标系中的3D ARUCO特征点
  /// Returns 3d ARUCO features in the global frame
  std::vector<Eigen::Vector3d> get_features_ARUCO();

  /// 返回上次更新中使用的全局坐标系中的3D特征点
  /// Returns 3d features used in the last update in global frame
  std::vector<Eigen::Vector3d> get_good_features_MSCKF() { return good_features_MSCKF; }

  /// 返回投影活动跟踪时使用的图像
  /// Return the image used when projecting the active tracks
  void get_active_image(double &timestamp, cv::Mat &image) {
    timestamp = active_tracks_time;
    image = active_image;
  }

  /// 返回当前帧中活动跟踪的特征点
  /// Returns active tracked features in the current frame
  void get_active_tracks(double &timestamp, std::unordered_map<size_t, Eigen::Vector3d> &feat_posinG,
                         std::unordered_map<size_t, Eigen::Vector3d> &feat_tracks_uvd) {
    timestamp = active_tracks_time;
    feat_posinG = active_tracks_posinG;
    feat_tracks_uvd = active_tracks_uvd;
  }

protected:
  /**
   * @brief 对新的相机图像进行跟踪
   *
   * 如果使用双目跟踪，将调用双目跟踪函数。
   * 否则将在每个传入的图像上进行跟踪。
   *
   * @param message 包含时间戳、图像和相机ID的消息
   * 
   * @brief Given a new set of camera images, this will track them.
   *
   * If we are having stereo tracking, we should call stereo tracking functions.
   * Otherwise we will try to track on each of the images passed.
   *
   * @param message Contains our timestamp, images, and camera ids
   */
  void track_image_and_update(const ov_core::CameraData &message);

  /**
   * @brief 执行状态传播和特征更新
   * @param message 包含时间戳、图像和相机ID的消息
   * 
   * @brief This will do the propagation and feature updates to the state
   * @param message Contains our timestamp, images, and camera ids
   */
  void do_feature_propagate_update(const ov_core::CameraData &message);

  /**
   * @brief 尝试初始化系统状态
   *
   * 该函数将调用初始化器并尝试初始化状态。
   * 未来应该从这里调用运动恢复结构（Structure-from-Motion）代码。
   * 该函数也可以用于在系统失败后重新初始化。
   *
   * @param message 包含时间戳、图像和相机ID的消息
   * @return 如果成功初始化则返回true
   * 
   * @brief This function will try to initialize the state.
   *
   * This should call on our initializer and try to init the state.
   * In the future we should call the structure-from-motion code from here.
   * This function could also be repurposed to re-initialize the system after failure.
   *
   * @param message Contains our timestamp, images, and camera ids
   * @return True if we have successfully initialized
   */
  bool try_to_initialize(const ov_core::CameraData &message);

  /**
   * @brief 重新三角化当前帧中的所有特征点
   *
   * 对于系统当前正在跟踪的所有特征点，将重新进行三角化。
   * 这对于需要当前点云的下游应用很有用（例如回环检测）。
   * 该函数将尝试三角化*所有*点，而不仅仅是更新中使用的点。
   *
   * @param message 包含时间戳、图像和相机ID的消息
   * 
   * @brief This function will will re-triangulate all features in the current frame
   *
   * For all features that are currently being tracked by the system, this will re-triangulate them.
   * This is useful for downstream applications which need the current pointcloud of points (e.g. loop closure).
   * This will try to triangulate *all* points, not just ones that have been used in the update.
   *
   * @param message Contains our timestamp, images, and camera ids
   */
  void retriangulate_active_tracks(const ov_core::CameraData &message);

  /// 是否打印调试信息
  /// If we should print debug information
  bool print_debug = false;

  /// 管理器参数配置
  /// Manager parameters
  VioManagerOptions params;

  /// 主状态对象
  /// Our master state object :D
  std::shared_ptr<State> state;

  /// 状态传播器
  /// Propagator of our state
  std::shared_ptr<Propagator> propagator;

  /// 稀疏特征跟踪器（KLT或描述符匹配）
  /// Our sparse feature tracker (klt or descriptor)
  std::shared_ptr<ov_core::TrackBase> trackFEATS;

  /// ArUco标签跟踪器
  /// Our aruoc tracker
  std::shared_ptr<ov_core::TrackBase> trackARUCO;

  /// 状态初始化器
  /// State initializer
  std::shared_ptr<ov_init::InertialInitializer> initializer;

  /// 系统是否已初始化的布尔标志
  /// Boolean if we are initialized or not
  bool is_initialized_vio = false;

  /// MSCKF特征更新器
  /// Our MSCKF feature updater
  std::shared_ptr<UpdaterMSCKF> updaterMSCKF;

  /// SLAM/ARUCO特征更新器
  /// Our SLAM/ARUCO feature updater
  std::shared_ptr<UpdaterSLAM> updaterSLAM;

  /// 零速度更新器
  /// Our zero velocity tracker
  std::shared_ptr<UpdaterZeroVelocity> updaterZUPT;

  /// 自开始初始化以来收到的测量时间队列
  /// 初始化后，我们希望快速传播和更新到最新时间戳
  /// This is the queue of measurement times that have come in since we starting doing initialization
  /// After we initialize, we will want to prop & update to the latest timestamp quickly
  std::vector<double> camera_queue_init;
  std::mutex camera_queue_init_mtx;

  // 时间统计文件和变量
  // Timing statistic file and variables
  std::ofstream of_statistics;
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  // 跟踪已行驶的距离
  // Track how much distance we have traveled
  double timelastupdate = -1;
  double distance = 0;

  // 滤波器启动时间
  // Startup time of the filter
  double startup_time = -1;

  // 线程及其原子变量
  // Threads and their atomics
  std::atomic<bool> thread_init_running, thread_init_success;

  // 是否执行了零速度更新
  // If we did a zero velocity update
  bool did_zupt_update = false;
  bool has_moved_since_zupt = false;

  // 上次更新中使用的好特征点（用于可视化）
  // Good features that where used in the last update (used in visualization)
  std::vector<Eigen::Vector3d> good_features_MSCKF;

  // 从当前帧看到的重新三角化特征点的3D位置（用于可视化）
  // 对于每个特征点，我们创建一个线性系统 A * p_FinG = b 并递增其代价
  // Re-triangulated features 3d positions seen from the current frame (used in visualization)
  // For each feature we have a linear system A * p_FinG = b we create and increment their costs
  double active_tracks_time = -1;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_posinG;  // 活动跟踪特征点在全局坐标系中的位置
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;     // 活动跟踪特征点的UV坐标和深度
  cv::Mat active_image;                                              // 活动图像
  std::map<size_t, Eigen::Matrix3d> active_feat_linsys_A;           // 活动特征线性系统的A矩阵
  std::map<size_t, Eigen::Vector3d> active_feat_linsys_b;           // 活动特征线性系统的b向量
  std::map<size_t, int> active_feat_linsys_count;                    // 活动特征线性系统的观测计数
};

} // namespace ov_msckf

#endif // OV_MSCKF_VIOMANAGER_H
