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

#ifndef OV_MSCKF_ROS1VISUALIZER_H
#define OV_MSCKF_ROS1VISUALIZER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>

#include <Eigen/Eigen>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>

namespace ov_core {
class YamlParser;
struct CameraData;
} // namespace ov_core

namespace ov_msckf {

class VioManager;
class Simulator;

/**
 * @brief Helper class that will publish results onto the ROS framework.
 * @brief 辅助类，用于将结果发布到ROS框架中
 *
 * Also save to file the current total state and covariance along with the groundtruth if we are simulating.
 * 如果正在仿真，还会将当前总状态和协方差以及真值保存到文件
 * We visualize the following things:
 * 我们可视化以下内容：
 * - State of the system on TF, pose message, and path
 *   系统状态（TF变换、位姿消息和路径）
 * - Image of our tracker
 *   跟踪器图像
 * - Our different features (SLAM, MSCKF, ARUCO)
 *   不同的特征点（SLAM、MSCKF、ARUCO）
 * - Groundtruth trajectory if we have it
 *   真值轨迹（如果有）
 */
class ROS1Visualizer {

public:
  /**
   * @brief Default constructor
   * @brief 默认构造函数
   * @param nh ROS node handler
   * @param nh ROS节点句柄
   * @param app Core estimator manager
   * @param app 核心估计器管理器
   * @param sim Simulator if we are simulating
   * @param sim 仿真器（如果正在仿真）
   */
  ROS1Visualizer(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<VioManager> app, std::shared_ptr<Simulator> sim = nullptr);

  /**
   * @brief Will setup ROS subscribers and callbacks
   * @brief 设置ROS订阅者和回调函数
   * @param parser Configuration file parser
   * @param parser 配置文件解析器
   */
  void setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser);

  /**
   * @brief Will visualize the system if we have new things
   * @brief 如果有新的数据，将可视化系统状态
   */
  void visualize();

  /**
   * @brief Will publish our odometry message for the current timestep.
   * @brief 发布当前时间步的里程计消息
   * This will take the current state estimate and get the propagated pose to the desired time.
   * 这将获取当前状态估计，并将位姿传播到所需时间
   * This can be used to get pose estimates on systems which require high frequency pose estimates.
   * 这可用于需要高频位姿估计的系统
   */
  void visualize_odometry(double timestamp);

  /**
   * @brief After the run has ended, print results
   * @brief 运行结束后，打印结果
   */
  void visualize_final();

  /// Callback for inertial information
  /// 惯性信息回调函数
  void callback_inertial(const sensor_msgs::Imu::ConstPtr &msg);

  /// Callback for monocular cameras information
  /// 单目相机信息回调函数
  void callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0);

  /// Callback for synchronized stereo camera information
  /// 同步双目相机信息回调函数
  void callback_stereo(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, int cam_id0, int cam_id1);

protected:
  /// Publish the current state
  /// 发布当前状态
  void publish_state();

  /// Publish the active tracking image
  /// 发布活动跟踪图像
  void publish_images();

  /// Publish current features
  /// 发布当前特征点
  void publish_features();

  /// Publish groundtruth (if we have it)
  /// 发布真值（如果有）
  void publish_groundtruth();

  /// Publish loop-closure information of current pose and active track information
  /// 发布当前位姿和活动轨迹信息的回环闭合信息
  void publish_loopclosure_information();

  /// Global node handler
  /// 全局节点句柄
  std::shared_ptr<ros::NodeHandle> _nh;

  /// Core application of the filter system
  /// 滤波器系统的核心应用
  std::shared_ptr<VioManager> _app;

  /// Simulator (is nullptr if we are not sim'ing)
  /// 仿真器（如果不进行仿真则为nullptr）
  std::shared_ptr<Simulator> _sim;

  // Our publishers
  // 发布者
  image_transport::Publisher it_pub_tracks, it_pub_loop_img_depth, it_pub_loop_img_depth_color; // 图像发布者：跟踪图像、回环深度图、彩色深度图
  ros::Publisher pub_poseimu, pub_odomimu, pub_pathimu; // IMU位姿、里程计、路径发布者
  ros::Publisher pub_points_msckf, pub_points_slam, pub_points_aruco, pub_points_sim; // 特征点发布者：MSCKF、SLAM、ARUCO、仿真
  ros::Publisher pub_loop_pose, pub_loop_point, pub_loop_extrinsic, pub_loop_intrinsics; // 回环信息发布者：位姿、点云、外参、内参
  std::shared_ptr<tf::TransformBroadcaster> mTfBr; // TF变换广播器

  // Our subscribers and camera synchronizers
  // 订阅者和相机同步器
  ros::Subscriber sub_imu; // IMU订阅者
  std::vector<ros::Subscriber> subs_cam; // 相机订阅者列表
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol; // 近似时间同步策略
  std::vector<std::shared_ptr<message_filters::Synchronizer<sync_pol>>> sync_cam; // 相机同步器列表
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>> sync_subs_cam; // 同步相机订阅者列表

  // For path viz
  // 用于路径可视化
  unsigned int poses_seq_imu = 0; // IMU位姿序列号
  std::vector<geometry_msgs::PoseStamped> poses_imu; // IMU位姿列表

  // Groundtruth infomation
  // 真值信息
  ros::Publisher pub_pathgt, pub_posegt; // 真值路径和位姿发布者
  double summed_mse_ori = 0.0; // 累积方向均方误差
  double summed_mse_pos = 0.0; // 累积位置均方误差
  double summed_nees_ori = 0.0; // 累积方向归一化估计误差平方
  double summed_nees_pos = 0.0; // 累积位置归一化估计误差平方
  size_t summed_number = 0; // 累积数量

  // Start and end timestamps
  // 开始和结束时间戳
  bool start_time_set = false; // 开始时间是否已设置
  boost::posix_time::ptime rT1, rT2; // 开始和结束时间点

  // Thread atomics
  // 线程原子变量
  std::atomic<bool> thread_update_running; // 更新线程是否正在运行

  /// Queue up camera measurements sorted by time and trigger once we have
  /// exactly one IMU measurement with timestamp newer than the camera measurement
  /// This also handles out-of-order camera measurements, which is rare, but
  /// a nice feature to have for general robustness to bad camera drivers.
  /// 按时间排序的相机测量队列，当有一个时间戳比相机测量更新的IMU测量时触发
  /// 这也处理乱序的相机测量，虽然很少见，但对于处理不良相机驱动程序的鲁棒性很有用
  std::deque<ov_core::CameraData> camera_queue; // 相机数据队列
  std::mutex camera_queue_mtx; // 相机队列互斥锁

  // Last camera message timestamps we have received (mapped by cam id)
  // 最后接收到的相机消息时间戳（按相机ID映射）
  std::map<int, double> camera_last_timestamp;

  // Last timestamp we visualized at
  // 最后可视化的时间戳
  double last_visualization_timestamp = 0; // 最后可视化状态的时间戳
  double last_visualization_timestamp_image = 0; // 最后可视化图像的时间戳

  // Our groundtruth states
  // 真值状态
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states; // 真值状态映射（时间戳 -> 状态向量）

  // For path viz
  // 用于路径可视化
  unsigned int poses_seq_gt = 0; // 真值位姿序列号
  std::vector<geometry_msgs::PoseStamped> poses_gt; // 真值位姿列表
  bool publish_global2imu_tf = true; // 是否发布全局到IMU的TF变换
  bool publish_calibration_tf = true; // 是否发布标定TF变换

  // Files and if we should save total state
  // 文件以及是否应该保存总状态
  bool save_total_state = false; // 是否保存总状态
  std::ofstream of_state_est, of_state_std, of_state_gt; // 状态估计、标准差、真值文件流
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS1VISUALIZER_H
