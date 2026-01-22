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

#include "VioManager.h"

#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureInitializer.h"
#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/opencv_lambda_body.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include "init/InertialInitializer.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterZeroVelocity.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

// 用于跟踪统计的静态变量
// Static variables for tracking statistics
static size_t num_features_before_tracking = 0;
static size_t num_features_after_tracking = 0;

/**
 * VioManager构造函数 - 初始化VIO系统
 * @param params_ VIO管理器配置参数
 */
VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false) {

  // 启动信息
  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS 流形上的EKF正在启动\n");
  PRINT_DEBUG("=======================================\n");

  // 保存参数并打印配置信息
  // Nice debug
  this->params = params_;
  params.print_and_load_estimator();
  params.print_and_load_noise();
  params.print_and_load_state();
  params.print_and_load_trackers();

  // 全局设置OpenCV使用的线程数
  // -1将重置为系统默认线程数（通常是核心数）
  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  // cv::setNumThreads(params.num_opencv_threads);
  cv::setNumThreads(1);
  
  cv::setRNGSeed(0);

  // 创建状态对象
  // Create the state!!
  state = std::make_shared<State>(params.state_options);

  // 设置IMU内参（陀螺仪和加速度计的偏差、标定参数等）
  // Set the IMU intrinsics
  state->_calib_imu_dw->set_value(params.vec_dw);
  state->_calib_imu_dw->set_fej(params.vec_dw);
  state->_calib_imu_da->set_value(params.vec_da);
  state->_calib_imu_da->set_fej(params.vec_da);
  state->_calib_imu_tg->set_value(params.vec_tg);
  state->_calib_imu_tg->set_fej(params.vec_tg);
  state->_calib_imu_GYROtoIMU->set_value(params.q_GYROtoIMU);
  state->_calib_imu_GYROtoIMU->set_fej(params.q_GYROtoIMU);
  state->_calib_imu_ACCtoIMU->set_value(params.q_ACCtoIMU);
  state->_calib_imu_ACCtoIMU->set_fej(params.q_ACCtoIMU);

  // 设置相机到IMU的时间偏移
  // Timeoffset from camera to IMU
  Eigen::VectorXd temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = params.calib_camimu_dt;
  state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // 遍历并加载每个相机的内参和外参
  // Loop through and load each of the cameras
  state->_cam_intrinsics_cameras = params.camera_intrinsics;
  for (int i = 0; i < state->_options.num_cameras; i++) {
    state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i)->get_value());
    state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i)->get_value());
    state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
    state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // 如果启用统计记录，则打开统计文件
  // If we are recording statistics, then open our file
  if (params.record_timing_information) {
    // 如果文件已存在，则删除它
    // If the file exists, then delete it
    if (boost::filesystem::exists(params.record_timing_filepath)) {
      boost::filesystem::remove(params.record_timing_filepath);
      PRINT_INFO(YELLOW "[统计]: 发现旧文件，已删除...\n" RESET);
    }
    // 创建文件所在目录
    // Create the directory that we will open the file in
    boost::filesystem::path p(params.record_timing_filepath);
    boost::filesystem::create_directories(p.parent_path());
    // 打开统计文件
    // Open our statistics file!
    of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
    // 写入表头信息
    // Write the header information into it
    of_statistics << "# timestamp (sec),tracking,propagation,msckf update,";
    if (state->_options.max_slam_features > 0) {
      of_statistics << "slam update,slam delayed,";
    }
    of_statistics << "re-tri & marg,total" << std::endl;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // 创建特征提取器
  // 注意：初始化后我们将增加特征跟踪的总数
  // 注意：我们将在所有相机之间均匀分配特征总数
  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  // NOTE: we will split the total number of features over all cameras uniformly
  int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
  // 很奇怪的设定，应该是设置每个相机的特征点数量
  if (params.use_klt) {
    // 使用KLT（Kanade-Lucas-Tomasi）跟踪器
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(state->_cam_intrinsics_cameras, init_max_features,
                                                         state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
                                                         params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist));
  } else {
    // 使用描述符跟踪器
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->_cam_intrinsics_cameras, init_max_features, state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
        params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist, params.knn_ratio));
  }

  // 初始化ArUco标签提取器
  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(state->_cam_intrinsics_cameras, state->_options.max_aruco_features,
                                                           params.use_stereo, params.histogram_method, params.downsize_aruco));
  }

  // 初始化状态传播器（用于IMU积分和状态传播）
  // Initialize our state propagator
  propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // 初始化状态初始化器
  // Our state initialize
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

  // 创建更新器
  // Make the updater!
  updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options, params.featinit_options);
  updaterSLAM = std::make_shared<UpdaterSLAM>(params.slam_options, params.aruco_options, params.featinit_options);

  // 如果使用零速度更新，则创建零速度更新器
  // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                        propagator, params.gravity_mag, params.zupt_max_velocity,
                                                        params.zupt_noise_multiplier, params.zupt_max_disparity);
  }
}

/**
 * 输入IMU测量数据
 * @param message IMU数据消息
 */
void VioManager::feed_measurement_imu(const ov_core::ImuData &message) {

  // 我们需要的最旧IMU时间是最后一个克隆状态的时间
  // 我们不应该需要整个窗口，但如果时间倒退，我们就会需要
  // The oldest time we need IMU with is the last clone
  // We shouldn't really need the whole window, but if we go backwards in time we will
  double oldest_time = state->margtimestep();
  if (oldest_time > state->_timestamp) {
    oldest_time = -1;
  }
  // 如果未初始化，使用初始化窗口时间
  if (!is_initialized_vio) {
    oldest_time = message.timestamp - params.init_options.init_window_time + state->_calib_dt_CAMtoIMU->value()(0) - 0.10;
  }
  // 将IMU数据传递给传播器
  propagator->feed_imu(message, oldest_time);

  // 如果未初始化，将IMU数据传递给初始化器
  // Push back to our initializer
  if (!is_initialized_vio) {
    initializer->feed_imu(message, oldest_time);
  }

  // 如果启用了零速度更新器，将IMU数据传递给它
  // 如果只在开始时进行零速度更新且已经移动过，则不需要传递
  // Push back to the zero velocity updater if it is enabled
  // No need to push back if we are just doing the zv-update at the begining and we have moved
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    updaterZUPT->feed_imu(message, oldest_time);
  }
}

/**
 * 输入仿真测量数据
 * @param timestamp 时间戳
 * @param camids 相机ID列表
 * @param feats 特征点数据（特征ID和像素坐标）
 */
void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                             const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats) {

  // 开始计时
  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 检查我们是否真的有仿真跟踪器
  // 如果没有，重新创建并将跟踪器转换为仿真跟踪器
  // Check if we actually have a simulated tracker
  // If not, recreate and re-cast the tracker to our simulation tracker
  std::shared_ptr<TrackSIM> trackSIM = std::dynamic_pointer_cast<TrackSIM>(trackFEATS);
  if (trackSIM == nullptr) {
    // 替换为仿真跟踪器
    // Replace with the simulated tracker
    trackSIM = std::make_shared<TrackSIM>(state->_cam_intrinsics_cameras, state->_options.max_aruco_features);
    trackFEATS = trackSIM;
    // 也需要在初始化和零速度更新器中替换，因为它们指向trackFEATS的数据库指针
    // Need to also replace it in init and zv-upt since it points to the trackFEATS db pointer
    initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());
    if (params.try_zupt) {
      updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                          propagator, params.gravity_mag, params.zupt_max_velocity,
                                                          params.zupt_noise_multiplier, params.zupt_max_disparity);
    }
    PRINT_WARNING(RED "[仿真]: 将跟踪器转换为TrackSIM对象！\n" RESET);
  }

  // 将仿真数据传递给仿真跟踪器
  // Feed our simulation tracker
  trackSIM->feed_measurement_simulation(timestamp, camids, feats);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // 检查是否应该进行零速度更新，如果是则更新状态
  // 注意：如果只在初始化阶段使用，且已经移动过，则不应该尝试零速度更新
  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // 如果状态时间相同，使用上一个时间步的决策
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, timestamp);
    }
    if (did_zupt_update) {
      assert(state->_timestamp == timestamp);
      propagator->clean_old_imu_measurements(timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      propagator->invalidate_cache();
      return;
    }
  }

  // 如果没有VIO初始化，则返回错误
  // If we do not have VIO initialization, then return an error
  if (!is_initialized_vio) {
    PRINT_ERROR(RED "[仿真]: 在仿真特征之前，您的VIO系统应该已经初始化！！！\n" RESET);
    PRINT_ERROR(RED "[仿真]: 在调用feed_measurement_simulation()之前，请先初始化您的系统！！！！\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // 调用传播和更新函数
  // 仿真要么全部同步，要么是单相机...
  // Call on our propagate and update function
  // Simulation is either all sync, or single camera...
  ov_core::CameraData message;
  message.timestamp = timestamp;
  for (auto const &camid : camids) {
    int width = state->_cam_intrinsics_cameras.at(camid)->w();
    int height = state->_cam_intrinsics_cameras.at(camid)->h();
    message.sensor_ids.push_back(camid);
    message.images.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
  }
  do_feature_propagate_update(message);
}

/**
 * 跟踪图像并更新状态
 * @param message_const 相机数据消息（包含图像和时间戳）
 */
void VioManager::track_image_and_update(const ov_core::CameraData &message_const) {

  // 开始计时
  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 打印时间戳信息用于调试（特别是在初始化期间）
  // Print timestamp information for debugging (especially during initialization)
  static int processed_image_count = 0;
  static double last_processed_timestamp = 0.0;
  processed_image_count++;
  double time_interval = 0.0;
  if (last_processed_timestamp > 0.0) {
    time_interval = message_const.timestamp - last_processed_timestamp;
  }
  last_processed_timestamp = message_const.timestamp;
  
  // 在初始化阶段或每20帧打印一次
  // Print during initialization phase or every 20th image
  bool should_print = (!is_initialized_vio) || (processed_image_count % 20 == 0);
  if (should_print) {
    PRINT_INFO(CYAN "[VM] 帧 #%d : %.6f s, 间隔: %.3f ms (%.2f Hz), Init: %d\n" RESET,
               processed_image_count, message_const.timestamp, time_interval * 1000.0,
               (time_interval > 0 ? 1.0/time_interval : 0.0), (int)is_initialized_vio);
  }

  // 断言我们有有效的测量数据和ID
  // Assert we have valid measurement data and ids
  assert(!message_const.sensor_ids.empty());
  assert(message_const.sensor_ids.size() == message_const.images.size());
  for (size_t i = 0; i < message_const.sensor_ids.size() - 1; i++) {
    assert(message_const.sensor_ids.at(i) != message_const.sensor_ids.at(i + 1));
  }

  // 如果启用下采样，则对图像进行下采样
  // Downsample if we are downsampling
  ov_core::CameraData message = message_const;
  for (size_t i = 0; i < message.sensor_ids.size() && params.downsample_cameras; i++) {
    cv::Mat img = message.images.at(i);
    cv::Mat mask = message.masks.at(i);
    cv::Mat img_temp, mask_temp;
    cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
    message.images.at(i) = img_temp;
    cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
    message.masks.at(i) = mask_temp;
  }

  // 执行特征跟踪！
  // 获取跟踪前的特征数量（用于跟踪统计）
  // Perform our feature tracking!
  // Get feature count before tracking (for tracking statistics)
  num_features_before_tracking = trackFEATS->get_feature_database()->size();
  // 获得目前还在跟踪的特征点坐标
  // Get coordinates of currently tracked features
  // Lambda函数：收集当前帧中被跟踪特征的UV坐标
  // Lambda function: collect UV coordinates of tracked features in current frame
  auto collect_tracked_feature_uvs = [&message](const std::shared_ptr<ov_core::FeatureDatabase> &feat_db) -> std::vector<std::pair<size_t, Eigen::Vector2f>> {
    std::vector<std::pair<size_t, Eigen::Vector2f>> result; // (feature id, uv coordinates)
    auto features_map = feat_db->get_internal_data();
    for (const auto &pair : features_map) {
      const auto &feat_ptr = pair.second;
      if (!feat_ptr->to_delete) {
        // 对于每个被跟踪的特征，找到该特征在本帧的UV坐标
        // For each tracked feature, find its UV coordinates in current frame
        for (size_t cam_idx = 0; cam_idx < message.sensor_ids.size(); ++cam_idx) {
          size_t camid = message.sensor_ids[cam_idx];
          auto it_uvs = feat_ptr->uvs.find(camid);
          auto it_times = feat_ptr->timestamps.find(camid);
          if (it_uvs != feat_ptr->uvs.end() && it_times != feat_ptr->timestamps.end()) {
            const std::vector<double>& ts_vec = it_times->second;
            const std::vector<Eigen::VectorXf>& uvs_vec = it_uvs->second;
            for (size_t j = 0; j < ts_vec.size(); ++j) {
              if (ts_vec[j] == message.timestamp && uvs_vec[j].size() >= 2) {
                Eigen::Vector2f uv(uvs_vec[j][0], uvs_vec[j][1]);
                result.emplace_back(feat_ptr->featid, uv);
                break;
              }
            }
          }
        }
      }
    }
    return result;
  };
  
  std::vector<std::pair<size_t, Eigen::Vector2f>> tracked_feature_uvs_before = collect_tracked_feature_uvs(trackFEATS->get_feature_database());
  trackFEATS->feed_new_camera(message);
  num_features_after_tracking = trackFEATS->get_feature_database()->size();
  
  std::vector<std::pair<size_t, Eigen::Vector2f>> tracked_feature_uvs_after = collect_tracked_feature_uvs(trackFEATS->get_feature_database());

  

  // 获取丢失的特征用于跟踪统计
  // 使用消息时间戳查找未跟踪到当前帧的特征
  // Get lost features for tracking statistics
  // Use message timestamp to find features that were not tracked into this frame
  std::vector<std::shared_ptr<Feature>> feats_lost_temp = 
      trackFEATS->get_feature_database()->features_not_containing_newer(message.timestamp, false, true);
  size_t num_features_lost_this_frame = feats_lost_temp.size();
  
  // 打印跟踪统计信息用于调试
  // Print tracking statistics for debugging
  static int tracking_frame_count = 0;
  tracking_frame_count++;
  long num_features_new = (long)num_features_after_tracking - (long)num_features_before_tracking;
  long net_change = (long)num_features_after_tracking - (long)num_features_before_tracking;
  
  // 在初始化阶段或每20帧打印一次
  // Print during initialization phase or every 20th frame
  bool should_print_tracking = (!is_initialized_vio) || (tracking_frame_count % 20 == 0);
  if (should_print_tracking) {
    PRINT_INFO(CYAN "[VM] 帧 #%d - 总数: %zu, 丢失: %zu, 新增: %ld, 净变化: %ld\n" RESET,
               tracking_frame_count, num_features_after_tracking, num_features_lost_this_frame, 
               num_features_new, net_change);
  }

  // 如果ArUco跟踪器可用，也将数据传递给它
  // 注意：ArUco的双目跟踪没有意义，因为我们默认有ID
  // 注意：因此如果进行双目跟踪，我们只调用立体跟踪
  // If the aruco tracker is available, the also pass to it
  // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
  // NOTE: thus we just call the stereo tracking if we are doing binocular!
  if (is_initialized_vio && trackARUCO != nullptr) {
    trackARUCO->feed_new_camera(message);
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // 检查是否应该进行零速度更新，如果是则更新状态
  // 注意：如果只在初始化阶段使用，且已经移动过，则不应该尝试零速度更新
  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // 如果状态时间相同，使用上一个时间步的决策
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != message.timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, message.timestamp);
    }
    if (did_zupt_update) {
      assert(state->_timestamp == message.timestamp);
      propagator->clean_old_imu_measurements(message.timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(message.timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      propagator->invalidate_cache();
      return;
    }
  }

  // 如果没有VIO初始化，则尝试初始化
  // TODO: 或者如果我们试图重置系统，则在这里执行
  // If we do not have VIO initialization, then try to initialize
  // TODO: Or if we are trying to reset the system, then do that here!
  if (!is_initialized_vio) {
    is_initialized_vio = try_to_initialize(message);
    if (!is_initialized_vio) {
      double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
      PRINT_DEBUG(BLUE "[VM]: 跟踪耗时 %.4f 秒\n" RESET, time_track);
      return;
    }
  }

  // 调用传播和更新函数
  // Call on our propagate and update function
  do_feature_propagate_update(message);
}

/**
 * 执行特征传播和状态更新
 * 这是VIO系统的核心函数，负责状态传播、特征管理和EKF更新
 * @param message 相机数据消息
 */
void VioManager::do_feature_propagate_update(const ov_core::CameraData &message) {

  //===================================================================================
  // 状态传播和克隆状态增强
  // State propagation, and clone augmentation
  //===================================================================================

  // Return if the camera measurement is out of order
  if (state->_timestamp > message.timestamp) {
    PRINT_WARNING(YELLOW "图像接收顺序错误，无法处理 (传播时间差 = %3f)\n" RESET,
                  (message.timestamp - state->_timestamp));
    return;
  }

  // 将状态传播到当前更新时间
  // 同时用新的克隆状态增强状态向量
  // 注意：如果状态已经在给定时间（可能在仿真中发生），则不需要传播
  // Propagate the state forward to the current update time
  // Also augment it with a new clone!
  // NOTE: if the state is already at the given time (can happen in sim)
  // NOTE: then no need to prop since we already are at the desired timestep
  if (state->_timestamp != message.timestamp) {
    propagator->propagate_and_clone(state, message.timestamp);
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // 如果还没有达到最大克隆数，我们应该返回...
  // 这不是很理想，但这使得后续逻辑更容易...
  // 当我们至少有5个克隆状态时，我们可以开始处理，因为我们可以开始三角化...
  // If we have not reached max clones, we should just return...
  // This isn't super ideal, but it keeps the logic after this easier...
  // We can start processing things when we have at least 5 clones since we can start triangulating things...
  if ((int)state->_clones_IMU.size() < std::min(state->_options.max_clone_size, 5)) {
    PRINT_DEBUG("等待足够的克隆状态 (%d / %d)....\n", (int)state->_clones_IMU.size(),
                std::min(state->_options.max_clone_size, 5));
    return;
  }

  // 如果无法传播，则返回
  // Return if we where unable to propagate
  if (state->_timestamp != message.timestamp) {
    PRINT_WARNING(RED "[传播]: 传播器无法将状态向前传播到指定时间！\n" RESET);
    PRINT_WARNING(RED "[传播]: 距离上次传播已经过了 %.3f 秒\n" RESET, message.timestamp - state->_timestamp);
    return;
  }
  has_moved_since_zupt = true;

  //===================================================================================
  // MSCKF特征和作为SLAM特征的KLT跟踪
  // MSCKF features and KLT tracks that are SLAM features
  //===================================================================================

  // 现在，让我们获取所有应该用于更新但在最新帧中丢失的特征
  // 我们明确请求在另一个更新步骤中未被删除（使用）的特征
  // Now, lets get all features that should be used for an update that are lost in the newest frame
  // We explicitly request features that have not been deleted (used) in another update step
  std::vector<std::shared_ptr<Feature>> feats_lost, feats_marg, feats_slam;

  // feats_lost 跟踪丢失的特征
  feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->_timestamp, false, true);
  
  // 打印跟踪统计信息用于调试
  // Print tracking statistics for debugging
  static int tracking_update_count = 0;
  tracking_update_count++;
  size_t num_features_current = trackFEATS->get_feature_database()->size();
  size_t num_features_lost = feats_lost.size();
  
  // 计算新特征和净变化（使用跟踪步骤中的静态变量）
  // Calculate new features and net change (using static variables from tracking step)
  long num_features_new = (long)num_features_after_tracking - (long)num_features_before_tracking;
  long net_change = (long)num_features_after_tracking - (long)num_features_before_tracking;
  
  // 在初始化阶段或每20帧打印一次
  // Print during initialization phase or every 20th frame
  bool should_print_tracking = (!is_initialized_vio) || (tracking_update_count % 20 == 0);
  if (should_print_tracking) {
    PRINT_INFO(CYAN "[跟踪] 更新 #%d - 总特征数: %zu, 丢失: %zu, 新增: %ld, 净变化: %ld\n" RESET,
               tracking_update_count, num_features_current, num_features_lost, num_features_new, net_change);
  }

  // 在达到最大克隆数之前，不需要获取最旧的特征
  // Don't need to get the oldest features until we reach our max number of clones
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size || (int)state->_clones_IMU.size() > 5) {
    // 注释：当克隆数量超过最大数量时，触发对最旧时间步的特征获取（边缘化时用）
    feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep(), false, true);
    //trackARUCO 信息需要被保留下来
    if (trackARUCO != nullptr && message.timestamp - startup_time >= params.dt_slam_delay) {
      feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep(), false, true);
    }
  }

  // 移除来自其他图像流的丢失特征
  // 例如：如果我们是cam1，而cam0还没有处理，我们不想在更新中使用那些特征
  // 例如：因此我们等到cam0处理其最新图像后再移除从该相机看到的特征
  // Remove any lost features that were from other image streams
  // E.g: if we are cam1 and cam0 has not processed yet, we don't want to try to use those in the update yet
  // E.g: thus we wait until cam0 process its newest image to remove features which were seen from that camera
  auto it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    bool found_current_message_camid = false;
    for (const auto &camuvpair : (*it1)->uvs) {
      if (std::find(message.sensor_ids.begin(), message.sensor_ids.end(), camuvpair.first) != message.sensor_ids.end()) {
        found_current_message_camid = true;
        break;
      }
    }
    if (found_current_message_camid) {
      it1++;
    } else {
      it1 = feats_lost.erase(it1);
    }
  }

  // 我们还需要确保最大跟踪不包含任何丢失的特征
  // 如果特征在最后一帧丢失，但在边缘化时间步有测量，则可能发生这种情况
  // We also need to make sure that the max tracks does not contain any lost features
  // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
  it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    if (std::find(feats_marg.begin(), feats_marg.end(), (*it1)) != feats_marg.end()) {
      // PRINT_WARNING(YELLOW "FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!\n" RESET);
      it1 = feats_lost.erase(it1);
    } else {
      it1++;
    }
  }

  // 查找已达到最大长度的跟踪，这些可以转换为SLAM特征
  // Find tracks that have reached max length, these can be made into SLAM features
  std::vector<std::shared_ptr<Feature>> feats_maxtracks;
  auto it2 = feats_marg.begin();
  while (it2 != feats_marg.end()) {
    // 查看我们的任何相机是否达到最大跟踪
    // See if any of our camera's reached max track
    bool reached_max = false;
    for (const auto &cams : (*it2)->timestamps) {
      if ((int)cams.second.size() > state->_options.max_clone_size) {
        reached_max = true;
        break;
      }
    }
    // 如果达到最大跟踪，则将其添加到可能的SLAM特征列表
    // If max track, then add it to our possible slam feature list
    if (reached_max) {
      feats_maxtracks.push_back(*it2);
      it2 = feats_marg.erase(it2);
    } else {
      it2++;
    }
  }

  // 计算状态中有多少个ArUco标签
  // Count how many aruco tags we have in our state
  int curr_aruco_tags = 0;
  auto it0 = state->_features_SLAM.begin();
  while (it0 != state->_features_SLAM.end()) {
    if ((int)(*it0).second->_featid <= 4 * state->_options.max_aruco_features)
      curr_aruco_tags++;
    it0++;
  }

  // 如果有空间，则追加新的SLAM特征
  // 同时检查是否已满足延迟时间要求（用于避免初始化阶段SLAM特征点质量不佳的问题）
  // Append a new SLAM feature if we have the room to do so
  // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
  if (state->_options.max_slam_features > 0 && //需要做SLAM 特征
      message.timestamp - startup_time >= params.dt_slam_delay && //等待延迟时间
      (int)state->_features_SLAM.size() < state->_options.max_slam_features + curr_aruco_tags) {  //特征点数目超过限制
    // 获取要添加的总数，然后根据边缘化特征数组获取可以添加的最大数量
    // Get the total amount to add, then the max amount that we can add given our marginalize feature array
    int amount_to_add = (state->_options.max_slam_features + curr_aruco_tags) - (int)state->_features_SLAM.size();
    int valid_amount = (amount_to_add > (int)feats_maxtracks.size()) ? (int)feats_maxtracks.size() : amount_to_add;
    // 如果我们至少有一个可以添加，让我们添加它！
    // 注意：我们从feat_marg数组中移除它们，因为我们不想重用信息...
    // 目前从尾部取特征依赖于特征数据库的返回顺序，可能不够可靠
    // If we have at least 1 that we can add, lets add it!
    // Note: we remove them from the feat_marg array since we don't want to reuse information...
    if (valid_amount > 0) {
      feats_slam.insert(feats_slam.end(), feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
      feats_maxtracks.erase(feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
    }
  }

  // 遍历当前的SLAM特征，我们有它们的跟踪，为这次更新获取它们！
  // 注意：如果我们有一个丢失跟踪的SLAM特征，我们应该将其边缘化
  // 注意：我们只在当前相机消息是特征被看到的地方时才强制执行此操作
  // 注意：如果你不使用FEJ，这些类型的SLAM特征会*降低*估计器性能....//TOASK:为什么？
  // 注意：如果SLAM特征连续几次更新失败，我们也会将其边缘化 这里是特征跟踪策略，特征跟踪策略决定了特征的寿命 可以加入图像质量评价
  // Loop through current SLAM features, we have tracks of them, grab them for this update!
  // NOTE: if we have a slam feature that has lost tracking, then we should marginalize it out
  // NOTE: we only enforce this if the current camera message is where the feature was seen from
  // NOTE: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
  // NOTE: we will also marginalize SLAM features if they have failed their update a couple times in a row
  for (std::pair<const size_t, std::shared_ptr<Landmark>> &landmark : state->_features_SLAM) {
    if (trackARUCO != nullptr) {
      std::shared_ptr<Feature> feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
      if (feat1 != nullptr)
        feats_slam.push_back(feat1);
    }
    std::shared_ptr<Feature> feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
    if (feat2 != nullptr)
      feats_slam.push_back(feat2);
    assert(landmark.second->_unique_camera_id != -1);
    bool current_unique_cam =
        std::find(message.sensor_ids.begin(), message.sensor_ids.end(), landmark.second->_unique_camera_id) != message.sensor_ids.end();
    // 改进的边缘化策略：
    // 1. 使用连续丢失帧数而不是单次丢失（避免偶尔遮挡误判）
    // 2. 根据运动状态调整阈值（静止时更宽容，运动时更严格）
    // 3. 结合更新失败次数综合判断
    // TODO: 更好的策略建议：
    //    - 添加连续丢失帧数计数器（如：tracking_fail_count）
    //    - 静止状态（has_moved_since_zupt=false）：连续丢失N帧（如5-10帧）才边缘化
    //    - 运动状态：连续丢失M帧（如2-3帧）就边缘化
    //    - 考虑特征历史质量（跟踪成功率、chi2历史等）
    //    - 使用时间窗口而非帧数（如：丢失超过X秒才边缘化）
    if (feat2 == nullptr && current_unique_cam) {
      // 当前策略：单次丢失立即边缘化（可能过于激进）
      // 如果特征点没有被跟踪到，则边缘化
      landmark.second->should_marg = true;
    }
    if (landmark.second->update_fail_count > 1) {
      // 如果特征点连续几次更新失败（chi2检验失败），则边缘化
      landmark.second->should_marg = true;
    }
  }

  // 让我们在这里边缘化所有旧的SLAM特征
  // 这些是未成功跟踪到当前帧的特征
  // 我们*不*边缘化ArUco标签地标
  // Lets marginalize out all old SLAM features here
  // These are ones that where not successfully tracked into the current frame
  // We do *NOT* marginalize out our aruco tags landmarks
  StateHelper::marginalize_slam(state);

  // 将SLAM特征分为新的和旧的 //TOASK
  // Separate our SLAM features into new ones, and old ones
  std::vector<std::shared_ptr<Feature>> feats_slam_DELAYED, feats_slam_UPDATE;
  for (size_t i = 0; i < feats_slam.size(); i++) {
    if (state->_features_SLAM.find(feats_slam.at(i)->featid) != state->_features_SLAM.end()) {
      feats_slam_UPDATE.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: found old feature %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    } else {
      feats_slam_DELAYED.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: new feature ready %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    }
  }

  // 连接MSCKF特征数组（即不用于SLAM更新的特征） 
  // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
  std::vector<std::shared_ptr<Feature>> featsup_MSCKF = feats_lost;
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());

  //===================================================================================
  // 现在我们有了特征列表，让我们为MSCKF和SLAM执行EKF更新！
  // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
  //===================================================================================

  // 根据跟踪长度排序
  // TODO: 我们应该有更好的选择逻辑（例如视场中的均匀特征分布等）
  // TODO: 现在"丢失"的特征在这个向量的前面，而末尾的是长跟踪
  // Sort based on track length
  // TODO: we should have better selection logic here (i.e. even feature distribution in the FOV etc..)
  // TODO: right now features that are "lost" are at the front of this vector, while ones at the end are long-tracks
  auto compare_feat = [](const std::shared_ptr<Feature> &a, const std::shared_ptr<Feature> &b) -> bool {
    size_t asize = 0;
    size_t bsize = 0;
    for (const auto &pair : a->timestamps)
      asize += pair.second.size();
    for (const auto &pair : b->timestamps)
      bsize += pair.second.size();
    return asize < bsize;
  };
  std::sort(featsup_MSCKF.begin(), featsup_MSCKF.end(), compare_feat);

  // 将它们传递给MSCKF更新器
  // 注意：如果我们有超过最大值的特征，我们为这次更新选择"最佳"的（即最大跟踪）
  // 注意：这应该只在你想跟踪大量特征或计算资源有限时使用
  // Pass them to our MSCKF updater
  // NOTE: if we have more then the max, we select the "best" ones (i.e. max tracks) for this update
  // NOTE: this should only really be used if you want to track a lot of features, or have limited computational resources
  if ((int)featsup_MSCKF.size() > state->_options.max_msckf_in_update)
    featsup_MSCKF.erase(featsup_MSCKF.begin(), featsup_MSCKF.end() - state->_options.max_msckf_in_update);
  updaterMSCKF->update(state, featsup_MSCKF);
  propagator->invalidate_cache();
  rT4 = boost::posix_time::microsec_clock::local_time();

  // 执行SLAM延迟初始化和更新
  // 注意：我们在这里提供执行*顺序*更新的选项
  // 注意：这会快得多，但不会那么准确
  // Perform SLAM delay init and update
  // NOTE: that we provide the option here to do a *sequential* update
  // NOTE: this will be a lot faster but won't be as accurate.
  std::vector<std::shared_ptr<Feature>> feats_slam_UPDATE_TEMP;
  while (!feats_slam_UPDATE.empty()) {
    // 获取我们将用于更新的特征子向量
    // Get sub vector of the features we will update with
    std::vector<std::shared_ptr<Feature>> featsup_TEMP;
    featsup_TEMP.insert(featsup_TEMP.begin(), feats_slam_UPDATE.begin(),
                        feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
    feats_slam_UPDATE.erase(feats_slam_UPDATE.begin(),
                            feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
    // 执行更新
    // Do the update
    updaterSLAM->update(state, featsup_TEMP);
    feats_slam_UPDATE_TEMP.insert(feats_slam_UPDATE_TEMP.end(), featsup_TEMP.begin(), featsup_TEMP.end());
    propagator->invalidate_cache();
  }
  feats_slam_UPDATE = feats_slam_UPDATE_TEMP;
  rT5 = boost::posix_time::microsec_clock::local_time();
  // 执行延迟初始化（对于新的SLAM特征）
  updaterSLAM->delayed_init(state, feats_slam_DELAYED);
  rT6 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // 更新可视化特征集，并清理旧特征
  // Update our visualization feature set, and clean up the old features
  //===================================================================================

  // 重新三角化当前帧中的所有当前跟踪
  // Re-triangulate all current tracks in the current frame
  if (message.sensor_ids.at(0) == 0) {

    // 重新三角化特征
    // Re-triangulate features
    retriangulate_active_tracks(message);

    // 仅在基础相机上清除MSCKF特征
    // 因此我们应该能够可视化其他唯一相机流的MSCKF特征，因为它们也会被追加到向量中
    // Clear the MSCKF features only on the base camera
    // Thus we should be able to visualize the other unique camera stream
    // MSCKF features as they will also be appended to the vector
    good_features_MSCKF.clear();
  }

  // 保存更新中使用的所有MSCKF特征
  // Save all the MSCKF features used in the update
  for (auto const &feat : featsup_MSCKF) {
    good_features_MSCKF.push_back(feat->p_FinG);
    feat->to_delete = true;
  }

  //===================================================================================
  // 清理，边缘化我们不再需要的内容...
  // Cleanup, marginalize out what we don't need any more...
  //===================================================================================

  // 从提取器中移除在上一个时间步用于更新的特征
  // 这允许如果这次未能使用测量，将来可以使用它们
  // 注意我们需要在输入新图像之前执行此操作，因为我们希望所有新测量不被删除
  // Remove features that where used for the update from our extractors at the last timestep
  // This allows for measurements to be used in the future if they failed to be used this time
  // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
  trackFEATS->get_feature_database()->cleanup();
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup();
  }

  // 如果我们将要失去一个锚点姿态，首先执行锚点更改
  // 锚点的意义：
  // 1. 对于"相对表示"（ANCHORED_*）的SLAM特征，它们的位置不是用全局坐标表示的，而是相对于某个"锚点相机姿态"表示的
  // 2. 锚点通常是特征第一次被观测到的相机姿态，特征的位置存储在锚点相机坐标系中（或相对于锚点的表示，如逆深度）
  // 3. 使用锚点的好处：
  //    - 避免全局坐标的不可观测性问题（特别是全局yaw角）
  //    - 在相机坐标系中特征通常有正的深度，数值更稳定
  //    - 可以使用更紧凑的表示（如单逆深度）减少状态维度
  // 4. 为什么需要锚点更改：
  //    - 滑动窗口会定期边缘化最旧的姿态以控制状态大小
  //    - 如果某个特征的锚点正好是要被边缘化的姿态，该特征就会失去其参考系
  //    - 因此必须在边缘化之前，将锚点切换到仍然在状态中的较新姿态
  //    - 这需要重新计算特征在新锚点下的表示，并正确传播协方差矩阵
  // First do anchor change if we are about to lose an anchor pose
  // Anchor meaning:
  // 1. For "relative representation" (ANCHORED_*) SLAM features, their positions are not represented in global coordinates,
  //    but relative to some "anchor camera pose"
  // 2. The anchor is usually the camera pose where the feature was first observed, and the feature position is stored in the
  //    anchor camera frame (or relative representation like inverse depth)
  // 3. Benefits of using anchors:
  //    - Avoids unobservability issues with global coordinates (especially global yaw)
  //    - Features in camera frame usually have positive depth, more numerically stable
  //    - Can use more compact representations (e.g., single inverse depth) to reduce state dimension
  // 4. Why anchor change is needed:
  //    - Sliding window periodically marginalizes oldest poses to control state size
  //    - If a feature's anchor is exactly the pose to be marginalized, the feature loses its reference frame
  //    - Therefore, must switch anchor to a newer pose still in the state before marginalization
  //    - This requires recomputing the feature representation in the new anchor frame and correctly propagating covariance
  updaterSLAM->change_anchors(state);

  // 清理比边缘化时间更旧的特征
  // Cleanup any features older than the marginalization time
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
    trackFEATS->get_feature_database()->cleanup_measurements(state->margtimestep());
    if (trackARUCO != nullptr) {
      trackARUCO->get_feature_database()->cleanup_measurements(state->margtimestep());
    }
  }

  // 最后，如果需要，边缘化最旧的克隆状态
  // Finally marginalize the oldest clone if needed
  StateHelper::marginalize_old_clone(state);
  rT7 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // 调试信息和统计跟踪
  // Debug info, and stats tracking
  //===================================================================================

  // 获取时间统计信息
  // Get timing statitics information
  double time_track = (rT2 - rT1).total_microseconds() * 1e-6;      // 特征跟踪时间
  double time_prop = (rT3 - rT2).total_microseconds() * 1e-6;        // 状态传播时间
  double time_msckf = (rT4 - rT3).total_microseconds() * 1e-6;       // MSCKF更新时间
  double time_slam_update = (rT5 - rT4).total_microseconds() * 1e-6;  // SLAM更新时间
  double time_slam_delay = (rT6 - rT5).total_microseconds() * 1e-6; // SLAM延迟初始化时间
  double time_marg = (rT7 - rT6).total_microseconds() * 1e-6;       // 重新三角化和边缘化时间
  double time_total = (rT7 - rT1).total_microseconds() * 1e-6;     // 总时间

  // 打印时间信息
  // Timing information
  if (print_debug) {
    PRINT_DEBUG(BLUE "[VM]: 跟踪耗时 %.4f 秒\n" RESET, time_track);
    PRINT_DEBUG(BLUE "[VM]: 传播耗时 %.4f 秒\n" RESET, time_prop);
    PRINT_DEBUG(BLUE "[VM]: MSCKF更新耗时 %.4f 秒 (%d 个特征)\n" RESET, time_msckf, (int)featsup_MSCKF.size());
    if (state->_options.max_slam_features > 0) {
      PRINT_DEBUG(BLUE "[VM]: SLAM更新耗时 %.4f 秒 (%d 个特征)\n" RESET, time_slam_update, (int)state->_features_SLAM.size());
      PRINT_DEBUG(BLUE "[VM]: SLAM延迟初始化耗时 %.4f 秒 (%d 个特征)\n" RESET, time_slam_delay, (int)feats_slam_DELAYED.size());
    }
    PRINT_DEBUG(BLUE "[VM]: 重新三角化和边缘化耗时 %.4f 秒 (状态中有 %d 个克隆)\n" RESET, time_marg, (int)state->_clones_IMU.size());

    std::stringstream ss;
    ss << "[VM]: 总耗时 " << std::setprecision(4) << time_total << " 秒 (相机";
    for (const auto &id : message.sensor_ids) {
      ss << " " << id;
    }
    ss << ")" << std::endl;
    PRINT_DEBUG(BLUE "%s" RESET, ss.str().c_str());
  }
  // 最后，如果正在保存统计信息到文件，将其保存到文件
  // Finally if we are saving stats to file, lets save it to file
  if (params.record_timing_information && of_statistics.is_open()) {
    // 我们希望以IMU时钟帧发布
    // 状态中的时间戳将是最后一个相机时间
    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->_timestamp + t_ItoC;
    // 追加到文件
    // Append to the file
    of_statistics << std::fixed << std::setprecision(15) << timestamp_inI << "," << std::fixed << std::setprecision(5) << time_track << ","
                  << time_prop << "," << time_msckf << ",";
    if (state->_options.max_slam_features > 0) {
      of_statistics << time_slam_update << "," << time_slam_delay << ",";
    }
    of_statistics << time_marg << "," << time_total << std::endl;
    of_statistics.flush();
  }

  // 更新我们行驶的距离
  // Update our distance traveled
  if (timelastupdate != -1 && state->_clones_IMU.find(timelastupdate) != state->_clones_IMU.end()) {
    Eigen::Matrix<double, 3, 1> dx = state->_imu->pos() - state->_clones_IMU.at(timelastupdate)->pos();
    distance += dx.norm();  // 累加位移距离
  }
  timelastupdate = message.timestamp;  // 更新最后更新时间戳

  // 调试：打印当前状态
  // Debug, print our current state
  // q_GtoI: 全局到IMU的旋转四元数, p_IinG: IMU在全局坐标系中的位置, dist: 累计距离
  PRINT_INFO("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | Dist = %.2f s, \n", state->_imu->quat()(0),
             state->_imu->quat()(1), state->_imu->quat()(2), state->_imu->quat()(3), state->_imu->pos()(0), state->_imu->pos()(1),
             state->_imu->pos()(2), distance);
  // bg: 陀螺仪偏差, ba: 加速度计偏差
  PRINT_INFO("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f\n", state->_imu->bias_g()(0), state->_imu->bias_g()(1), state->_imu->bias_g()(2),
             state->_imu->bias_a()(0), state->_imu->bias_a()(1), state->_imu->bias_a()(2));

  // 调试：相机-IMU时间偏移
  // Debug for camera imu offset
  if (state->_options.do_calib_camera_timeoffset) {
    PRINT_INFO("相机-IMU时间偏移 = %.5f\n", state->_calib_dt_CAMtoIMU->value()(0));
  }

  // 调试：相机内参
  // Debug for camera intrinsics
  if (state->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = state->_cam_intrinsics.at(i);
      PRINT_INFO("相机%d 内参 = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f\n", (int)i, calib->value()(0), calib->value()(1),
                 calib->value()(2), calib->value()(3), calib->value()(4), calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // 调试：相机外参（IMU到相机的变换）
  // Debug for camera extrinsics
  if (state->_options.do_calib_camera_pose) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = state->_calib_IMUtoCAM.at(i);
      PRINT_INFO("相机%d 外参 = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f\n", (int)i, calib->quat()(0), calib->quat()(1), calib->quat()(2),
                 calib->quat()(3), calib->pos()(0), calib->pos()(1), calib->pos()(2));
    }
  }

  // 调试：IMU内参
  // Debug for imu intrinsics
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    // 陀螺仪到IMU的旋转四元数
    PRINT_INFO("q_GYROtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_GYROtoIMU->value()(0), state->_calib_imu_GYROtoIMU->value()(1),
               state->_calib_imu_GYROtoIMU->value()(2), state->_calib_imu_GYROtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    // 加速度计到IMU的旋转四元数
    PRINT_INFO("q_ACCtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_ACCtoIMU->value()(0), state->_calib_imu_ACCtoIMU->value()(1),
               state->_calib_imu_ACCtoIMU->value()(2), state->_calib_imu_ACCtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    // 陀螺仪偏差和标度因子矩阵 Dw
    PRINT_INFO("Dw = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
               state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
               state->_calib_imu_dw->value()(5));
    // 加速度计偏差和标度因子矩阵 Da
    PRINT_INFO("Da = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
               state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
               state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    // RPNG模型的陀螺仪参数
    PRINT_INFO("Dw = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
               state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
               state->_calib_imu_dw->value()(5));
    // RPNG模型的加速度计参数
    PRINT_INFO("Da = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
               state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
               state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.do_calib_imu_g_sensitivity) {
    // 陀螺仪g敏感度矩阵 Tg
    PRINT_INFO("Tg = | %.4f,%.4f,%.4f |  %.4f,%.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_tg->value()(0),
               state->_calib_imu_tg->value()(1), state->_calib_imu_tg->value()(2), state->_calib_imu_tg->value()(3),
               state->_calib_imu_tg->value()(4), state->_calib_imu_tg->value()(5), state->_calib_imu_tg->value()(6),
               state->_calib_imu_tg->value()(7), state->_calib_imu_tg->value()(8));
  }
}
