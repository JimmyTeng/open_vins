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

#include "DynamicInitializer.h"

#include "ceres/Factor_GenericPrior.h"
#include "ceres/Factor_ImageReprojCalib.h"
#include "ceres/Factor_ImuCPIv1.h"
#include "ceres/State_JPLQuatLocal.h"
#include "utils/helper.h"

#include "cpi/CpiV1.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

bool DynamicInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                                    std::shared_ptr<ov_type::IMU> &_imu, std::map<double, std::shared_ptr<ov_type::PoseJPL>> &_clones_IMU,
                                    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> &_features_SLAM) {


  PRINT_INFO(CYAN "[DynamicInitializer] 动态初始化开始\n" RESET);

  // 获取我们将尝试进行初始化的最新和最旧时间戳！
  auto rT1 = boost::posix_time::microsec_clock::local_time();
  double newest_cam_time = -1;
  for (auto const &feat : _db->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        newest_cam_time = std::max(newest_cam_time, time);
      }
    }
  }
  double oldest_time = newest_cam_time - params.init_window_time;
  if (newest_cam_time < 0 || oldest_time < 0) {
    PRINT_INFO(YELLOW "[DynamicInitializer] 动态初始化失败: 无效的时间戳\n" RESET);
    return false;
  }

  // 移除所有早于初始化窗口的测量数据
  // 然后我们将尝试使用特征数据库中的所有特征点！
  _db->cleanup_measurements(oldest_time);
  bool have_old_imu_readings = false;
  auto it_imu = imu_data->begin();
  while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + params.calib_camimu_dt) {
    have_old_imu_readings = true;
    it_imu = imu_data->erase(it_imu);
  }
  if (_db->get_internal_data().size() < 0.75 * params.init_max_features) {
    PRINT_INFO(YELLOW "[DynamicInitializer] 动态初始化失败: 特征数据库检查失败 - 有 %zu 个特征点, 需要 %.0f\n" RESET, 
               _db->get_internal_data().size(), 0.75 * params.init_max_features);
    return false;
  }
  if (imu_data->size() < 2 || !have_old_imu_readings) {
    PRINT_INFO(YELLOW "[DynamicInitializer] 动态初始化失败: IMU数据检查失败 - imu_data->size()=%zu, have_old_imu_readings=%d\n" RESET, 
               imu_data->size(), have_old_imu_readings);
    return false;
  }
  if (print_debug) {
    PRINT_DEBUG(CYAN "[DynamicInitializer] 动态初始化特征数据库检查成功 - 有 %zu 个特征点, IMU读数: %zu\n" RESET, 
             _db->get_internal_data().size(), imu_data->size());
  }

  // 现在我们将在这里复制特征点
  // 这样做是为了确保特征数据库可以继续以异步方式追加新的测量数据，
  // 这样初始化可以在辅助线程中执行，同时特征跟踪仍在进行。
  std::unordered_map<size_t, std::shared_ptr<Feature>> features;
  for (const auto &feat : _db->get_internal_data()) {
    auto feat_new = std::make_shared<Feature>();
    feat_new->featid = feat.second->featid;
    feat_new->uvs = feat.second->uvs;
    feat_new->uvs_norm = feat.second->uvs_norm;
    feat_new->timestamps = feat.second->timestamps;
    features.insert({feat.first, feat_new});
  }

  // ======================================================
  // ======================================================

  // 设置参数
  const int min_num_meas_to_optimize = (int)params.init_window_time;  // 优化所需的最小测量数量
  const int min_valid_features = 8;  // 最小有效特征点数量

  // 可用于优化的特征点验证信息
  bool have_stereo = false;
  int count_valid_features = 0;
  std::map<size_t, int> map_features_num_meas;
  int num_measurements = 0;
  double oldest_camera_time = INFINITY;
  std::map<double, bool> map_camera_times; // 相机位姿时间戳
  map_camera_times[newest_cam_time] = true; // always insert final pose
  std::map<size_t, bool> map_camera_ids;
  double pose_dt_avg = params.init_window_time / (double)(params.init_dyn_num_pose + 1);
  for (auto const &feat : features) {

    // 遍历每个时间戳并确保它是有效的位姿
    std::vector<double> times;
    std::map<size_t, bool> camids;
    for (auto const &camtime : feat.second->timestamps) {
      for (double time : camtime.second) {
        double time_dt = INFINITY;
        for (auto const &tmp : map_camera_times) {
          time_dt = std::min(time_dt, std::abs(time - tmp.first));
        }
        for (auto const &tmp : times) {
          time_dt = std::min(time_dt, std::abs(time - tmp));
        }
        // 要么这个位姿是期望频率下的新位姿
        // 要么是我们已经拥有的时间戳，因此可以免费使用
        if (time_dt >= pose_dt_avg || time_dt == 0.0) {
          times.push_back(time);
          camids[camtime.first] = true;
        }
      }
    }

    // 如果测量数量不足，这不是我们应该使用的特征点
    map_features_num_meas[feat.first] = (int)times.size();
    if (map_features_num_meas[feat.first] < min_num_meas_to_optimize)
      continue;
    // PRINT_INFO(YELLOW "[DynamicInitializer] times.size(): %d\n" RESET, times.size());
    // 如果我们有足够的测量数据，应该添加这个特征点！
    for (auto const &tmp : times) {
      map_camera_times[tmp] = true;
      oldest_camera_time = std::min(oldest_camera_time, tmp);
      num_measurements += 2;
    }
    for (auto const &tmp : camids) {
      map_camera_ids[tmp.first] = true;
    }
    if (camids.size() > 1) {
      have_stereo = true;
    }
    count_valid_features++;
  }

  // 如果我们没有完整的窗口或测量数据不足，则返回
  // 同时检查是否有足够的特征点进行初始化
  if ((int)map_camera_times.size() < params.init_dyn_num_pose) {
    PRINT_INFO(YELLOW "[DynamicInitializer] 动态初始化失败: 相机位姿不足 - 有 %zu, 需要 %d\n" RESET, 
               map_camera_times.size(), params.init_dyn_num_pose);
      // 打印时间戳信息用于调试
    if (print_debug && map_camera_times.size() > 0) {
      PRINT_INFO(YELLOW "[DynamicInitializer] 相机位姿时间戳收集:\n" RESET);
      int idx = 0;
      for (auto const &timepair : map_camera_times) {
        double time_relative = timepair.first - oldest_camera_time;
        PRINT_INFO(YELLOW "[DynamicInitializer] [%d] timestamp: %.6f sec, relative: %.6f sec\n" RESET, 
                   idx++, timepair.first, time_relative);
      }
      if (map_camera_times.size() > 1) {
        auto it = map_camera_times.begin();
        double first_time = it->first;
        ++it;
        double second_time = it->first;
        double time_interval = second_time - first_time;
        PRINT_INFO(YELLOW "[DynamicInitializer] 第一个和第二个时间戳间隔: %.6f sec (required: >= %.6f sec)\n" RESET,
                   time_interval, pose_dt_avg);
      }
    }
    if (print_debug) { 
    PRINT_INFO(YELLOW "[DynamicInitializer] pose_dt_avg 阈值: %.6f sec, init_window_time: %.2f sec\n" RESET,
               pose_dt_avg, params.init_window_time);
    }
    return false;
  }
  if (count_valid_features < min_valid_features) {
    // 打印特征点统计信息
    int feat_with_enough_meas = 0;
    for (auto const &feat : features) {
      if (map_features_num_meas[feat.first] >= min_num_meas_to_optimize) {
        feat_with_enough_meas++;
      }
    }
    PRINT_INFO(YELLOW "[DynamicInitializer] 特征验证失败 - 有效: %d, 需要: %d, min_num_meas: %d\n" RESET, 
               count_valid_features, min_valid_features, min_num_meas_to_optimize);
    return false;
  }
  if (print_debug) {
    PRINT_INFO(CYAN "[DynamicInitializer] 特征验证成功 - 有效特征点: %d, 相机位姿: %zu, 测量: %d\n" RESET, 
             count_valid_features, map_camera_times.size(), num_measurements);
  // 打印相机位姿时间戳用于调试
    PRINT_INFO(CYAN "[DynamicInitializer] 相机位姿时间戳 (总共 %zu):\n" RESET, map_camera_times.size());
  
    int idx = 0;
    for (auto const &timepair : map_camera_times) {
      double time_relative = timepair.first - oldest_camera_time;
      double time_from_newest = newest_cam_time - timepair.first;
      PRINT_INFO(CYAN "[DynamicInitializer] [%d] timestamp: %.6f sec, relative: %.6f sec, from_newest: %.6f sec\n" RESET,
                 idx++, timepair.first, time_relative, time_from_newest);
    }
  
  if (print_debug && map_camera_times.size() > 1) {
    auto it = map_camera_times.begin();
    double prev_time = it->first;
    ++it;
    PRINT_INFO(CYAN "[DynamicInitializer] 时间戳间隔:\n" RESET);
    int interval_idx = 0;
    for (; it != map_camera_times.end(); ++it) {
      double interval = it->first - prev_time;
        PRINT_INFO(CYAN "[DynamicInitializer] [%d->%d] interval: %.6f sec (required: >= %.6f sec) %s\n" RESET,
                   interval_idx, interval_idx+1, interval, pose_dt_avg,
                   (interval >= pose_dt_avg ? "✓" : "✗"));
          prev_time = it->first;
          interval_idx++;
      }
    }
  }
  // 启动文件指定的偏置初始猜测值
  // 我们现在不费力恢复偏置，因为它们应该在启动前大致已知，
  // 或者可以认为接近零...

  // TODO:如果有静态初始化器，则使用静态初始化器的偏置
  Eigen::Vector3d gyroscope_bias = params.init_dyn_bias_g;
  Eigen::Vector3d accelerometer_bias = params.init_dyn_bias_a;

  // 检查我们是否有角速度/姿态变化
  double accel_inI_norm = 0.0;
  double theta_inI_norm = 0.0;
  double time0_in_imu = oldest_camera_time + params.calib_camimu_dt;
  double time1_in_imu = newest_cam_time + params.calib_camimu_dt;
  std::vector<ov_core::ImuData> readings = InitializerHelper::select_imu_readings(*imu_data, time0_in_imu, time1_in_imu);
  assert(readings.size() > 2);
  for (size_t k = 0; k < readings.size() - 1; k++) {
    auto imu0 = readings.at(k);
    auto imu1 = readings.at(k + 1);
    double dt = imu1.timestamp - imu0.timestamp;
    Eigen::Vector3d wm = 0.5 * (imu0.wm + imu1.wm) - gyroscope_bias;
    Eigen::Vector3d am = 0.5 * (imu0.am + imu1.am) - accelerometer_bias;
    theta_inI_norm += (-wm * dt).norm();
    accel_inI_norm += am.norm();
  }
  accel_inI_norm /= (double)(readings.size() - 1);
  if (180.0 / M_PI * theta_inI_norm < params.init_dyn_min_deg) {
    PRINT_WARNING(YELLOW "[DynamicInitializer] 动态初始化失败: 陀螺仪只有 %.2f 度变化 (%.2f 阈值)\n" RESET, 180.0 / M_PI * theta_inI_norm,
                  params.init_dyn_min_deg);
    return false;
  }
  if (print_debug) {
    PRINT_DEBUG("[DynamicInitializer] |theta_I| = %.4f deg and |accel| = %.4f\n", 180.0 / M_PI * theta_inI_norm, accel_inI_norm);
  }

  //  // 在第一帧中创建特征点的方向向量
  //  // 这给我们：p_FinI0 = depth * bearing
  //  Eigen::Vector4d q_ItoC = data_ori.camera_q_ItoC.at(cam_id);
  //  Eigen::Vector3d p_IinC = data_init.camera_p_IinC.at(cam_id);
  //  Eigen::Matrix3d R_ItoC = quat_2_Rot(q_ItoC);
  //  std::map<size_t, Eigen::Vector3d> features_bearings;
  //  std::map<size_t, int> features_index;
  //  for (auto const &feat : features) {
  //  if (map_features_num_meas[feat.first] < min_num_meas_to_optimize)
  //    continue;
  //    assert(feat->timestamps.find(cam_id) != feat->timestamps.end());
  //    double timestamp = data_ori.timestamps_cam.at(cam_id).at(0);
  //    auto it0 = std::find(feat->timestamps.at(cam_id).begin(), feat->timestamps.at(cam_id).end(), timestamp);
  //    if (it0 == feat->timestamps.at(cam_id).end())
  //      continue;
  //    auto idx0 = std::distance(feat->timestamps.at(cam_id).begin(), it0);
  //    Eigen::Vector3d bearing;
  //    bearing << feat->uvs_norm.at(cam_id).at(idx0)(0), feat->uvs_norm.at(cam_id).at(idx0)(1), 1;
  //    bearing = bearing / bearing.norm();
  //    bearing = R_ItoC.transpose() * bearing;
  //    features_bearings.insert({feat->featid, bearing});
  //    features_index.insert({feat->featid, (int)features_index.size()});
  //  }
  auto rT2 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // ======================================================

  // 我们将恢复特征点位置、速度和重力
  // 基于以下论文中的方程(14)：
  // https://ieeexplore.ieee.org/abstract/document/6386235
  // 状态顺序为：[特征点, 速度, 重力]
  // 特征大小为1将使用特征点的第一个方向向量作为真实值（仅深度..）
  const bool use_single_depth = false;
  int size_feature = (use_single_depth) ? 1 : 3;
  int num_features = count_valid_features;
  int system_size = size_feature * num_features + 3 + 3;

  // 确保我们有足够的测量数据来完全约束系统
  PRINT_INFO(CYAN "[DynamicInitializer] 系统大小检查 - 测量: %d, 系统大小: %d, 特征: %d\n" RESET, 
             num_measurements, system_size, num_features);
  if (num_measurements < system_size) {
    PRINT_INFO(YELLOW "[DynamicInitializer] 失败: 测量约束失败 - 需要 %d 测量 for %d 状态\n" RESET, 
               system_size, system_size);
    return false;
  }

  // 现在让我们从第一个时间到最后一个时间进行预积分 TODO  用连续积分替代
  assert(oldest_camera_time < newest_cam_time);
  double last_camera_timestamp = 0.0;
  std::map<double, std::shared_ptr<ov_core::CpiV1>> map_camera_cpi_I0toIi, map_camera_cpi_IitoIi1;
  for (auto const &timepair : map_camera_times) {

    // 第一个时间戳处不进行预积分
    double current_time = timepair.first;
    if (current_time == oldest_camera_time) {
      map_camera_cpi_I0toIi.insert({current_time, nullptr});
      map_camera_cpi_IitoIi1.insert({current_time, nullptr});
      last_camera_timestamp = current_time;
      continue;
    }

    // 执行从I0到Ii的预积分（用于线性系统）
    double cpiI0toIi1_time0_in_imu = oldest_camera_time + params.calib_camimu_dt;
    double cpiI0toIi1_time1_in_imu = current_time + params.calib_camimu_dt;
    auto cpiI0toIi1 = std::make_shared<ov_core::CpiV1>(params.sigma_w, params.sigma_wb, params.sigma_a, params.sigma_ab, true);
    cpiI0toIi1->setLinearizationPoints(gyroscope_bias, accelerometer_bias);
    std::vector<ov_core::ImuData> cpiI0toIi1_readings =
        InitializerHelper::select_imu_readings(*imu_data, cpiI0toIi1_time0_in_imu, cpiI0toIi1_time1_in_imu);
    if (cpiI0toIi1_readings.size() < 2) {
      PRINT_DEBUG(YELLOW "[DynamicInitializer] 失败: 相机 %.2f 在有 %zu IMU 读数!\n" RESET, (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu),
                  cpiI0toIi1_readings.size());
      return false;
    }
    double cpiI0toIi1_dt_imu = cpiI0toIi1_readings.at(cpiI0toIi1_readings.size() - 1).timestamp - cpiI0toIi1_readings.at(0).timestamp;
    if (std::abs(cpiI0toIi1_dt_imu - (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu)) > 0.01) {
      PRINT_DEBUG(YELLOW "[DynamicInitializer] 失败: 相机 IMU 只传播了 %.3f of %.3f\n" RESET, cpiI0toIi1_dt_imu,
                  (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu));
      return false;
    }
    for (size_t k = 0; k < cpiI0toIi1_readings.size() - 1; k++) {
      auto imu0 = cpiI0toIi1_readings.at(k);
      auto imu1 = cpiI0toIi1_readings.at(k + 1);
      cpiI0toIi1->feed_IMU(imu0.timestamp, imu1.timestamp, imu0.wm, imu0.am, imu1.wm, imu1.am);
    }

    // 执行从Ii到Ii1的预积分（用于MLE优化）
    double cpiIitoIi1_time0_in_imu = last_camera_timestamp + params.calib_camimu_dt;
    double cpiIitoIi1_time1_in_imu = current_time + params.calib_camimu_dt;
    auto cpiIitoIi1 = std::make_shared<ov_core::CpiV1>(params.sigma_w, params.sigma_wb, params.sigma_a, params.sigma_ab, true);
    cpiIitoIi1->setLinearizationPoints(gyroscope_bias, accelerometer_bias);
    std::vector<ov_core::ImuData> cpiIitoIi1_readings =
        InitializerHelper::select_imu_readings(*imu_data, cpiIitoIi1_time0_in_imu, cpiIitoIi1_time1_in_imu);
    if (cpiIitoIi1_readings.size() < 2) {
      PRINT_WARNING(YELLOW "[DynamicInitializer] 动态初始化失败: 相机 %.2f 在有 %zu IMU 读数!\n" RESET, (cpiIitoIi1_time1_in_imu - cpiIitoIi1_time0_in_imu),
                  cpiIitoIi1_readings.size());
      return false;
    }
    double cpiIitoIi1_dt_imu = cpiIitoIi1_readings.at(cpiIitoIi1_readings.size() - 1).timestamp - cpiIitoIi1_readings.at(0).timestamp;
    if (std::abs(cpiIitoIi1_dt_imu - (cpiIitoIi1_time1_in_imu - cpiIitoIi1_time0_in_imu)) > 0.01) {
      PRINT_WARNING(YELLOW "[DynamicInitializer] 动态初始化失败: 相机 IMU 只传播了 %.3f of %.3f\n" RESET, cpiIitoIi1_dt_imu,
                  (cpiIitoIi1_time1_in_imu - cpiIitoIi1_time0_in_imu));
      return false;
    }
    for (size_t k = 0; k < cpiIitoIi1_readings.size() - 1; k++) {
      auto imu0 = cpiIitoIi1_readings.at(k);
      auto imu1 = cpiIitoIi1_readings.at(k + 1);
      cpiIitoIi1->feed_IMU(imu0.timestamp, imu1.timestamp, imu0.wm, imu0.am, imu1.wm, imu1.am);
    }

    // 最后将我们的积分结果推入！
    map_camera_cpi_I0toIi.insert({current_time, cpiI0toIi1});
    map_camera_cpi_IitoIi1.insert({current_time, cpiIitoIi1});
    last_camera_timestamp = current_time;
  }
  auto rT2a = boost::posix_time::microsec_clock::local_time();

  // 遍历每个特征点观测并添加它！
  // 状态顺序为：[特征点, 速度, 重力]
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_measurements, system_size);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_measurements);
 
  if (print_debug) {
    PRINT_DEBUG("[DynamicInitializer] 系统创建 - 测量: %d x 状态: %d (%d 特征, %s)\n", num_measurements, system_size, num_features,
              (have_stereo) ? "stereo" : "mono");
  }
  int index_meas = 0;
  int idx_feat = 0;
  std::map<size_t, int> A_index_features;
  for (auto const &feat : features) {
    // 跳过测量数据不足的特征点
    if (map_features_num_meas[feat.first] < min_num_meas_to_optimize) 
      continue;
    // 如果特征点不在A_index_features中，则添加到A_index_features中
    if (A_index_features.find(feat.first) == A_index_features.end()) {
      A_index_features.insert({feat.first, idx_feat});
      idx_feat += 1;
    }
    for (auto const &camtime : feat.second->timestamps) {
      // 这个相机
      size_t cam_id = camtime.first;
      Eigen::Vector4d q_ItoC = params.camera_extrinsics.at(cam_id).block(0, 0, 4, 1);
      Eigen::Vector3d p_IinC = params.camera_extrinsics.at(cam_id).block(4, 0, 3, 1);
      Eigen::Matrix3d R_ItoC = quat_2_Rot(q_ItoC);

      // Loop through each observation
      for (size_t i = 0; i < camtime.second.size(); i++) {

        // Skip measurements we don't have poses for
        double time = feat.second->timestamps.at(cam_id).at(i);
        if (map_camera_times.find(time) == map_camera_times.end())
          continue;

        // Our measurement
        Eigen::Vector2d uv_norm;
        uv_norm << (double)feat.second->uvs_norm.at(cam_id).at(i)(0), (double)feat.second->uvs_norm.at(cam_id).at(i)(1);

        // 预积分值
        double DT = 0.0;
        Eigen::MatrixXd R_I0toIk = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd alpha_I0toIk = Eigen::MatrixXd::Zero(3, 1);
        if (map_camera_cpi_I0toIi.find(time) != map_camera_cpi_I0toIi.end() && map_camera_cpi_I0toIi.at(time) != nullptr) {
          DT = map_camera_cpi_I0toIi.at(time)->DT;
          R_I0toIk = map_camera_cpi_I0toIi.at(time)->R_k2tau;
          alpha_I0toIk = map_camera_cpi_I0toIi.at(time)->alpha_tau;
        }

        // 基于特征点重投影创建线性系统
        // [ 1 0 -u ] p_FinCi = [ 0 ]
        // [ 0 1 -v ]           [ 0 ]
        // 其中
        // p_FinCi = R_C0toCi * R_ItoC * (p_FinI0 - p_IiinI0) + p_IinC
        //         = R_C0toCi * R_ItoC * (p_FinI0 - v_I0inI0 * dt - 0.5 * grav_inI0 * dt^2 - alpha) + p_IinC
        Eigen::MatrixXd H_proj = Eigen::MatrixXd::Zero(2, 3);
        H_proj << 1, 0, -uv_norm(0), 0, 1, -uv_norm(1);
        Eigen::MatrixXd Y = H_proj * R_ItoC * R_I0toIk;
        Eigen::MatrixXd H_i = Eigen::MatrixXd::Zero(2, system_size);
        Eigen::MatrixXd b_i = Y * alpha_I0toIk - H_proj * p_IinC;
        if (size_feature == 1) {
          assert(false);
          // 代入 p_FinI0 = z*bearing_inC0_rotI0 - R_ItoC^T*p_IinC
          // H_i.block(0, size_feature * A_index_features.at(feat.first), 2, 1) = Y * features_bearings.at(feat.first);
          // b_i += Y * R_ItoC.transpose() * p_IinC;
        } else {
          H_i.block(0, size_feature * A_index_features.at(feat.first), 2, 3) = Y; // 特征点
        }
        H_i.block(0, size_feature * num_features + 0, 2, 3) = -DT * Y;            // 速度
        H_i.block(0, size_feature * num_features + 3, 2, 3) = 0.5 * DT * DT * Y;  // 重力

        // 否则让我们将其添加到我们的系统中！
        A.block(index_meas, 0, 2, A.cols()) = H_i;
        b.block(index_meas, 0, 2, 1) = b_i;
        index_meas += 2;
      }
    }
  }
  auto rT3 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // ======================================================

  // 无约束求解线性系统
  // Eigen::MatrixXd AtA = A.transpose() * A;
  // Eigen::MatrixXd Atb = A.transpose() * b;
  // Eigen::MatrixXd x_hat = AtA.colPivHouseholderQr().solve(Atb);

  // 带约束求解 |g| = 9.81 约束
  Eigen::MatrixXd A1 = A.block(0, 0, A.rows(), A.cols() - 3);
  // Eigen::MatrixXd A1A1_inv = (A1.transpose() * A1).inverse();
  Eigen::MatrixXd A1A1_inv = (A1.transpose() * A1).llt().solve(Eigen::MatrixXd::Identity(A1.cols(), A1.cols()));
  Eigen::MatrixXd A2 = A.block(0, A.cols() - 3, A.rows(), 3);
  Eigen::MatrixXd Temp = A2.transpose() * (Eigen::MatrixXd::Identity(A1.rows(), A1.rows()) - A1 * A1A1_inv * A1.transpose());
  Eigen::MatrixXd D = Temp * A2;
  Eigen::MatrixXd d = Temp * b;
  Eigen::Matrix<double, 7, 1> coeff = InitializerHelper::compute_dongsi_coeff(D, d, params.gravity_mag);

  // 创建我们多项式的伴随矩阵
  // https://en.wikipedia.org/wiki/Companion_matrix
  assert(coeff(0) == 1);
  Eigen::Matrix<double, 6, 6> companion_matrix = Eigen::Matrix<double, 6, 6>::Zero(coeff.rows() - 1, coeff.rows() - 1);
  companion_matrix.diagonal(-1).setOnes();
  companion_matrix.col(companion_matrix.cols() - 1) = -coeff.reverse().head(coeff.rows() - 1);
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd0(companion_matrix);
  Eigen::MatrixXd singularValues0 = svd0.singularValues();
  double cond0 = singularValues0(0) / singularValues0(singularValues0.rows() - 1);
  
  if (print_debug) {
    PRINT_DEBUG("[DynamicInitializer] CM cond = %.3f | rank = %d of %d (%4.3e thresh)\n", cond0, (int)svd0.rank(), (int)companion_matrix.cols(),
              svd0.threshold());
  }
  if (svd0.rank() != companion_matrix.rows()) {
    PRINT_ERROR(RED "[DynamicInitializer] 动态初始化失败: 特征值分解不是满秩!!\n" RESET);
    return false;
  }

  // 找到它的特征值（可能是复数）
  Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> solver(companion_matrix, false);
  if (solver.info() != Eigen::Success) {
    PRINT_ERROR(RED "[DynamicInitializer] 动态初始化失败: 无法计算特征值分解!!\n" RESET);
    return false;
  }

  // 找到最小的实特征值
  // 注意：我们找到给出最小约束成本的那个
  // 注意：不确定是否最好，但给出正确幅值的应该是不错的？
  bool lambda_found = false;
  double lambda_min = -1;
  double cost_min = INFINITY;
  Eigen::MatrixXd I_dd = Eigen::MatrixXd::Identity(D.rows(), D.rows());
  // double g2 = params.gravity_mag * params.gravity_mag;
  // Eigen::MatrixXd ddt = d * d.transpose();
  for (int i = 0; i < solver.eigenvalues().size(); i++) {
    auto val = solver.eigenvalues()(i);
    if (val.imag() == 0) {
      double lambda = val.real();
      // Eigen::MatrixXd mat = (D - lambda * I_dd) * (D - lambda * I_dd) - 1 / g2 * ddt;
      // double cost = mat.determinant();
      Eigen::MatrixXd D_lambdaI_inv = (D - lambda * I_dd).llt().solve(I_dd);
      Eigen::VectorXd state_grav = D_lambdaI_inv * d;
      double cost = std::abs(state_grav.norm() - params.gravity_mag);
      // std::cout << lambda << " - " << cost << " -> " << state_grav.transpose() << std::endl;
      if (!lambda_found || cost < cost_min) {
        lambda_found = true;
        lambda_min = lambda;
        cost_min = cost;
      }
    }
  }
  if (!lambda_found) {
    PRINT_ERROR(RED "[DynamicInitializer] 动态初始化失败: 无法找到实特征值!!\n" RESET);
    return false;
  }
  if (print_debug) {
    PRINT_DEBUG("[DynamicInitializer] 最小实特征值 = %.5f (成本为 %f)\n", lambda_min, cost_min);
  }

  // 从约束中恢复我们的重力！
  // Eigen::MatrixXd D_lambdaI_inv = (D - lambda_min * I_dd).inverse();
  Eigen::MatrixXd D_lambdaI_inv = (D - lambda_min * I_dd).llt().solve(I_dd);
  Eigen::VectorXd state_grav = D_lambdaI_inv * d;

  // 覆盖我们的状态：[特征点, 速度, 重力]
  Eigen::VectorXd state_feat_vel = -A1A1_inv * A1.transpose() * A2 * state_grav + A1A1_inv * A1.transpose() * b;
  Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(system_size, 1);
  x_hat.block(0, 0, size_feature * num_features + 3, 1) = state_feat_vel;
  x_hat.block(size_feature * num_features + 3, 0, 3, 1) = state_grav;
  Eigen::Vector3d v_I0inI0 = x_hat.block(size_feature * num_features + 0, 0, 3, 1);
  PRINT_INFO("[DynamicInitializer] 速度在 I0 为 %.3f,%.3f,%.3f 和 |v| = %.4f\n", v_I0inI0(0), v_I0inI0(1), v_I0inI0(2), v_I0inI0.norm());

  // 检查重力幅值以查看是否收敛
  Eigen::Vector3d gravity_inI0 = x_hat.block(size_feature * num_features + 3, 0, 3, 1);
  double init_max_grav_difference = 1e-3;
  if (std::abs(gravity_inI0.norm() - params.gravity_mag) > init_max_grav_difference) {
    PRINT_WARNING(YELLOW "[DynamicInitializer] 动态初始化失败: 重力未收敛 (%.3f > %.3f)\n" RESET, std::abs(gravity_inI0.norm() - params.gravity_mag),
                  init_max_grav_difference);
    return false;
  }
  PRINT_INFO("[DynamicInitializer] 重力在 I0 为 %.3f,%.3f,%.3f 和 |g| = %.4f\n", gravity_inI0(0), gravity_inI0(1), gravity_inI0(2),
             gravity_inI0.norm());
  PRINT_INFO(CYAN "[DynamicInitializer] MLE 优化完成 - 速度: |v|=%.4f, 重力: |g|=%.4f\n" RESET,
             v_I0inI0.norm(), gravity_inI0.norm());
  auto rT4 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // ======================================================

  // 提取IMU状态元素
  std::map<double, Eigen::VectorXd> ori_I0toIi, pos_IiinI0, vel_IiinI0;
  for (auto const &timepair : map_camera_times) {

    // 这个位姿的时间戳
    double time = timepair.first;

    // 获取我们的CPI积分值
    double DT = 0.0;
    Eigen::MatrixXd R_I0toIk = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd alpha_I0toIk = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd beta_I0toIk = Eigen::MatrixXd::Zero(3, 1);
    if (map_camera_cpi_I0toIi.find(time) != map_camera_cpi_I0toIi.end() && map_camera_cpi_I0toIi.at(time) != nullptr) {
      auto cpi = map_camera_cpi_I0toIi.at(time);
      DT = cpi->DT;
      R_I0toIk = cpi->R_k2tau;
      alpha_I0toIk = cpi->alpha_tau;
      beta_I0toIk = cpi->beta_tau;
    }

    // 积分以获得相对于当前时间戳的值
    Eigen::Vector3d p_IkinI0 = v_I0inI0 * DT - 0.5 * gravity_inI0 * DT * DT + alpha_I0toIk;
    Eigen::Vector3d v_IkinI0 = v_I0inI0 - gravity_inI0 * DT + beta_I0toIk;

    // 记录所有转换到I0坐标系的值
    ori_I0toIi.insert({time, rot_2_quat(R_I0toIk)});
    pos_IiinI0.insert({time, p_IkinI0});
    vel_IiinI0.insert({time, v_IkinI0});
  }

  // 在第一个IMU坐标系中恢复特征点
  count_valid_features = 0;
  std::map<size_t, Eigen::Vector3d> features_inI0;
  int total_features_checked = 0;
  int features_behind_camera = 0;
  int features_with_insufficient_meas = 0;
  
  PRINT_INFO(CYAN "[DynamicInitializer] 开始特征恢复 after MLE - 检查 %zu 特征\n" RESET, features.size());
  
  for (auto const &feat : features) {
    if (map_features_num_meas[feat.first] < min_num_meas_to_optimize) {
      features_with_insufficient_meas++;
      continue;
    }
    total_features_checked++;
    
    Eigen::Vector3d p_FinI0;
    if (size_feature == 1) {
      assert(false);
      // double depth = x_hat(size_feature * A_index_features.at(feat.first), 0);
      // p_FinI0 = depth * features_bearings.at(feat.first) - R_ItoC.transpose() * p_IinC;
    } else {
      p_FinI0 = x_hat.block(size_feature * A_index_features.at(feat.first), 0, 3, 1);
    }
    
    bool is_behind = false;
    Eigen::Vector3d p_FinC0_last{};
    for (auto const &camtime : feat.second->timestamps) {
      size_t cam_id = camtime.first;
      Eigen::Vector4d q_ItoC = params.camera_extrinsics.at(cam_id).block(0, 0, 4, 1);
      Eigen::Vector3d p_IinC = params.camera_extrinsics.at(cam_id).block(4, 0, 3, 1);
      Eigen::Matrix3d R_ItoC = quat_2_Rot(q_ItoC);
      Eigen::Vector3d p_FinC0 = R_ItoC * p_FinI0 + p_IinC;
      p_FinC0_last = p_FinC0;
      
      if (p_FinC0(2) < 0) {
        is_behind = true;
        if (total_features_checked <= 5) {  // 打印前5个特征点用于调试
          PRINT_INFO(YELLOW "[DynamicInitializer] 特征 %zu 在相机后面 - p_FinI0=(%.3f,%.3f,%.3f), p_FinC0=(%.3f,%.3f,%.3f), z=%.3f\n" RESET,
                     feat.first, p_FinI0(0), p_FinI0(1), p_FinI0(2), 
                     p_FinC0(0), p_FinC0(1), p_FinC0(2), p_FinC0(2));
        }
      }
    }
    
    if (is_behind) {
      features_behind_camera++;
    } else {
      features_inI0.insert({feat.first, p_FinI0});
      count_valid_features++;
      if (count_valid_features <= 3) {  // 打印前3个有效特征点
        PRINT_INFO(GREEN "[init-d-debug]: Valid feature %zu - p_FinI0=(%.3f,%.3f,%.3f), p_FinC0=(%.3f,%.3f,%.3f)\n" RESET,
                   feat.first, p_FinI0(0), p_FinI0(1), p_FinI0(2),
                   p_FinC0_last(0), p_FinC0_last(1), p_FinC0_last(2));
      }
    }
  }
  
  PRINT_INFO(YELLOW "[DynamicInitializer] 特征恢复总结:\n" RESET);
  PRINT_INFO(YELLOW "  - 总特征: %zu\n" RESET, features.size());
  PRINT_INFO(YELLOW "  - 测量不足的特征: %d\n" RESET, features_with_insufficient_meas);
  PRINT_INFO(YELLOW "  - 检查的特征: %d\n" RESET, total_features_checked);
  PRINT_INFO(YELLOW "  - 在相机后面的特征: %d\n" RESET, features_behind_camera);
  PRINT_INFO(YELLOW "  - 有效特征: %d (需要: %d)\n" RESET, count_valid_features, min_valid_features);
  
  if (count_valid_features < min_valid_features) {
    PRINT_ERROR(YELLOW "[DynamicInitializer] 动态初始化失败: 特征不足 (%zu < %d)!\n" RESET, count_valid_features, min_valid_features);
    // 打印相机外参用于调试
    for (auto const &ext : params.camera_extrinsics) {
      Eigen::Vector4d q_ItoC = ext.second.block(0, 0, 4, 1);
      Eigen::Vector3d p_IinC = ext.second.block(4, 0, 3, 1);
      PRINT_INFO(YELLOW "[DynamicInitializer] 相机 %zu 外参 - q_ItoC=(%.4f,%.4f,%.4f,%.4f), p_IinC=(%.4f,%.4f,%.4f)\n" RESET,
                 ext.first, q_ItoC(0), q_ItoC(1), q_ItoC(2), q_ItoC(3), p_IinC(0), p_IinC(1), p_IinC(2));
    }
    return false;
  }

  // 将我们的状态转换为重力对齐的全局参考坐标系
  // 这里我们说I0坐标系位于0,0,0并与全局原点共享
  Eigen::Matrix3d R_GtoI0;
  InitializerHelper::gram_schmidt(gravity_inI0, R_GtoI0);
  Eigen::Vector4d q_GtoI0 = rot_2_quat(R_GtoI0);
  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, params.gravity_mag;
  std::map<double, Eigen::VectorXd> ori_GtoIi, pos_IiinG, vel_IiinG;
  std::map<size_t, Eigen::Vector3d> features_inG;
  for (auto const &timepair : map_camera_times) {
    ori_GtoIi[timepair.first] = quat_multiply(ori_I0toIi.at(timepair.first), q_GtoI0);
    pos_IiinG[timepair.first] = R_GtoI0.transpose() * pos_IiinI0.at(timepair.first);
    vel_IiinG[timepair.first] = R_GtoI0.transpose() * vel_IiinI0.at(timepair.first);
  }
  for (auto const &feat : features_inI0) {
    features_inG[feat.first] = R_GtoI0.transpose() * feat.second;
  }
  auto rT4a = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // ======================================================

  // Ceres问题相关
  // 注意：默认情况下问题拥有内存的所有权
  ceres::Problem problem;

  // 我们的系统状态（从时间到索引的映射）
  std::map<double, int> map_states;
  std::vector<double *> ceres_vars_ori;  // 姿态
  std::vector<double *> ceres_vars_pos;  // 位置
  std::vector<double *> ceres_vars_vel;  // 速度
  std::vector<double *> ceres_vars_bias_g;  // 陀螺仪偏置
  std::vector<double *> ceres_vars_bias_a;  // 加速度计偏置

  // 特征点状态（3自由度 p_FinG）
  std::map<size_t, int> map_features;
  std::vector<double *> ceres_vars_feat;

  // 设置外参标定 q_ItoC, p_IinC（从相机ID到索引的映射）
  std::map<size_t, int> map_calib_cam2imu;
  std::vector<double *> ceres_vars_calib_cam2imu_ori;
  std::vector<double *> ceres_vars_calib_cam2imu_pos;

  // 设置内参标定 焦距、中心、畸变（从相机ID到索引的映射）
  std::map<size_t, int> map_calib_cam;
  std::vector<double *> ceres_vars_calib_cam_intrinsics;

  // 辅助lambda函数，将释放我们已分配的任何内存
  auto free_state_memory = [&]() {
    for (auto const &ptr : ceres_vars_ori)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_pos)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_vel)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_bias_g)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_bias_a)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_feat)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_calib_cam2imu_ori)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_calib_cam2imu_pos)
      delete[] ptr;
    for (auto const &ptr : ceres_vars_calib_cam_intrinsics)
      delete[] ptr;
  };

  // 设置优化参数
  // 注意：我们使用密集Schur，因为在消除特征点后我们有一个密集问题
  // 注意：http://ceres-solver.org/solving_faqs.html#solving
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  // options.linear_solver_type = ceres::SPARSE_SCHUR;
  // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  // options.preconditioner_type = ceres::SCHUR_JACOBI;
  // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = params.init_dyn_mle_max_threads;
  options.max_solver_time_in_seconds = params.init_dyn_mle_max_time;
  options.max_num_iterations = params.init_dyn_mle_max_iter;
  // options.minimizer_progress_to_stdout = true;
  // options.linear_solver_ordering = ordering;
  options.function_tolerance = 1e-5;
  options.gradient_tolerance = 1e-4 * options.function_tolerance;

  // 遍历每个CPI积分并将其测量添加到问题中
  double timestamp_k = -1;
  for (auto const &timepair : map_camera_times) {

    // 获取请求的相机时间步处的预测状态
    double timestamp_k1 = timepair.first;
    std::shared_ptr<ov_core::CpiV1> cpi = map_camera_cpi_IitoIi1.at(timestamp_k1);
    Eigen::Matrix<double, 16, 1> state_k1;
    state_k1.block(0, 0, 4, 1) = ori_GtoIi.at(timestamp_k1);
    state_k1.block(4, 0, 3, 1) = pos_IiinG.at(timestamp_k1);
    state_k1.block(7, 0, 3, 1) = vel_IiinG.at(timestamp_k1);
    state_k1.block(10, 0, 3, 1) = gyroscope_bias;
    state_k1.block(13, 0, 3, 1) = accelerometer_bias;

    // ================================================================
    //  添加图状态/估计值！
    // ================================================================

    // 将我们的状态变量加载到已分配的状态指针中
    auto *var_ori = new double[4];
    for (int j = 0; j < 4; j++) {
      var_ori[j] = state_k1(0 + j, 0);
    }
    auto *var_pos = new double[3];
    auto *var_vel = new double[3];
    auto *var_bias_g = new double[3];
    auto *var_bias_a = new double[3];
    for (int j = 0; j < 3; j++) {
      var_pos[j] = state_k1(4 + j, 0);
      var_vel[j] = state_k1(7 + j, 0);
      var_bias_g[j] = state_k1(10 + j, 0);
      var_bias_a[j] = state_k1(13 + j, 0);
    }

    // 现在在ceres问题中实际创建参数块
    auto ceres_jplquat = new State_JPLQuatLocal();
    problem.AddParameterBlock(var_ori, 4, ceres_jplquat);
    problem.AddParameterBlock(var_pos, 3);
    problem.AddParameterBlock(var_vel, 3);
    problem.AddParameterBlock(var_bias_g, 3);
    problem.AddParameterBlock(var_bias_a, 3);

    // 固定第一个位姿以约束问题
    // 注意：如果我们不这样做，问题将不是满秩的
    // 注意：由于初始化是在一个小窗口上进行的，我们很可能是退化的
    // 注意：因此我们需要固定这些参数
    if (map_states.empty()) {

      // Construct state and prior
      Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(13, 1);
      for (int j = 0; j < 4; j++) {
        x_lin(0 + j) = var_ori[j];
      }
      for (int j = 0; j < 3; j++) {
        x_lin(4 + j) = var_pos[j];
        x_lin(7 + j) = var_bias_g[j];
        x_lin(10 + j) = var_bias_a[j];
      }
      Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(10, 1);
      Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(10, 10);
      prior_Info.block(0, 0, 4, 4) *= 1.0 / std::pow(1e-5, 2); // 4自由度不可观测的偏航角和位置
      prior_Info.block(4, 4, 3, 3) *= 1.0 / std::pow(0.05, 2); // 陀螺仪偏置先验
      prior_Info.block(7, 7, 3, 3) *= 1.0 / std::pow(0.10, 2); // 加速度计偏置先验

      // Construct state type and ceres parameter pointers
      std::vector<std::string> x_types;
      std::vector<double *> factor_params;
      factor_params.push_back(var_ori);
      x_types.emplace_back("quat_yaw");
      factor_params.push_back(var_pos);
      x_types.emplace_back("vec3");
      factor_params.push_back(var_bias_g);
      x_types.emplace_back("vec3");
      factor_params.push_back(var_bias_a);
      x_types.emplace_back("vec3");

      // Append it to the problem
      auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
      problem.AddResidualBlock(factor_prior, nullptr, factor_params);
    }

    // Append to our historical vector of states
    map_states.insert({timestamp_k1, (int)ceres_vars_ori.size()});
    ceres_vars_ori.push_back(var_ori);
    ceres_vars_pos.push_back(var_pos);
    ceres_vars_vel.push_back(var_vel);
    ceres_vars_bias_g.push_back(var_bias_g);
    ceres_vars_bias_a.push_back(var_bias_a);

    // ================================================================
    //  添加图因子！
    // ================================================================

    // 添加新的IMU因子
    if (cpi != nullptr) {
      assert(timestamp_k != -1);
      std::vector<double *> factor_params;
      factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_bias_g.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_vel.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_bias_a.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k)));
      factor_params.push_back(ceres_vars_ori.at(map_states.at(timestamp_k1)));
      factor_params.push_back(ceres_vars_bias_g.at(map_states.at(timestamp_k1)));
      factor_params.push_back(ceres_vars_vel.at(map_states.at(timestamp_k1)));
      factor_params.push_back(ceres_vars_bias_a.at(map_states.at(timestamp_k1)));
      factor_params.push_back(ceres_vars_pos.at(map_states.at(timestamp_k1)));
      auto *factor_imu = new Factor_ImuCPIv1(cpi->DT, gravity, cpi->alpha_tau, cpi->beta_tau, cpi->q_k2tau, cpi->b_a_lin, cpi->b_w_lin,
                                             cpi->J_q, cpi->J_b, cpi->J_a, cpi->H_b, cpi->H_a, cpi->P_meas);
      problem.AddResidualBlock(factor_imu, nullptr, factor_params);
    }

    // 时间向前移动
    timestamp_k = timestamp_k1;
  }

  // 首先确保已添加标定状态
  for (auto const &idpair : map_camera_ids) {
    size_t cam_id = idpair.first;
    if (map_calib_cam2imu.find(cam_id) == map_calib_cam2imu.end()) {
      auto *var_calib_ori = new double[4];
      for (int j = 0; j < 4; j++) {
        var_calib_ori[j] = params.camera_extrinsics.at(cam_id)(0 + j, 0);
      }
      auto *var_calib_pos = new double[3];
      for (int j = 0; j < 3; j++) {
        var_calib_pos[j] = params.camera_extrinsics.at(cam_id)(4 + j, 0);
      }
      auto ceres_calib_jplquat = new State_JPLQuatLocal();
      problem.AddParameterBlock(var_calib_ori, 4, ceres_calib_jplquat);
      problem.AddParameterBlock(var_calib_pos, 3);
      map_calib_cam2imu.insert({cam_id, (int)ceres_vars_calib_cam2imu_ori.size()});
      ceres_vars_calib_cam2imu_ori.push_back(var_calib_ori);
      ceres_vars_calib_cam2imu_pos.push_back(var_calib_pos);

      // Construct state and prior
      Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(7, 1);
      for (int j = 0; j < 4; j++) {
        x_lin(0 + j) = var_calib_ori[j];
      }
      for (int j = 0; j < 3; j++) {
        x_lin(4 + j) = var_calib_pos[j];
      }
      Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(6, 1);
      Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(6, 6);
      prior_Info.block(0, 0, 3, 3) *= 1.0 / std::pow(0.001, 2);
      prior_Info.block(3, 3, 3, 3) *= 1.0 / std::pow(0.01, 2);

      // Construct state type and ceres parameter pointers
      std::vector<std::string> x_types;
      std::vector<double *> factor_params;
      factor_params.push_back(var_calib_ori);
      x_types.emplace_back("quat");
      factor_params.push_back(var_calib_pos);
      x_types.emplace_back("vec3");
      auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
      problem.AddResidualBlock(factor_prior, nullptr, factor_params);
      if (!params.init_dyn_mle_opt_calib) {
        problem.SetParameterBlockConstant(var_calib_ori);
        problem.SetParameterBlockConstant(var_calib_pos);
      }
    }
    if (map_calib_cam.find(cam_id) == map_calib_cam.end()) {
      auto *var_calib_cam = new double[8];
      for (int j = 0; j < 8; j++) {
        var_calib_cam[j] = params.camera_intrinsics.at(cam_id)->get_value()(j, 0);
      }
      problem.AddParameterBlock(var_calib_cam, 8);
      map_calib_cam.insert({cam_id, (int)ceres_vars_calib_cam_intrinsics.size()});
      ceres_vars_calib_cam_intrinsics.push_back(var_calib_cam);

      // Construct state and prior
      Eigen::MatrixXd x_lin = Eigen::MatrixXd::Zero(8, 1);
      for (int j = 0; j < 8; j++) {
        x_lin(0 + j) = var_calib_cam[j];
      }
      Eigen::MatrixXd prior_grad = Eigen::MatrixXd::Zero(8, 1);
      Eigen::MatrixXd prior_Info = Eigen::MatrixXd::Identity(8, 8);
      prior_Info.block(0, 0, 4, 4) *= 1.0 / std::pow(1.0, 2);
      prior_Info.block(4, 4, 4, 4) *= 1.0 / std::pow(0.005, 2);

      // Construct state type and ceres parameter pointers
      std::vector<std::string> x_types;
      std::vector<double *> factor_params;
      factor_params.push_back(var_calib_cam);
      x_types.emplace_back("vec8");
      auto *factor_prior = new Factor_GenericPrior(x_lin, x_types, prior_Info, prior_grad);
      problem.AddResidualBlock(factor_prior, nullptr, factor_params);
      if (!params.init_dyn_mle_opt_calib) {
        problem.SetParameterBlockConstant(var_calib_cam);
      }
    }
  }
  assert(map_calib_cam2imu.size() == map_calib_cam.size());

  // 然后，添加从所有相机看到的新特征点观测因子
  for (auto const &feat : features) {
    // 跳过测量数据不足的特征点
    if (map_features_num_meas[feat.first] < min_num_meas_to_optimize)
      continue;
    // 如果特征点在相机后面，可以移除！
    if (features_inG.find(feat.first) == features_inG.end())
      continue;
    // 最后遍历每个原始uv观测并将其添加为因子
    for (auto const &camtime : feat.second->timestamps) {

      // 获取我们的ID以及相机是否为鱼眼相机
      size_t feat_id = feat.first;
      size_t cam_id = camtime.first;
      bool is_fisheye = (std::dynamic_pointer_cast<ov_core::CamEqui>(params.camera_intrinsics.at(cam_id)) != nullptr);

      // 遍历每个观测
      for (size_t i = 0; i < camtime.second.size(); i++) {

        // 跳过我们没有位姿的测量数据
        double time = feat.second->timestamps.at(cam_id).at(i);
        if (map_camera_times.find(time) == map_camera_times.end())
          continue;

        // 我们的测量数据
        Eigen::Vector2d uv_raw = feat.second->uvs.at(cam_id).at(i).block(0, 0, 2, 1).cast<double>();

        // 如果我们没有特征点状态，应该创建该参数块
        // 特征点的初始猜测值来自SFM的缩放特征图
        if (map_features.find(feat_id) == map_features.end()) {
          auto *var_feat = new double[3];
          for (int j = 0; j < 3; j++) {
            var_feat[j] = features_inG.at(feat_id)(j);
          }
          problem.AddParameterBlock(var_feat, 3);
          map_features.insert({feat_id, (int)ceres_vars_feat.size()});
          ceres_vars_feat.push_back(var_feat);
        }

        // 然后让我们添加因子
        std::vector<double *> factor_params;
        factor_params.push_back(ceres_vars_ori.at(map_states.at(time)));
        factor_params.push_back(ceres_vars_pos.at(map_states.at(time)));
        factor_params.push_back(ceres_vars_feat.at(map_features.at(feat_id)));
        factor_params.push_back(ceres_vars_calib_cam2imu_ori.at(map_calib_cam2imu.at(cam_id)));
        factor_params.push_back(ceres_vars_calib_cam2imu_pos.at(map_calib_cam2imu.at(cam_id)));
        factor_params.push_back(ceres_vars_calib_cam_intrinsics.at(map_calib_cam.at(cam_id)));
        auto *factor_pinhole = new Factor_ImageReprojCalib(uv_raw, params.sigma_pix, is_fisheye);
        // ceres::LossFunction *loss_function = nullptr;
        ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
        problem.AddResidualBlock(factor_pinhole, loss_function, factor_params);
      }
    }
  }
  assert(ceres_vars_ori.size() == ceres_vars_bias_g.size());
  assert(ceres_vars_ori.size() == ceres_vars_vel.size());
  assert(ceres_vars_ori.size() == ceres_vars_bias_a.size());
  assert(ceres_vars_ori.size() == ceres_vars_pos.size());
  auto rT5 = boost::posix_time::microsec_clock::local_time();

  // 优化ceres图
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  PRINT_INFO("[DynamicInitializer] %d 迭代 | %zu 状态, %zu 特征 (%zu 有效) | %d 参数和 %d 残差 | cost %.4e => %.4e\n",
             (int)summary.iterations.size(), map_states.size(), map_features.size(), count_valid_features, summary.num_parameters,
             summary.num_residuals, summary.initial_cost, summary.final_cost);
  auto rT6 = boost::posix_time::microsec_clock::local_time();

  // 如果失败则返回！
  timestamp = newest_cam_time;
  if (params.init_dyn_mle_max_iter != 0 && summary.termination_type != ceres::CONVERGENCE) {
    PRINT_WARNING(YELLOW "[DynamicInitializer] 动态初始化失败: %s!\n" RESET, summary.message.c_str());
    free_state_memory();
    return false;
  }
  PRINT_INFO("[DynamicInitializer] %s\n", summary.message.c_str());

  //======================================================
  //======================================================

  // 辅助函数：从我们的ceres问题中获取IMU位姿值
  auto get_pose = [&](double timestamp) {
    Eigen::VectorXd state_imu = Eigen::VectorXd::Zero(16);
    for (int i = 0; i < 4; i++) {
      state_imu(0 + i) = ceres_vars_ori[map_states[timestamp]][i];
    }
    for (int i = 0; i < 3; i++) {
      state_imu(4 + i) = ceres_vars_pos[map_states[timestamp]][i];
      state_imu(7 + i) = ceres_vars_vel[map_states[timestamp]][i];
      state_imu(10 + i) = ceres_vars_bias_g[map_states[timestamp]][i];
      state_imu(13 + i) = ceres_vars_bias_a[map_states[timestamp]][i];
    }
    return state_imu;
  };

  // 我们最近的状态是IMU状态！
  assert(map_states.find(newest_cam_time) != map_states.end());
  if (_imu == nullptr) {
    _imu = std::make_shared<ov_type::IMU>();
  }
  Eigen::VectorXd imu_state = get_pose(newest_cam_time);
  _imu->set_value(imu_state);
  _imu->set_fej(imu_state);

  // 添加我们的IMU克隆（包括最近的）
  for (auto const &statepair : map_states) {
    Eigen::VectorXd pose = get_pose(statepair.first);
    if (_clones_IMU.find(statepair.first) == _clones_IMU.end()) {
      auto _pose = std::make_shared<ov_type::PoseJPL>();
      _pose->set_value(pose.block(0, 0, 7, 1));
      _pose->set_fej(pose.block(0, 0, 7, 1));
      _clones_IMU.insert({statepair.first, _pose});
    } else {
      _clones_IMU.at(statepair.first)->set_value(pose.block(0, 0, 7, 1));
      _clones_IMU.at(statepair.first)->set_fej(pose.block(0, 0, 7, 1));
    }
  }

  // 将特征点添加为SLAM特征点！
  for (auto const &featpair : map_features) {
    Eigen::Vector3d feature;
    feature << ceres_vars_feat[featpair.second][0], ceres_vars_feat[featpair.second][1], ceres_vars_feat[featpair.second][2];
    if (_features_SLAM.find(featpair.first) == _features_SLAM.end()) {
      auto _feature = std::make_shared<ov_type::Landmark>(3);
      _feature->_featid = featpair.first;
      _feature->_feat_representation = LandmarkRepresentation::Representation::GLOBAL_3D;
      _feature->set_from_xyz(feature, false);
      _feature->set_from_xyz(feature, true);
      _features_SLAM.insert({featpair.first, _feature});
    } else {
      _features_SLAM.at(featpair.first)->_featid = featpair.first;
      _features_SLAM.at(featpair.first)->_feat_representation = LandmarkRepresentation::Representation::GLOBAL_3D;
      _features_SLAM.at(featpair.first)->set_from_xyz(feature, false);
      _features_SLAM.at(featpair.first)->set_from_xyz(feature, true);
    }
  }

  // 如果我们优化了标定，也应该将其保存到我们的状态中
  if (params.init_dyn_mle_opt_calib) {
    // TODO: 如果我们在进行标定，也要添加我们的标定状态！
    // TODO: （如果我们不进行标定，不要标定它们....）
    // TODO: std::shared_ptr<ov_type::Vec> _calib_dt_CAMtoIMU,
    // TODO: std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>> &_calib_IMUtoCAM,
    // TODO: std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> &_cam_intrinsics
  }

  // 在这里恢复优化状态的协方差
  // 注意：目前只恢复IMU状态，但我们应该能够恢复所有状态
  // 注意：也许有特征点/克隆会使它更稳定？
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  int state_index = map_states[newest_cam_time];
  // 对角线元素
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_ori[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_pos[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_vel[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_bias_g[state_index], ceres_vars_bias_g[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_bias_a[state_index], ceres_vars_bias_a[state_index]));
  // 姿态相关
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_pos[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_vel[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_bias_g[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_ori[state_index], ceres_vars_bias_a[state_index]));
  // 位置相关
  covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_vel[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_bias_g[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_pos[state_index], ceres_vars_bias_a[state_index]));
  // 速度相关
  covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_bias_g[state_index]));
  covariance_blocks.push_back(std::make_pair(ceres_vars_vel[state_index], ceres_vars_bias_a[state_index]));
  // 陀螺仪偏置相关
  covariance_blocks.push_back(std::make_pair(ceres_vars_bias_g[state_index], ceres_vars_bias_a[state_index]));

  // 最后，计算协方差
  ceres::Covariance::Options options_cov;
  options_cov.null_space_rank = (!params.init_dyn_mle_opt_calib) * ((int)map_calib_cam2imu.size() * (6 + 8));
  options_cov.min_reciprocal_condition_number = params.init_dyn_min_rec_cond;
  // options_cov.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
  options_cov.apply_loss_function = true; // Better consistency if we use this
  options_cov.num_threads = params.init_dyn_mle_max_threads;
  ceres::Covariance problem_cov(options_cov);
  bool success = problem_cov.Compute(covariance_blocks, &problem);
  if (!success) {
    PRINT_WARNING(YELLOW "[DynamicInitializer] 动态初始化失败: 协方差恢复失败...\n" RESET);
    free_state_memory();
    return false;
  }

  // 构造我们将返回的协方差
  order.clear();
  order.push_back(_imu);
  covariance = Eigen::MatrixXd::Zero(_imu->size(), _imu->size());
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> covtmp = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();

  // 块对角线
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_ori[state_index], covtmp.data()));
  covariance.block(0, 0, 3, 3) = covtmp.eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_pos[state_index], covtmp.data()));
  covariance.block(3, 3, 3, 3) = covtmp.eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_vel[state_index], covtmp.data()));
  covariance.block(6, 6, 3, 3) = covtmp.eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_g[state_index], ceres_vars_bias_g[state_index], covtmp.data()));
  covariance.block(9, 9, 3, 3) = covtmp.eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_a[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(12, 12, 3, 3) = covtmp.eval();

  // 姿态相关
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_pos[state_index], covtmp.data()));
  covariance.block(0, 3, 3, 3) = covtmp.eval();
  covariance.block(3, 0, 3, 3) = covtmp.transpose();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_vel[state_index], covtmp.data()));
  covariance.block(0, 6, 3, 3) = covtmp.eval();
  covariance.block(6, 0, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_bias_g[state_index], covtmp.data()));
  covariance.block(0, 9, 3, 3) = covtmp.eval();
  covariance.block(9, 0, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_ori[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(0, 12, 3, 3) = covtmp.eval();
  covariance.block(12, 0, 3, 3) = covtmp.transpose().eval();

  // 位置相关
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_vel[state_index], covtmp.data()));
  covariance.block(3, 6, 3, 3) = covtmp.eval();
  covariance.block(6, 3, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_bias_g[state_index], covtmp.data()));
  covariance.block(3, 9, 3, 3) = covtmp.eval();
  covariance.block(9, 3, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_pos[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(3, 12, 3, 3) = covtmp.eval();
  covariance.block(12, 3, 3, 3) = covtmp.transpose().eval();

  // 速度相关
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_bias_g[state_index], covtmp.data()));
  covariance.block(6, 9, 3, 3) = covtmp.eval();
  covariance.block(9, 6, 3, 3) = covtmp.transpose().eval();
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_vel[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(6, 12, 3, 3) = covtmp.eval();
  covariance.block(12, 6, 3, 3) = covtmp.transpose().eval();

  // 陀螺仪偏置相关
  CHECK(problem_cov.GetCovarianceBlockInTangentSpace(ceres_vars_bias_g[state_index], ceres_vars_bias_a[state_index], covtmp.data()));
  covariance.block(9, 12, 3, 3) = covtmp.eval();
  covariance.block(12, 9, 3, 3) = covtmp.transpose().eval();

  // 根据需要膨胀
  covariance.block(0, 0, 3, 3) *= params.init_dyn_inflation_orientation;
  covariance.block(6, 6, 3, 3) *= params.init_dyn_inflation_velocity;
  covariance.block(9, 9, 3, 3) *= params.init_dyn_inflation_bias_gyro;
  covariance.block(12, 12, 3, 3) *= params.init_dyn_inflation_bias_accel;

  // 我们完成了 >:D
  covariance = 0.5 * (covariance + covariance.transpose());
  Eigen::Vector3d sigmas_vel = covariance.block(6, 6, 3, 3).diagonal().transpose().cwiseSqrt();
  Eigen::Vector3d sigmas_bg = covariance.block(9, 9, 3, 3).diagonal().transpose().cwiseSqrt();
  Eigen::Vector3d sigmas_ba = covariance.block(12, 12, 3, 3).diagonal().transpose().cwiseSqrt();
  if (print_debug) {
    PRINT_INFO("[DynamicInitializer] 速度先验 = %.3f, %.3f, %.3f\n", sigmas_vel(0), sigmas_vel(1), sigmas_vel(2));
    PRINT_INFO("[DynamicInitializer] 重力先验 = %.3f, %.3f, %.3f\n", sigmas_bg(0), sigmas_bg(1), sigmas_bg(2));
    PRINT_INFO("[DynamicInitializer] 加速度先验 = %.3f, %.3f, %.3f\n", sigmas_ba(0), sigmas_ba(1), sigmas_ba(2));
  }
  auto rT7 = boost::posix_time::microsec_clock::local_time();

  // 将我们的位置设置为零
  Eigen::MatrixXd x = _imu->value();
  x.block(4, 0, 3, 1).setZero();
  _imu->set_value(x);
  _imu->set_fej(x);

  // 动态初始化各模块耗时（始终打印）
  PRINT_INFO(CYAN "[DynamicInitializer] 各模块耗时:\n" RESET);
  PRINT_INFO("  预检查与数据准备:   %.4f s\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_INFO("  IMU预积分:         %.4f s\n", (rT2a - rT2).total_microseconds() * 1e-6);
  PRINT_INFO("  线性系统构建:      %.4f s\n", (rT3 - rT2a).total_microseconds() * 1e-6);
  PRINT_INFO("  线性系统求解:      %.4f s\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_INFO("  特征恢复与坐标变换: %.4f s\n", (rT4a - rT4).total_microseconds() * 1e-6);
  PRINT_INFO("  Ceres问题构建:     %.4f s\n", (rT5 - rT4a).total_microseconds() * 1e-6);
  PRINT_INFO("  Ceres优化:         %.4f s\n", (rT6 - rT5).total_microseconds() * 1e-6);
  PRINT_INFO("  协方差恢复:        %.4f s\n", (rT7 - rT6).total_microseconds() * 1e-6);
  PRINT_INFO("  总耗时:            %.4f s\n", (rT7 - rT1).total_microseconds() * 1e-6);

  free_state_memory();
  
  PRINT_INFO(CYAN "========================================\n" RESET);
  PRINT_INFO(CYAN "[DynamicInitializer] 动态初始化成功\n" RESET);
  PRINT_INFO(CYAN "========================================\n" RESET);
  return true;
}
