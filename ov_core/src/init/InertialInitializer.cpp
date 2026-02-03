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

#include "InertialInitializer.h"

#ifndef __ANDROID__
#include "dynamic/DynamicInitializer.h"
#endif
#include "static/StaticInitializer.h"

#include "feat/FeatureHelper.h"
#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

InertialInitializer::InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db)
    : params(params_), _db(db) {

  // 创建IMU数据向量
  imu_data = std::make_shared<std::vector<ov_core::ImuData>>();

  // 创建初始化器
  // 静态初始化器：用于设备静止时的初始化
  init_static = std::make_shared<StaticInitializer>(params, _db, imu_data);
#ifndef __ANDROID__
  // 动态初始化器：用于设备运动时的初始化（Android平台不支持，因为需要Ceres Solver）
  init_dynamic = std::make_shared<DynamicInitializer>(params, _db, imu_data);
#else
  init_dynamic = nullptr;
#endif
}

void InertialInitializer::feed_imu(const ov_core::ImuData &message, double oldest_time) {

  // 将IMU数据添加到向量中
  imu_data->emplace_back(message);

  // 对IMU数据进行排序（处理任何乱序的测量数据）
  // std::sort(imu_data->begin(), imu_data->end(), [](const IMUDATA i, const IMUDATA j) {
  //    return i.timestamp < j.timestamp;
  //});

  // 遍历并删除早于请求时间的IMU消息
  // std::cout << "INIT: imu_data.size() " << imu_data->size() << std::endl;
  if (oldest_time != -1) {
    auto it0 = imu_data->begin();
    while (it0 != imu_data->end()) {
      if (it0->timestamp < oldest_time) {
        // 删除过旧的IMU数据
        it0 = imu_data->erase(it0);
      } else {
        it0++;
      }
    }
  }
}

bool InertialInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                                     std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk) {

  // 获取我们将尝试初始化的最新和最旧时间戳
  double newest_cam_time = -1;
  // 遍历特征数据库，找到最新的相机时间戳
  for (auto const &feat : _db->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        newest_cam_time = std::max(newest_cam_time, time);
      }
    }
  }
  // 计算初始化窗口的最旧时间（减去窗口时间和额外缓冲时间0.10秒）
  double oldest_time = newest_cam_time - params.init_window_time - 0.10;
  if (newest_cam_time < 0 || oldest_time < 0) {
    return false;
  }

  // 删除初始化窗口之前的所有测量数据
  // 然后我们将尝试使用特征数据库中的所有特征！
  _db->cleanup_measurements(oldest_time);
  // 删除过旧的IMU数据（考虑相机-IMU时间偏移）
  auto it_imu = imu_data->begin();
  while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + params.calib_camimu_dt) {
    it_imu = imu_data->erase(it_imu);
  }

  // 计算当前时间步系统的视差（用于判断设备是否在运动）
  // 如果视差为零或负值，我们将始终使用静态初始化器
  bool disparity_detected_moving_1to0 = false;  // 前半段窗口是否检测到运动
  bool disparity_detected_moving_2to1 = false;  // 后半段窗口是否检测到运动
  if (params.init_max_disparity > 0) {

    // 获取从当前图像到前一图像的视差统计信息
    // 仅计算初始化周期最旧一半的视差
    double newest_time_allowed = newest_cam_time - 0.5 * params.init_window_time;
    int num_features0 = 0;  // 前半段窗口的特征数量
    int num_features1 = 0;  // 后半段窗口的特征数量
    double avg_disp0, avg_disp1;  // 平均视差
    double var_disp0, var_disp1;  // 视差方差
    // 计算前半段窗口的视差（从最旧时间到中间时间）
    FeatureHelper::compute_disparity(_db, avg_disp0, var_disp0, num_features0, newest_time_allowed);
    // 计算后半段窗口的视差（从中间时间到最新时间）
    FeatureHelper::compute_disparity(_db, avg_disp1, var_disp1, num_features1, newest_cam_time, newest_time_allowed);

    // 如果无法计算视差则返回
    int feat_thresh = 15;  // 特征数量阈值
    if (num_features0 < feat_thresh || num_features1 < feat_thresh) {
      PRINT_INFO(YELLOW "[InertialInitializer]:初始化失败, 视差计算所需特征不足: min(前段特征 %d, 后段特征 %d) < %d\n" RESET, num_features0, num_features1, feat_thresh);
      return false;
    }
    if (print_debug) {
    // 检查是否通过我们的检查！
      PRINT_DEBUG(YELLOW "[InertialInitializer]: 前段视差 %.3f, 后段视差 %.3f (%.2f 阈值)\n" RESET, avg_disp0, avg_disp1, params.init_max_disparity);
    }
    // 判断两个时间段是否检测到运动（视差超过阈值）
    disparity_detected_moving_1to0 = (avg_disp0 > params.init_max_disparity);
    disparity_detected_moving_2to1 = (avg_disp1 > params.init_max_disparity);
  }

  // 使用静态初始化器！
  // 情况1：如果视差显示我们在上一个窗口是静止的，而在最新窗口有运动，则检测到抖动（jerk）
  // 情况2：如果两个视差都低于阈值，则平台在这两个时间段都是静止的
  bool has_jerk = (!disparity_detected_moving_1to0 && disparity_detected_moving_2to1);  // 检测到抖动：从静止到运动
  bool is_still = (!disparity_detected_moving_1to0 && !disparity_detected_moving_2to1);  // 完全静止：两个时间段都静止
  if (print_debug) {
    PRINT_DEBUG(YELLOW "[InertialInitializer]: 检测到抖动: %d, 完全静止: %d\n" RESET, has_jerk, is_still);
  }
    // 使用静态初始化器的条件：
  // 1. 检测到抖动且等待抖动，或
  // 2. 完全静止且不等待抖动
  // 并且IMU阈值大于0
  if (((has_jerk && wait_for_jerk) || (is_still && !wait_for_jerk)) && params.init_imu_thresh > 0.0) {
    return init_static->initialize(timestamp, covariance, order, t_imu, wait_for_jerk);
  } else if (params.init_dyn_use && !is_still) {
    // 如果启用动态初始化且平台不是静止的，则使用动态初始化器
#ifndef __ANDROID__ // Android平台不支持动态初始化，因为需要Ceres Solver
    if (init_dynamic) {
      std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;  // IMU位姿克隆
      std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;  // SLAM特征
      return init_dynamic->initialize(timestamp, covariance, order, t_imu, _clones_IMU, _features_SLAM);
    } else {
      PRINT_ERROR(RED "[InertialInitializer]: 动态初始化器不可用 (Ceres Solver未包含)\n" RESET);
    }
#else
    PRINT_ERROR(RED "[InertialInitializer]: DYNAMIC INITIALIZER not available on Android (Ceres Solver not included)\n" RESET);
#endif
  } else {
    // 初始化失败，生成错误消息
    std::string msg = (has_jerk) ? "" : " 未检测到加速度抖动";  // 未检测到加速度抖动
    msg += (has_jerk || is_still) ? "" : ", ";
    msg += (is_still) ? "" : " 平台运动过多";  // 平台运动过多
    PRINT_INFO(YELLOW "[InertialInitializer]: 静态初始化失败: %s\n" RESET, msg.c_str());
  }
  return false;
}
