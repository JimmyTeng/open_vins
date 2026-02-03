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

#ifndef OV_INIT_STATICINITIALIZER_H
#define OV_INIT_STATICINITIALIZER_H

#include <cstddef>

#include "init/InertialInitializerOptions.h"

namespace ov_core {
class FeatureDatabase;
struct ImuData;
} // namespace ov_core
namespace ov_type {
class Type;
class IMU;
} // namespace ov_type

namespace ov_init {

/**
 * @brief 静态视觉惯性系统的初始化器
 *
 * 此实现假设IMU从静止状态开始。
 * 从静止状态初始化的步骤：
 * 1. 收集所有惯性测量值
 * 2. 检查在最后一个窗口内是否有加速度的跳变
 * 3. 如果跳变超过我们的阈值，则应该初始化（即我们已经开始移动）
 * 4. 使用*前一个*窗口，该窗口应该是静止的，用于初始化朝向
 * 5. 返回与重力对齐的横滚角和俯仰角以及偏置。
 *
 */
class StaticInitializer {

public:
  /**
   * @brief 默认构造函数
   * @param params_ 从ROS或CMDLINE加载的参数
   * @param db 包含所有特征的特征跟踪器数据库
   * @param imu_data_ 指向我们的IMU历史信息向量的共享指针
   */
  explicit StaticInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db,
                             std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_)
      : params(params_), _db(db), imu_data(imu_data_) {}

  /**
   * @brief 尝试仅使用IMU获取初始化的系统
   *
   * 这将检查我们是否在加速度中有足够大的跳变。
   * 如果有，我们将使用跳变之前的时间段来初始化状态。
   * 这假设我们的IMU处于静止状态且不移动（因此如果我们经历恒定加速度，这将失败）。
   *
   * 如果我们不等待跳变（即`wait_for_jerk`为false），则系统将尽快尝试初始化。
   * 这仅在您启用了零速度更新来处理静止情况时推荐使用。
   * 在这种情况下初始化，我们需要使平均角方差低于设定的阈值（即我们需要处于静止状态）。
   *
   * @param[out] timestamp 我们初始化状态的时间戳
   * @param[out] covariance 返回状态的已计算协方差
   * @param[out] order 协方差矩阵的顺序
   * @param[out] t_imu 我们的IMU类型元素
   * @param wait_for_jerk 如果为true，我们将等待"急动"
   * @return 如果我们成功初始化了系统则返回true
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk = true);

private:
  bool print_debug = false;
  /// 初始化参数
  InertialInitializerOptions params;

  /// 包含所有特征的特征跟踪器数据库
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// 我们的IMU消息历史记录（时间、角速度、线速度）
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;

  /// 每个 IMU 窗口所需的最少样本数（计算样本方差至少需要 2 个样本）
  size_t min_imu_samples_per_window() const;
};

} // namespace ov_init

#endif // OV_INIT_STATICINITIALIZER_H
