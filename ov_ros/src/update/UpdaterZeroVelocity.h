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

#ifndef OV_MSCKF_UPDATER_ZEROVELOCITY_H
#define OV_MSCKF_UPDATER_ZEROVELOCITY_H

#include <memory>

#include "utils/sensor_data.h"

#include "UpdaterOptions.h"
#include "utils/NoiseManager.h"

namespace ov_core {
class Feature;
class FeatureDatabase;
} // namespace ov_core
namespace ov_type {
class Landmark;
} // namespace ov_type

namespace ov_msckf {

class State;
class Propagator;

/**
 * @brief 尝试检测并使用零速度假设进行更新
 *
 * 考虑VIO单元在一段时间内保持静止的情况。
 * 通常这会在没有SLAM特征的单目系统中引起问题，因为无法对特征进行三角化。
 * 此外，即使特征可以被三角化（例如立体视觉），质量也可能很差并影响性能。
 * 如果我们能够检测到静止的情况，就可以利用这一点来避免在此期间进行特征更新。
 * 主要应用场景是在需要停止的**轮式车辆**上（例如红绿灯或停车）。
 */
class UpdaterZeroVelocity {

public:
  /**
   * @brief 零速度检测器和更新器的默认构造函数
   * @param options 更新器选项（chi2乘数）
   * @param noises IMU噪声特性（连续时间）
   * @param db 包含所有特征的特征跟踪器数据库
   * @param prop 可以在时间上向前预测状态的传播器类对象
   * @param gravity_mag 系统的全局重力大小（通常为9.81）
   * @param zupt_max_velocity 进行更新时应考虑的最大速度
   * @param zupt_noise_multiplier IMU噪声矩阵的乘数（默认应为1.0）
   * @param zupt_max_disparity 进行更新时应考虑的最大视差
   */
  UpdaterZeroVelocity(UpdaterOptions &options, NoiseManager &noises, std::shared_ptr<ov_core::FeatureDatabase> db,
                      std::shared_ptr<Propagator> prop, double gravity_mag, double zupt_max_velocity, double zupt_noise_multiplier,
                      double zupt_max_disparity);

  /**
   * @brief 惯性数据的输入函数
   * @param message 包含时间戳和惯性信息的消息
   * @param oldest_time 可以丢弃此时间之前的测量值
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1);

  /**
   * @brief 移除比给定测量时间更早的任何IMU测量值
   * @param oldest_time 可以丢弃此时间之前的测量值（IMU时钟）
   */
  void clean_old_imu_measurements(double oldest_time);

  /**
   * @brief 首先检测系统是否为零速度，然后进行更新
   * @param state 滤波器的状态
   * @param timestamp 下一个相机时间戳，用于判断是否应该传播到该时间
   * @return 如果系统当前处于零速度则返回true
   */
  bool try_update(std::shared_ptr<State> state, double timestamp);

protected:

  bool print_debug = false;
  /// 更新期间使用的选项（chi2乘数）
  UpdaterOptions _options;

  /// IMU噪声值的容器
  NoiseManager _noises;

  /// 包含所有特征的特征跟踪器数据库
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// 状态传播器
  std::shared_ptr<Propagator> _prop;

  /// 重力向量
  Eigen::Vector3d _gravity;

  /// 进行零速度更新时应考虑的最大速度（m/s）
  double _zupt_max_velocity = 1.0;

  /// IMU噪声矩阵的乘数（默认应为1.0）
  double _zupt_noise_multiplier = 1.0;

  /// 进行零速度更新时应考虑的最大视差（像素）
  double _zupt_max_disparity = 1.0;

  /// 卡方分布95百分位表（查找键为残差的大小）
  std::map<int, double> chi_squared_table;

  /// IMU消息历史记录（时间、角速度、线加速度）
  std::vector<ov_core::ImuData> imu_data;

  /// 上次传播时间的偏移估计
  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  /// 上次进行零速度更新的时间戳
  double last_zupt_state_timestamp = 0.0;

  /// 调用更新的次数
  int last_zupt_count = 0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_ZEROVELOCITY_H
