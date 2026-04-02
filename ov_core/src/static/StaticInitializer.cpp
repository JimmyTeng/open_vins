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

#include "StaticInitializer.h"

#include <cmath>

#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/helper.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

size_t StaticInitializer::min_imu_samples_per_window() const { return 2; }

bool StaticInitializer::initialize(double &timestamp,
                                   Eigen::MatrixXd &covariance,
                                   std::vector<std::shared_ptr<Type>> &order,
                                   std::shared_ptr<IMU> t_imu,
                                   bool wait_for_jerk) {
  const double static_window_time = params.init_static_window_time;
  PRINT_INFO(CYAN "========================================\n" RESET);
  PRINT_INFO(CYAN "[静态初始化] 静态初始化开始\n" RESET);
  PRINT_INFO(CYAN "========================================\n" RESET);

  // 如果没有测量值则返回
  if (imu_data->size() < 2) {
    PRINT_INFO(YELLOW
               "[静态初始化] 静态初始化失败: 没有足够的IMU测量值\n" RESET);
    return false;
  }

  // 最新和最旧的IMU时间戳
  double newesttime = imu_data->at(imu_data->size() - 1).timestamp;
  double oldesttime = imu_data->at(0).timestamp;

  // 如果没有足够的数据用于两个窗口则返回
  if (newesttime - oldesttime < static_window_time) {
    PRINT_INFO(YELLOW
               "[静态初始化] 静态初始化失败: 无法选择IMU读数窗口, "
               "没有足够的读数，没有足够的时间窗口\n" RESET);
    return false;
  }

  // 仅使用最近一个静止窗口直接估计动态初始化 bias 初值（对应配置
  // init_dyn_bias_g/init_dyn_bias_a），不再做双窗口/急动判定。
  std::vector<ImuData> static_window;
  for (const ImuData &data : *imu_data) {
    if (data.timestamp > newesttime - static_window_time &&
        data.timestamp <= newesttime) {
      static_window.push_back(data);
    }
  }

  if (static_window.size() < min_imu_samples_per_window()) {
    PRINT_INFO(YELLOW
               "[静态初始化] 静态初始化失败: 窗口选择失败, "
               "无法选择IMU读数窗口, 没有足够的读数\n" RESET);
    return false;
  }

  Eigen::Vector3d a_avg = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_avg = Eigen::Vector3d::Zero();
  for (const ImuData &data : static_window) {
    a_avg += data.am;
    w_avg += data.wm;
  }
  a_avg /= static_cast<double>(static_window.size());
  w_avg /= static_cast<double>(static_window.size());

  // 获取z轴与-g对齐的旋转（z_in_G=0,0,1）
  Eigen::Vector3d z_axis = a_avg / a_avg.norm();
  Eigen::Matrix3d Ro;
  InitializerHelper::gram_schmidt(z_axis, Ro);
  Eigen::Vector4d q_GtoI = rot_2_quat(Ro);

  // 静态初始化阶段将重力模长视为常数，不在此阶段估计。
  double gravity_mag_used = params.gravity_mag;
  if (!std::isfinite(gravity_mag_used) || gravity_mag_used <= 0.0) {
    PRINT_WARNING(YELLOW
                  "[静态初始化] 配置重力模长无效(%.6f), 回退到默认值 9.810000\n"
                  RESET,
                  gravity_mag_used);
    gravity_mag_used = 9.81;
  }

  // 将我们的偏置设置为等于我们的噪声（从加速度计偏置中减去我们的重力）
  Eigen::Vector3d gravity_inG;
  gravity_inG << 0.0, 0.0, gravity_mag_used;
  Eigen::Vector3d bg = w_avg;
  Eigen::Vector3d ba = a_avg - quat_2_Rot(q_GtoI) * gravity_inG;

  // 设置我们的状态变量
  timestamp = static_window.back().timestamp;
  Eigen::VectorXd imu_state = Eigen::VectorXd::Zero(16);
  imu_state.block(0, 0, 4, 1) = q_GtoI;
  imu_state.block(10, 0, 3, 1) = bg;
  imu_state.block(13, 0, 3, 1) = ba;
  assert(t_imu != nullptr);
  t_imu->set_value(imu_state);
  t_imu->set_fej(imu_state);

  // 创建基础协方差及其协方差排序
  order.clear();
  order.push_back(t_imu);
  covariance = std::pow(0.02, 2) *
               Eigen::MatrixXd::Identity(t_imu->size(), t_imu->size());
  covariance.block(0, 0, 3, 3) =
      std::pow(0.02, 2) * Eigen::Matrix3d::Identity();  // q
  covariance.block(3, 3, 3, 3) =
      std::pow(0.05, 2) * Eigen::Matrix3d::Identity();  // p
  covariance.block(6, 6, 3, 3) =
      std::pow(0.01, 2) * Eigen::Matrix3d::Identity();  // v (static)

  // VIO系统有4个自由度的不可观测方向，可以任意选择。
  // 这意味着在启动时，我们可以将偏航角和位置固定为100%已知。
  // TODO: 为什么我们不能将这些设置为零并获得良好的NEES真实世界结果？
  // 因此，在确定初始化后从全局到当前IMU朝向之后，我们可以将全局误差传播
  // 到新的IMU位姿中。在这种情况下，位置是直接等价的，但朝向需要传播。
  // 我们将全局朝向传播到当前局部IMU坐标系
  // R_GtoI = R_GtoI*R_GtoG -> H = R_GtoI
  // Eigen::Matrix3d R_GtoI = quat_2_Rot(q_GtoI);
  // covariance(2, 2) = std::pow(1e-4, 2);
  // covariance.block(0, 0, 3, 3) = R_GtoI * covariance.block(0, 0, 3, 3) *
  // R_GtoI.transpose(); covariance.block(3, 3, 3, 3) = std::pow(1e-3, 2) *
  // Eigen::Matrix3d::Identity();

  PRINT_INFO(CYAN "========================================\n" RESET);
  PRINT_INFO(CYAN "  [静态初始化] 静态初始化成功\n" RESET);
  PRINT_INFO(CYAN "  [静态初始化] 使用固定重力模长 |g| = %.6f m/s^2\n" RESET,
             gravity_mag_used);
  PRINT_INFO(CYAN
             "  [静态初始化] 估计 bias 初值 bg=(%.6f, %.6f, %.6f), "
             "ba=(%.6f, %.6f, %.6f)\n" RESET,
             bg(0), bg(1), bg(2), ba(0), ba(1), ba(2));
  PRINT_INFO(CYAN "========================================\n" RESET);

  // 返回 :D
  return true;
}
