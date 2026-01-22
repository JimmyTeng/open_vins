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

#include "utils/helper.h"

#include "feat/FeatureHelper.h"
#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

size_t StaticInitializer::min_imu_samples_per_window() const { return 2; }

bool StaticInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<Type>> &order,
                                   std::shared_ptr<IMU> t_imu, bool wait_for_jerk) {

  PRINT_INFO(CYAN "========================================\n" RESET);
  PRINT_INFO(CYAN "[静态初始化] 静态初始化开始\n" RESET);
  PRINT_INFO(CYAN "========================================\n" RESET);

  // 如果没有测量值则返回
  if (imu_data->size() < 2) {
    PRINT_INFO(YELLOW "[静态初始化] 静态初始化失败: 没有足够的IMU测量值\n" RESET);
    return false;
  }

  // 最新和最旧的IMU时间戳
  double newesttime = imu_data->at(imu_data->size() - 1).timestamp;
  double oldesttime = imu_data->at(0).timestamp;

  // 如果没有足够的数据用于两个窗口则返回
  if (newesttime - oldesttime < params.init_window_time) {
    PRINT_INFO(YELLOW "[静态初始化] 静态初始化失败: 无法选择IMU读数窗口, 没有足够的读数，没有足够的时间窗口\n" RESET);
    return false;
  }

  // 首先让我们从最新测量到最旧测量收集一个IMU读数窗口
  // 这里把数据分成两个窗口，一个窗口是一半窗口时间，一个窗口是整个窗口时间，窗口时间可以根据需要调整
  // 一半窗口时间用于计算加速度的平均值 整个窗口时间用于计算加速度的方差

  std::vector<ImuData> window_1to0, window_2to1;
  for (const ImuData &data : *imu_data) {
    if (data.timestamp > newesttime - 0.5 * params.init_window_time && data.timestamp <= newesttime - 0.0 * params.init_window_time) {
      // 一半窗口时间用于计算加速度的平均值 t∈[newesttime - 0.5 * params.init_window_time, newesttime - 0.0 * params.init_window_time]
      window_1to0.push_back(data);
    }
    if (data.timestamp > newesttime - 1.0 * params.init_window_time && data.timestamp <= newesttime - 0.5 * params.init_window_time) {
      // 整个窗口时间用于计算加速度的方差
      window_2to1.push_back(data);
    }
  }

  // 如果这两个都失败则返回
  
  if (window_1to0.size() < min_imu_samples_per_window() || window_2to1.size() < min_imu_samples_per_window()) {
    PRINT_INFO(YELLOW "[静态初始化] 静态初始化失败: 窗口选择失败, 无法选择IMU读数窗口, 没有足够的读数\n" RESET);
    return false;
  }

  // 计算从1到0的最新窗口的样本方差
  Eigen::Vector3d a_avg_1to0 = Eigen::Vector3d::Zero();
  for (const ImuData &data : window_1to0) {
    a_avg_1to0 += data.am;
  }
  a_avg_1to0 /= (int)window_1to0.size();
  double a_var_1to0 = 0;
  for (const ImuData &data : window_1to0) {
    a_var_1to0 += (data.am - a_avg_1to0).dot(data.am - a_avg_1to0);
  }
  a_var_1to0 = std::sqrt(a_var_1to0 / ((int)window_1to0.size() - 1));

  // 计算从2到1的第二新窗口的样本方差
  Eigen::Vector3d a_avg_2to1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_avg_2to1 = Eigen::Vector3d::Zero();
  for (const ImuData &data : window_2to1) {
    a_avg_2to1 += data.am;
    w_avg_2to1 += data.wm;
  }
  a_avg_2to1 = a_avg_2to1 / window_2to1.size();
  w_avg_2to1 = w_avg_2to1 / window_2to1.size();
  double a_var_2to1 = 0;
  for (const ImuData &data : window_2to1) {
    a_var_2to1 += (data.am - a_avg_2to1).dot(data.am - a_avg_2to1);
  }
  a_var_2to1 = std::sqrt(a_var_2to1 / ((int)window_2to1.size() - 1));
  if (print_debug) {
    PRINT_INFO(YELLOW "  [静态初始化] IMU 激励标准差统计: 阶段2到1:%.3f, 阶段1到0:%.3f\n" RESET, a_var_2to1, a_var_1to0);
  }

  // 情况1: 如果最新窗口低于阈值并且我们想等待直到检测到急动
  // 说明：当前仍静止，等待急动模式需要检测到运动开始才能初始化
  // wait_for_jerk=true: 系统未启用ZUPT，需要等待急动(从静止到运动的跳变)来确认设备开始运动
  if (a_var_1to0 < params.init_imu_thresh && wait_for_jerk) {
    PRINT_INFO(YELLOW "  [静态初始化] 静态初始化失败[情况1-等待急动模式(wait_for_jerk=true,未启用ZUPT)]: 最新窗口仍静止(方差%.3f < 阈值%.3f), 需要等待急动(运动开始)才能初始化\n" RESET, a_var_1to0, params.init_imu_thresh);
    return false;
  }

  // 情况2: 我们还应该检查旧状态是否高于阈值！
  // 这是当我们启动时已经在移动的情况，因此我们需要等待一段静止运动
  // 说明：旧窗口有运动，等待急动模式下需要先静止才能初始化
  // wait_for_jerk=true: 系统未启用ZUPT，需要先静止再检测到急动才能初始化
  if (a_var_2to1 > params.init_imu_thresh && wait_for_jerk) {
    PRINT_INFO(YELLOW "  [静态初始化] 静态初始化失败[情况2-等待急动模式(wait_for_jerk=true,未启用ZUPT)]: 旧窗口检测到运动(方差%.3f > 阈值%.3f), 需要等待静止才能初始化\n" RESET, a_var_2to1, params.init_imu_thresh);
    return false;
  }

  // 情况3: 如果任一窗口高于阈值并且我们不等待急动
  // 那么我们不是静止的（即正在移动），所以我们应该等待直到静止
  // 说明：检测到运动，非等待急动模式下需要静止才能初始化
  // wait_for_jerk=false: 系统已启用ZUPT，可在静止状态下立即初始化，但当前检测到运动需要等待静止
  if ((a_var_1to0 > params.init_imu_thresh || a_var_2to1 > params.init_imu_thresh) && !wait_for_jerk) {
    PRINT_INFO(YELLOW "  [静态初始化] 静态初始化失败[情况3-非等待急动模式(wait_for_jerk=false,已启用ZUPT)]: 检测到运动(方差阶段2到1:%.3f, 阶段1到0:%.3f > 阈值%.3f), 需要等待静止才能初始化\n" RESET, a_var_2to1, a_var_1to0,
               params.init_imu_thresh);
    return false;
  }

  // 获取z轴与-g对齐的旋转（z_in_G=0,0,1）
  Eigen::Vector3d z_axis = a_avg_2to1 / a_avg_2to1.norm();
  Eigen::Matrix3d Ro;
  InitializerHelper::gram_schmidt(z_axis, Ro);
  Eigen::Vector4d q_GtoI = rot_2_quat(Ro);

  // 将我们的偏置设置为等于我们的噪声（从加速度计偏置中减去我们的重力）
  Eigen::Vector3d gravity_inG;
  gravity_inG << 0.0, 0.0, params.gravity_mag;
  Eigen::Vector3d bg = w_avg_2to1;
  Eigen::Vector3d ba = a_avg_2to1 - quat_2_Rot(q_GtoI) * gravity_inG;

  // 设置我们的状态变量
  timestamp = window_2to1.at(window_2to1.size() - 1).timestamp;
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
  covariance = std::pow(0.02, 2) * Eigen::MatrixXd::Identity(t_imu->size(), t_imu->size());
  covariance.block(0, 0, 3, 3) = std::pow(0.02, 2) * Eigen::Matrix3d::Identity(); // q
  covariance.block(3, 3, 3, 3) = std::pow(0.05, 2) * Eigen::Matrix3d::Identity(); // p
  covariance.block(6, 6, 3, 3) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity(); // v (static)

  // VIO系统有4个自由度的不可观测方向，可以任意选择。
  // 这意味着在启动时，我们可以将偏航角和位置固定为100%已知。
  // TODO: 为什么我们不能将这些设置为零并获得良好的NEES真实世界结果？
  // 因此，在确定初始化后从全局到当前IMU朝向之后，我们可以将全局误差传播
  // 到新的IMU位姿中。在这种情况下，位置是直接等价的，但朝向需要传播。
  // 我们将全局朝向传播到当前局部IMU坐标系
  // R_GtoI = R_GtoI*R_GtoG -> H = R_GtoI
  // Eigen::Matrix3d R_GtoI = quat_2_Rot(q_GtoI);
  // covariance(2, 2) = std::pow(1e-4, 2);
  // covariance.block(0, 0, 3, 3) = R_GtoI * covariance.block(0, 0, 3, 3) * R_GtoI.transpose();
  // covariance.block(3, 3, 3, 3) = std::pow(1e-3, 2) * Eigen::Matrix3d::Identity();

  PRINT_INFO(CYAN "========================================\n" RESET);
  PRINT_INFO(CYAN "  [静态初始化] 静态初始化成功\n" RESET);
  PRINT_INFO(CYAN "========================================\n" RESET);

  // 返回 :D
  return true;
}
