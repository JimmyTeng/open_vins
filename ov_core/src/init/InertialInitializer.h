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

#ifndef OV_INIT_INERTIALINITIALIZER_H
#define OV_INIT_INERTIALINITIALIZER_H

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

class StaticInitializer;
class DynamicInitializer;

/**
 * @brief 视觉-惯性系统初始化器
 *
 * 该类将尝试对系统状态进行动态和静态初始化。
 * 用户可以请求等待IMU读数中的"抖动"（例如设备被拿起）或尽快初始化。
 * 对于静态初始化，用户需要事先指定标定参数，否则总是使用动态初始化。
 * 初始化逻辑如下：
 * 1. 尝试执行状态元素的动态初始化
 * 2. 如果失败且我们有标定参数，则可以尝试静态初始化
 * 3. 如果设备静止且我们正在等待抖动，则直接返回，否则初始化状态！
 *
 * 动态系统基于并扩展了以下工作：[Estimator initialization in vision-aided inertial navigation
 * with unknown camera-IMU calibration](https://ieeexplore.ieee.org/document/6386235) @cite Dong2012IROS，
 * 该工作通过首先创建线性系统来恢复相机到IMU的旋转，然后恢复速度、重力和特征位置，
 * 最后进行完整优化以允许协方差恢复来解决初始化问题。
 * 另一篇可能对读者有用的论文是 [An Analytical Solution to the IMU Initialization
 * Problem for Visual-Inertial Systems](https://ieeexplore.ieee.org/abstract/document/9462400)，
 * 该论文对尺度恢复和加速度计偏置进行了详细的实验。
 */
class InertialInitializer {

public:
  /**
   * @brief 默认构造函数
   * @param params_ 从ROS或命令行加载的初始化参数
   * @param db 包含所有特征的特征跟踪器数据库
   */
  explicit InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db);

  /**
   * @brief 输入惯性数据的函数
   * @param message 包含时间戳和惯性信息的消息
   * @param oldest_time 可以丢弃此时间之前的测量数据，-1表示不丢弃
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1);

  /**
   * @brief 尝试初始化系统
   *
   * @m_class{m-note m-warning}
   *
   * @par 处理成本
   * 这是一个串行过程，可能需要数秒才能完成。
   * 如果您是实时应用程序，则可能希望从异步线程调用此函数，
   * 以便在后台处理。使用的特征是从特征数据库克隆的，因此应该是线程安全的，
   * 可以继续向数据库追加新的特征轨迹。
   *
   * @param[out] timestamp 初始化状态的时间戳
   * @param[out] covariance 返回状态的协方差矩阵
   * @param[out] order 协方差矩阵的顺序
   * @param[out] t_imu IMU类型对象（需要具有正确的ID）
   * @param wait_for_jerk 如果为true，将等待"抖动"（设备被拿起等动作）
   * @return 如果成功初始化系统则返回true
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk = true);

protected:
  bool print_debug = false;
  /// 初始化参数
  InertialInitializerOptions params;

  /// 包含所有特征的特征跟踪器数据库
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// IMU消息历史记录（时间、角速度、线加速度）
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;

  /// 静态初始化辅助类
  std::shared_ptr<StaticInitializer> init_static;

  /// 动态初始化辅助类
  std::shared_ptr<DynamicInitializer> init_dynamic;
};

} // namespace ov_init

#endif // OV_INIT_INERTIALINITIALIZER_H
