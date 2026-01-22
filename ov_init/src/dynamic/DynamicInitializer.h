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

#ifndef OV_INIT_DYNAMICINITIALIZER_H
#define OV_INIT_DYNAMICINITIALIZER_H

#include "init/InertialInitializerOptions.h"

namespace ov_core {
class FeatureDatabase;
struct ImuData;
} // namespace ov_core
namespace ov_type {
class Type;
class IMU;
class PoseJPL;
class Landmark;
class Vec;
} // namespace ov_type

namespace ov_init {

/**
 * @brief 动态视觉-惯性系统的初始化器
 *
 * 该实现将尝试恢复系统的初始条件。
 * 此外，我们还将尝试恢复系统的协方差矩阵。
 * 使用任意运动进行初始化的步骤：
 * 1. 预积分系统以获得相对旋转变化（假设偏置已知）
 * 2. 构建包含特征点的线性系统以恢复速度（使用|g|约束求解）
 * 3. 对所有标定参数执行最大似然估计(MLE)并恢复协方差矩阵
 *
 * 该方法基于以下工作（详细技术报告请参见 [tech report](https://pgeneva.com/downloads/reports/tr_init.pdf)）：
 *
 * > Dong-Si, Tue-Cuong, and Anastasios I. Mourikis.
 * > "Estimator initialization in vision-aided inertial navigation with unknown camera-IMU calibration."
 * > 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2012.
 *
 * - https://ieeexplore.ieee.org/abstract/document/6386235
 * - https://tdongsi.github.io/download/pubs/2011_VIO_Init_TR.pdf
 * - https://pgeneva.com/downloads/reports/tr_init.pdf
 *
 */
class DynamicInitializer {
public:
  /**
   * @brief 默认构造函数
   * @param params_ 从ROS或命令行加载的参数
   * @param db 包含所有特征点的特征跟踪器数据库
   * @param imu_data_ 指向IMU历史数据向量的共享指针
   */
  explicit DynamicInitializer(const InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db,
                              std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_)
      : params(params_), _db(db), imu_data(imu_data_) {}

  /**
   * @brief 尝试获取初始化后的系统
   *
   * @param[out] timestamp 初始化状态的时间戳（最后一个IMU状态）
   * @param[out] covariance 计算得到的返回状态的协方差矩阵
   * @param[out] order 协方差矩阵的顺序
   * @param _imu 指向"活跃"IMU状态的指针 (q_GtoI, p_IinG, v_IinG, bg, ba)
   * @param _clones_IMU 成像时间与克隆位姿之间的映射 (q_GtoIi, p_IiinG)
   * @param _features_SLAM 当前的SLAM特征点集合（3D位置）
   * @return 如果成功初始化系统则返回true
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> &_imu, std::map<double, std::shared_ptr<ov_type::PoseJPL>> &_clones_IMU,
                  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> &_features_SLAM);

private:
  bool print_debug = false;

  /// 初始化参数
  InertialInitializerOptions params;

  /// 包含所有特征点的特征跟踪器数据库
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// IMU消息历史记录（时间、角速度、线加速度）
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;

};

} // namespace ov_init

#endif // OV_INIT_DYNAMICINITIALIZER_H
