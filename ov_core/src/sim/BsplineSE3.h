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

#ifndef OV_CORE_BSPLINESE3_H
#define OV_CORE_BSPLINESE3_H

#include <Eigen/Eigen>
#include <map>
#include <vector>

#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_core {

/**
 * @brief 在SE(3)流形上执行插值的B样条
 *
 * 此类实现了b样条功能，允许在\f$\mathbb{SE}(3)\f$流形上进行插值。
 * 这基于[Continuous-Time Visual-Inertial Odometry for Event
 * Cameras](https://ieeexplore.ieee.org/abstract/document/8432102/)和[A Spline-Based Trajectory Representation for Sensor Fusion and
 * Rolling Shutter Cameras](https://link.springer.com/article/10.1007/s11263-015-0811-3)中的推导，一些额外的推导可在
 * [these notes](http://udel.edu/~pgeneva/downloads/notes/2018_notes_mueffler2017arxiv.pdf)中找到。使用b样条进行\f$\mathbb{SE}(3)\f$
 * 插值具有以下特性：
 *
 * 1. 局部控制，允许系统在线和批处理模式下运行
 * 2. \f$C^2\f$连续性，以支持惯性预测和计算
 * 3. 对最小力矩轨迹的良好近似
 * 4. 无奇点的刚体运动参数化
 *
 * 关键思想是将一组轨迹点转换为连续时间*均匀三次累积*b样条。
 * 与标准b样条表示相比，累积形式确保了流形上插值所需的局部连续性。我们利用三次b样条来确保\f$C^2\f$连续性，以便能够计算轨迹上任意点的加速度。
 * 通用方程如下
 *
 * \f{align*}{
 *  {}^{w}_{s}\mathbf{T}(u(t))
 *  &= {}^{w}_{i-1}\mathbf{T}~\mathbf{A}_0~\mathbf{A}_1~\mathbf{A}_2 \\
 * \empty
 *  {}^{w}_{s}\dot{\mathbf{T}}(u(t)) &=
 *  {}^{w}_{i-1}\mathbf{T}
 *  \Big(
 *  \dot{\mathbf{A}}_0~\mathbf{A}_1~\mathbf{A}_2 +
 *  \mathbf{A}_0~\dot{\mathbf{A}}_1~\mathbf{A}_2 +
 *  \mathbf{A}_0~\mathbf{A}_1~\dot{\mathbf{A}}_2
 *  \Big) \\
 * \empty
 *  {}^{w}_{s}\ddot{\mathbf{T}}(u(t)) &=
 *  {}^{w}_{i-1}\mathbf{T}
 *  \Big(
 *  \ddot{\mathbf{A}}_0~\mathbf{A}_1~\mathbf{A}_2 +
 *  \mathbf{A}_0~\ddot{\mathbf{A}}_1~\mathbf{A}_2 +
 *  \mathbf{A}_0~\mathbf{A}_1~\ddot{\mathbf{A}}_2 \nonumber\\
 *  &\hspace{4cm}
 *  + 2\dot{\mathbf{A}}_0\dot{\mathbf{A}}_1\mathbf{A}_2 +
 *  2\mathbf{A}_0\dot{\mathbf{A}}_1\dot{\mathbf{A}}_2 +
 *  2\dot{\mathbf{A}}_0\mathbf{A}_1\dot{\mathbf{A}}_2
 *  \Big)  \\[1em]
 * \empty
 *  {}^{i-1}_{i}\mathbf{\Omega} &= \mathrm{log}\big( {}^{w}_{i-1}\mathbf{T}^{-1}~{}^{w}_{i}\mathbf{T} \big) \\
 *  \mathbf{A}_j &= \mathrm{exp}\Big({B}_j(u(t))~{}^{i-1+j}_{i+j}\mathbf{\Omega} \Big) \\
 *  \dot{\mathbf{A}}_j &= \dot{B}_j(u(t)) ~{}^{i-1+j}_{i+j}\mathbf{\Omega}^\wedge ~\mathbf{A}_j \\
 *  \ddot{\mathbf{A}}_j &=
 *  \dot{B}_j(u(t)) ~{}^{i-1+j}_{i+j}\mathbf{\Omega}^\wedge ~\dot{\mathbf{A}}_j +
 *  \ddot{B}_j(u(t)) ~{}^{i-1+j}_{i+j}\mathbf{\Omega}^\wedge ~\mathbf{A}_j  \\[1em]
 * \empty
 *  {B}_0(u(t)) &= \frac{1}{3!}~(5+3u-3u^2+u^3) \\
 *  {B}_1(u(t)) &= \frac{1}{3!}~(1+3u+3u^2-2u^3) \\
 *  {B}_2(u(t)) &= \frac{1}{3!}~(u^3) \\[1em]
 * \empty
 *  \dot{{B}}_0(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t}~(3-6u+3u^2) \\
 *  \dot{{B}}_1(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t}~(3+6u-6u^2) \\
 *  \dot{{B}}_2(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t}~(3u^2) \\[1em]
 * \empty
 *  \ddot{{B}}_0(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t^2}~(-6+6u) \\
 *  \ddot{{B}}_1(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t^2}~(6-12u) \\
 *  \ddot{{B}}_2(u(t)) &= \frac{1}{3!}~\frac{1}{\Delta t^2}~(6u)
 * \f}
 *
 * 其中\f$u(t_s)=(t_s-t_i)/\Delta t=(t_s-t_i)/(t_{i+1}-t_i)\f$用于所有*u*值。
 * 注意，需要确保在上述所有方程中使用SE(3)矩阵指数、对数和hat运算。
 * 索引对应于比我们要获取的当前时间更旧的两个位姿和更新的两个位姿（即i-1和i小于s，而i+1和i+2都大于时间s）。
 * 一些额外的推导可在[these notes](http://udel.edu/~pgeneva/downloads/notes/2018_notes_mueffler2017arxiv.pdf)中找到。
 */
class BsplineSE3 {

public:
  /**
   * @brief 默认构造函数
   */
  BsplineSE3() {}

  /**
   * @brief 将输入一系列位姿，然后将其转换为控制点
   *
   * 我们的控制点需要在轨迹上均匀分布，因此给定轨迹后，我们将
   * 根据指定的位姿点之间的平均间距进行均匀采样。
   *
   * @param traj_points 我们将转换为控制点的轨迹位姿 (timestamp(s), q_GtoI, p_IinG)
   */
  void feed_trajectory(std::vector<Eigen::VectorXd> traj_points);

  /**
   * @brief 获取给定时间戳的朝向和位置
   * @param timestamp 要获取位姿的期望时间
   * @param R_GtoI 全局坐标系中的SO(3)朝向
   * @param p_IinG 全局坐标系中的位姿位置
   * @return 如果找不到则返回false
   */
  bool get_pose(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG);

  /**
   * @brief 获取给定时间戳的角速度和线速度
   * @param timestamp 要获取位姿的期望时间
   * @param R_GtoI 全局坐标系中的SO(3)朝向
   * @param p_IinG 全局坐标系中的位姿位置
   * @param w_IinI 惯性坐标系中的角速度
   * @param v_IinG 全局坐标系中的线速度
   * @return 如果找不到则返回false
   */
  bool get_velocity(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG, Eigen::Vector3d &w_IinI, Eigen::Vector3d &v_IinG);

  /**
   * @brief 获取给定时间戳的角加速度和线加速度
   * @param timestamp 要获取位姿的期望时间
   * @param R_GtoI 全局坐标系中的SO(3)朝向
   * @param p_IinG 全局坐标系中的位姿位置
   * @param w_IinI 惯性坐标系中的角速度
   * @param v_IinG 全局坐标系中的线速度
   * @param alpha_IinI 惯性坐标系中的角加速度
   * @param a_IinG 全局坐标系中的线加速度
   * @return 如果找不到则返回false
   */
  bool get_acceleration(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG, Eigen::Vector3d &w_IinI,
                        Eigen::Vector3d &v_IinG, Eigen::Vector3d &alpha_IinI, Eigen::Vector3d &a_IinG);

  /// 返回应该开始仿真的仿真开始时间
  double get_start_time() { return timestamp_start; }

protected:
  /// 控制点的均匀采样时间
  double dt;

  /// 系统的开始时间
  double timestamp_start;

  /// 对齐的eigen4d矩阵的类型定义: https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
  typedef std::map<double, Eigen::Matrix4d, std::less<double>, Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix4d>>>
      AlignedEigenMat4d;

  /// 我们的SE3控制位姿 (R_ItoG, p_IinG)
  AlignedEigenMat4d control_points;

  /**
   * @brief 将查找给定时间戳的两个边界位姿
   *
   * 这将遍历传入的位姿映射并找到两个边界位姿。
   * 如果没有边界位姿，则返回false。
   *
   * @param timestamp 我们要获取两个边界位姿的期望时间戳
   * @param poses 位姿和时间戳的映射
   * @param t0 第一个位姿的时间戳
   * @param pose0 第一个位姿的SE(3)位姿
   * @param t1 第二个位姿的时间戳
   * @param pose1 第二个位姿的SE(3)位姿
   * @return 如果无法找到边界位姿则返回false
   */
  static bool find_bounding_poses(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0, double &t1,
                                  Eigen::Matrix4d &pose1);

  /**
   * @brief 将为当前时间戳查找两个更旧的位姿和两个更新的位姿
   *
   * @param timestamp 我们要获取四个边界位姿的期望时间戳
   * @param poses 位姿和时间戳的映射
   * @param t0 第一个位姿的时间戳
   * @param pose0 第一个位姿的SE(3)位姿
   * @param t1 第二个位姿的时间戳
   * @param pose1 第二个位姿的SE(3)位姿
   * @param t2 第三个位姿的时间戳
   * @param pose2 第三个位姿的SE(3)位姿
   * @param t3 第四个位姿的时间戳
   * @param pose3 第四个位姿的SE(3)位姿
   * @return 如果无法找到边界位姿则返回false
   */
  static bool find_bounding_control_points(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0,
                                           double &t1, Eigen::Matrix4d &pose1, double &t2, Eigen::Matrix4d &pose2, double &t3,
                                           Eigen::Matrix4d &pose3);
};

} // namespace ov_core

#endif // OV_CORE_BSPLINESE3_H
