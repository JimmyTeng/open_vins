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

#ifndef OV_INIT_HELPER
#define OV_INIT_HELPER

#include "cpi/CpiV1.h"
#include "types/IMU.h"

namespace ov_init {

/**
 * @brief Has a bunch of helper functions for dynamic initialization (should all be static)
 * @brief 包含用于动态初始化的辅助函数集合（所有函数都应该是静态的）
 */
class InitializerHelper {
public:
  /**
   * @brief Nice helper function that will linearly interpolate between two imu messages.
   * @brief 在两个IMU消息之间进行线性插值的辅助函数
   *
   * This should be used instead of just "cutting" imu messages that bound the camera times
   * Give better time offset if we use this function, could try other orders/splines if the imu is slow.
   * 应该使用此函数而不是简单地"截断"包围相机时间的IMU消息。
   * 使用此函数可以获得更好的时间偏移，如果IMU频率较慢，可以尝试其他阶数/样条插值。
   *
   * @param imu_1 imu at begining of interpolation interval
   * @param imu_1 插值区间开始时刻的IMU数据
   * @param imu_2 imu at end of interpolation interval
   * @param imu_2 插值区间结束时刻的IMU数据
   * @param timestamp Timestamp being interpolated to
   * @param timestamp 要插值到的时间戳
   */
  static ov_core::ImuData interpolate_data(const ov_core::ImuData &imu_1, const ov_core::ImuData &imu_2, double timestamp) {
    // 计算插值系数 lambda，表示目标时间戳在区间中的相对位置
    double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
    ov_core::ImuData data;
    data.timestamp = timestamp;
    // 线性插值加速度计测量值
    data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
    // 线性插值陀螺仪测量值
    data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
    return data;
  }

  /**
   * @brief Helper function that given current imu data, will select imu readings between the two times.
   * @brief 从给定的IMU数据中选择两个时间点之间的IMU读数的辅助函数
   *
   * This will create measurements that we will integrate with, and an extra measurement at the end.
   * We use the @ref interpolate_data() function to "cut" the imu readings at the beginning and end of the integration.
   * The timestamps passed should already take into account the time offset values.
   * 这将创建用于积分的测量值，并在末尾添加一个额外的测量值。
   * 我们使用 @ref interpolate_data() 函数在积分开始和结束时"截断"IMU读数。
   * 传入的时间戳应该已经考虑了时间偏移值。
   *
   * @param imu_data_tmp IMU data we will select measurements from
   * @param imu_data_tmp 要从中选择测量值的IMU数据
   * @param time0 Start timestamp
   * @param time0 开始时间戳
   * @param time1 End timestamp
   * @param time1 结束时间戳
   * @return Vector of measurements (if we could compute them)
   * @return 测量值向量（如果能够计算的话）
   */
  static std::vector<ov_core::ImuData> select_imu_readings(const std::vector<ov_core::ImuData> &imu_data_tmp, double time0, double time1) {
    // 用于存储选中的IMU读数向量
    std::vector<ov_core::ImuData> prop_data;

    // 确保我们至少有一些测量值！
    if (imu_data_tmp.empty()) {
      return prop_data;
    }

    // 遍历并找到所有用于传播所需的测量值
    // 注意：我们根据给定的状态时间和更新时间戳来分割测量值
    for (size_t i = 0; i < imu_data_tmp.size() - 1; i++) {

      // 积分周期开始时刻
      // 如果时间0落在两个IMU测量值之间，则进行插值
      if (imu_data_tmp.at(i + 1).timestamp > time0 && imu_data_tmp.at(i).timestamp < time0) {
        ov_core::ImuData data = interpolate_data(imu_data_tmp.at(i), imu_data_tmp.at(i + 1), time0);
        prop_data.push_back(data);
        continue;
      }

      // 积分周期中间部分
      // 如果IMU测量值完全在积分区间内，则直接使用
      if (imu_data_tmp.at(i).timestamp >= time0 && imu_data_tmp.at(i + 1).timestamp <= time1) {
        prop_data.push_back(imu_data_tmp.at(i));
        continue;
      }

      // 积分周期结束时刻
      // 如果时间1落在两个IMU测量值之间，则进行插值
      if (imu_data_tmp.at(i + 1).timestamp > time1) {
        // 如果第一个测量值就超过了结束时间，则退出
        if (imu_data_tmp.at(i).timestamp > time1 && i == 0) {
          break;
        } else if (imu_data_tmp.at(i).timestamp > time1) {
          // 当前测量值超过结束时间，使用前一个测量值进行插值
          ov_core::ImuData data = interpolate_data(imu_data_tmp.at(i - 1), imu_data_tmp.at(i), time1);
          prop_data.push_back(data);
        } else {
          // 当前测量值在结束时间之前，直接添加
          prop_data.push_back(imu_data_tmp.at(i));
        }
        // 确保最后一个测量值的时间戳等于结束时间
        if (prop_data.at(prop_data.size() - 1).timestamp != time1) {
          ov_core::ImuData data = interpolate_data(imu_data_tmp.at(i), imu_data_tmp.at(i + 1), time1);
          prop_data.push_back(data);
        }
        break;
      }
    }

    // 检查我们是否至少有一个测量值用于传播
    if (prop_data.empty()) {
      return prop_data;
    }

    // 遍历并确保我们没有零时间间隔的测量值
    // 这会导致噪声协方差变为无穷大
    for (size_t i = 0; i < prop_data.size() - 1; i++) {
      if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) < 1e-12) {
        prop_data.erase(prop_data.begin() + i);
        i--;
      }
    }

    // 成功返回 :D
    return prop_data;
  }

  /**
   * @brief Given a gravity vector, compute the rotation from the inertial reference frame to this vector.
   * @brief 给定重力向量，计算从惯性参考系到该向量的旋转矩阵
   *
   * The key assumption here is that our gravity is along the vertical direction in the inertial frame.
   * We can take this vector (z_in_G=0,0,1) and find two arbitrary tangent directions to it.
   * https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
   * 关键假设是：在惯性参考系中，重力沿垂直方向。
   * 我们可以取该向量（z_in_G=0,0,1）并找到它的两个任意切向方向。
   * 使用 Gram-Schmidt 正交化过程来构建旋转矩阵。
   *
   * @param gravity_inI Gravity in our sensor frame
   * @param gravity_inI 传感器坐标系中的重力向量
   * @param R_GtoI Rotation from the arbitrary inertial reference frame to this gravity vector
   * @param R_GtoI 从任意惯性参考系到该重力向量的旋转矩阵（输出参数）
   */
  static void gram_schmidt(const Eigen::Vector3d &gravity_inI, Eigen::Matrix3d &R_GtoI) {

    // 找到与重力正交的向量，这是我们的局部z轴
    // 我们需要确保在每一步之后都进行归一化，以获得单位向量
    Eigen::Vector3d z_axis = gravity_inI / gravity_inI.norm();
    Eigen::Vector3d x_axis, y_axis;
    // 定义两个标准基向量用于构建正交基
    Eigen::Vector3d e_1(1.0, 0.0, 0.0);
    Eigen::Vector3d e_2(0.0, 1.0, 0.0);
    // 计算内积以选择更合适的基向量（避免与z轴平行）
    double inner1 = e_1.dot(z_axis) / z_axis.norm();
    double inner2 = e_2.dot(z_axis) / z_axis.norm();
    // 选择与z轴内积较小的基向量，以确保叉积结果不为零
    if (fabs(inner1) < fabs(inner2)) {
      // 使用e_1构建x轴
      x_axis = z_axis.cross(e_1);
      x_axis = x_axis / x_axis.norm();
      // 使用z轴和x轴构建y轴
      y_axis = z_axis.cross(x_axis);
      y_axis = y_axis / y_axis.norm();
    } else {
      // 使用e_2构建x轴
      x_axis = z_axis.cross(e_2);
      x_axis = x_axis / x_axis.norm();
      // 使用z轴和x轴构建y轴
      y_axis = z_axis.cross(x_axis);
      y_axis = y_axis / y_axis.norm();
    }

    // 原始方法（已注释）
    // https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
    // x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
    // x_axis = x_axis / x_axis.norm();
    // y_axis = ov_core::skew_x(z_axis) * x_axis;
    // y_axis = y_axis / y_axis.norm();

    // 从全局坐标系（重力仅沿z轴）到局部坐标系的旋转矩阵
    R_GtoI.block(0, 0, 3, 1) = x_axis;
    R_GtoI.block(0, 1, 3, 1) = y_axis;
    R_GtoI.block(0, 2, 3, 1) = z_axis;
  }

  /**
   * @brief Compute coefficients for the constrained optimization quadratic problem.
   * @brief 计算约束优化二次问题的系数
   *
   * https://gist.github.com/goldbattle/3791cbb11bbf4f5feb3f049dad72bfdd
   * 用于求解重力方向估计的约束优化问题，返回多项式的系数。
   *
   * @param D Gravity in our sensor frame
   * @param D 传感器坐标系中的重力相关矩阵（3x3）
   * @param d Rotation from the arbitrary inertial reference frame to this gravity vector
   * @param d 从任意惯性参考系到该重力向量的旋转相关向量（3x1）
   * @param gravity_mag Scalar size of gravity (normally is 9.81)
   * @param gravity_mag 重力的大小标量（通常为9.81 m/s²）
   * @return Coefficents from highest to the constant
   * @return 从最高次项到常数项的系数向量（7x1）
   */
  static Eigen::Matrix<double, 7, 1> compute_dongsi_coeff(Eigen::MatrixXd &D, const Eigen::MatrixXd &d, double gravity_mag) {

    // 检查矩阵维度（从MATLAB代码转换而来）
    assert(D.rows() == 3);
    assert(D.cols() == 3);
    assert(d.rows() == 3);
    // 提取矩阵D的元素
    double D1_1 = D(0, 0), D1_2 = D(0, 1), D1_3 = D(0, 2);
    double D2_1 = D(1, 0), D2_2 = D(1, 1), D2_3 = D(1, 2);
    double D3_1 = D(2, 0), D3_2 = D(2, 1), D3_3 = D(2, 2);
    // 提取向量d的元素
    double d1 = d(0, 0), d2 = d(1, 0), d3 = d(2, 0);
    double g = gravity_mag;

    // 计算平方项（用于替换x^2）
    double D1_1_sq = D1_1 * D1_1, D1_2_sq = D1_2 * D1_2, D1_3_sq = D1_3 * D1_3;
    double D2_1_sq = D2_1 * D2_1, D2_2_sq = D2_2 * D2_2, D2_3_sq = D2_3 * D2_3;
    double D3_1_sq = D3_1 * D3_1, D3_2_sq = D3_2 * D3_2, D3_3_sq = D3_3 * D3_3;
    double d1_sq = d1 * d1, d2_sq = d2 * d2, d3_sq = d3 * d3;
    double g_sq = g * g;

    // 计算多项式系数（从6次项到常数项）
    Eigen::Matrix<double, 7, 1> coeff = Eigen::Matrix<double, 7, 1>::Zero();
    coeff(6) =
        -(-D1_1_sq * D2_2_sq * D3_3_sq * g_sq + D1_1_sq * D2_2_sq * d3_sq + 2 * D1_1_sq * D2_2 * D2_3 * D3_2 * D3_3 * g_sq -
          D1_1_sq * D2_2 * D2_3 * d2 * d3 - D1_1_sq * D2_2 * D3_2 * d2 * d3 - D1_1_sq * D2_3_sq * D3_2_sq * g_sq +
          D1_1_sq * D2_3 * D3_2 * d2_sq + D1_1_sq * D2_3 * D3_2 * d3_sq - D1_1_sq * D2_3 * D3_3 * d2 * d3 -
          D1_1_sq * D3_2 * D3_3 * d2 * d3 + D1_1_sq * D3_3_sq * d2_sq + 2 * D1_1 * D1_2 * D2_1 * D2_2 * D3_3_sq * g_sq -
          2 * D1_1 * D1_2 * D2_1 * D2_2 * d3_sq - 2 * D1_1 * D1_2 * D2_1 * D2_3 * D3_2 * D3_3 * g_sq + D1_1 * D1_2 * D2_1 * D2_3 * d2 * d3 +
          D1_1 * D1_2 * D2_1 * D3_2 * d2 * d3 - 2 * D1_1 * D1_2 * D2_2 * D2_3 * D3_1 * D3_3 * g_sq + D1_1 * D1_2 * D2_2 * D2_3 * d1 * d3 +
          D1_1 * D1_2 * D2_2 * D3_1 * d2 * d3 + 2 * D1_1 * D1_2 * D2_3_sq * D3_1 * D3_2 * g_sq - D1_1 * D1_2 * D2_3 * D3_1 * d2_sq -
          D1_1 * D1_2 * D2_3 * D3_1 * d3_sq - D1_1 * D1_2 * D2_3 * D3_2 * d1 * d2 + D1_1 * D1_2 * D2_3 * D3_3 * d1 * d3 +
          D1_1 * D1_2 * D3_1 * D3_3 * d2 * d3 - D1_1 * D1_2 * D3_3_sq * d1 * d2 - 2 * D1_1 * D1_3 * D2_1 * D2_2 * D3_2 * D3_3 * g_sq +
          D1_1 * D1_3 * D2_1 * D2_2 * d2 * d3 + 2 * D1_1 * D1_3 * D2_1 * D2_3 * D3_2_sq * g_sq - D1_1 * D1_3 * D2_1 * D3_2 * d2_sq -
          D1_1 * D1_3 * D2_1 * D3_2 * d3_sq + D1_1 * D1_3 * D2_1 * D3_3 * d2 * d3 + 2 * D1_1 * D1_3 * D2_2_sq * D3_1 * D3_3 * g_sq -
          D1_1 * D1_3 * D2_2_sq * d1 * d3 - 2 * D1_1 * D1_3 * D2_2 * D2_3 * D3_1 * D3_2 * g_sq + D1_1 * D1_3 * D2_2 * D3_2 * d1 * d2 +
          D1_1 * D1_3 * D2_3 * D3_1 * d2 * d3 - D1_1 * D1_3 * D2_3 * D3_2 * d1 * d3 + D1_1 * D1_3 * D3_1 * D3_2 * d2 * d3 -
          2 * D1_1 * D1_3 * D3_1 * D3_3 * d2_sq + D1_1 * D1_3 * D3_2 * D3_3 * d1 * d2 + D1_1 * D2_1 * D2_2 * D3_2 * d1 * d3 -
          D1_1 * D2_1 * D2_3 * D3_2 * d1 * d2 + D1_1 * D2_1 * D3_2 * D3_3 * d1 * d3 - D1_1 * D2_1 * D3_3_sq * d1 * d2 -
          D1_1 * D2_2_sq * D3_1 * d1 * d3 + D1_1 * D2_2 * D2_3 * D3_1 * d1 * d2 - D1_1 * D2_3 * D3_1 * D3_2 * d1 * d3 +
          D1_1 * D2_3 * D3_1 * D3_3 * d1 * d2 - D1_2_sq * D2_1_sq * D3_3_sq * g_sq + D1_2_sq * D2_1_sq * d3_sq +
          2 * D1_2_sq * D2_1 * D2_3 * D3_1 * D3_3 * g_sq - D1_2_sq * D2_1 * D2_3 * d1 * d3 - D1_2_sq * D2_1 * D3_1 * d2 * d3 -
          D1_2_sq * D2_3_sq * D3_1_sq * g_sq + D1_2_sq * D2_3 * D3_1 * d1 * d2 + 2 * D1_2 * D1_3 * D2_1_sq * D3_2 * D3_3 * g_sq -
          D1_2 * D1_3 * D2_1_sq * d2 * d3 - 2 * D1_2 * D1_3 * D2_1 * D2_2 * D3_1 * D3_3 * g_sq + D1_2 * D1_3 * D2_1 * D2_2 * d1 * d3 -
          2 * D1_2 * D1_3 * D2_1 * D2_3 * D3_1 * D3_2 * g_sq + D1_2 * D1_3 * D2_1 * D3_1 * d2_sq + D1_2 * D1_3 * D2_1 * D3_1 * d3_sq -
          D1_2 * D1_3 * D2_1 * D3_3 * d1 * d3 + 2 * D1_2 * D1_3 * D2_2 * D2_3 * D3_1_sq * g_sq - D1_2 * D1_3 * D2_2 * D3_1 * d1 * d2 -
          D1_2 * D1_3 * D3_1_sq * d2 * d3 + D1_2 * D1_3 * D3_1 * D3_3 * d1 * d2 - D1_2 * D2_1_sq * D3_2 * d1 * d3 +
          D1_2 * D2_1 * D2_2 * D3_1 * d1 * d3 + D1_2 * D2_1 * D2_3 * D3_2 * d1_sq + D1_2 * D2_1 * D2_3 * D3_2 * d3_sq -
          D1_2 * D2_1 * D2_3 * D3_3 * d2 * d3 - D1_2 * D2_1 * D3_1 * D3_3 * d1 * d3 - D1_2 * D2_1 * D3_2 * D3_3 * d2 * d3 +
          D1_2 * D2_1 * D3_3_sq * d1_sq + D1_2 * D2_1 * D3_3_sq * d2_sq - D1_2 * D2_2 * D2_3 * D3_1 * d1_sq -
          D1_2 * D2_2 * D2_3 * D3_1 * d3_sq + D1_2 * D2_2 * D2_3 * D3_3 * d1 * d3 + D1_2 * D2_2 * D3_1 * D3_3 * d2 * d3 -
          D1_2 * D2_2 * D3_3_sq * d1 * d2 + D1_2 * D2_3_sq * D3_1 * d2 * d3 - D1_2 * D2_3_sq * D3_2 * d1 * d3 +
          D1_2 * D2_3 * D3_1_sq * d1 * d3 - D1_2 * D2_3 * D3_1 * D3_3 * d1_sq - D1_2 * D2_3 * D3_1 * D3_3 * d2_sq +
          D1_2 * D2_3 * D3_2 * D3_3 * d1 * d2 - D1_3_sq * D2_1_sq * D3_2_sq * g_sq + 2 * D1_3_sq * D2_1 * D2_2 * D3_1 * D3_2 * g_sq -
          D1_3_sq * D2_1 * D3_1 * d2 * d3 + D1_3_sq * D2_1 * D3_2 * d1 * d3 - D1_3_sq * D2_2_sq * D3_1_sq * g_sq +
          D1_3_sq * D3_1_sq * d2_sq - D1_3_sq * D3_1 * D3_2 * d1 * d2 + D1_3 * D2_1_sq * D3_2 * d1 * d2 -
          D1_3 * D2_1 * D2_2 * D3_1 * d1 * d2 - D1_3 * D2_1 * D2_2 * D3_2 * d1_sq - D1_3 * D2_1 * D2_2 * D3_2 * d3_sq +
          D1_3 * D2_1 * D2_2 * D3_3 * d2 * d3 + D1_3 * D2_1 * D3_1 * D3_3 * d1 * d2 + D1_3 * D2_1 * D3_2_sq * d2 * d3 -
          D1_3 * D2_1 * D3_2 * D3_3 * d1_sq - D1_3 * D2_1 * D3_2 * D3_3 * d2_sq + D1_3 * D2_2_sq * D3_1 * d1_sq +
          D1_3 * D2_2_sq * D3_1 * d3_sq - D1_3 * D2_2_sq * D3_3 * d1 * d3 - D1_3 * D2_2 * D2_3 * D3_1 * d2 * d3 +
          D1_3 * D2_2 * D2_3 * D3_2 * d1 * d3 - D1_3 * D2_2 * D3_1 * D3_2 * d2 * d3 + D1_3 * D2_2 * D3_2 * D3_3 * d1 * d2 -
          D1_3 * D2_3 * D3_1_sq * d1 * d2 + D1_3 * D2_3 * D3_1 * D3_2 * d1_sq + D1_3 * D2_3 * D3_1 * D3_2 * d2_sq -
          D1_3 * D2_3 * D3_2_sq * d1 * d2 + D2_1 * D2_2 * D3_2 * D3_3 * d1 * d3 - D2_1 * D2_2 * D3_3_sq * d1 * d2 -
          D2_1 * D2_3 * D3_2_sq * d1 * d3 + D2_1 * D2_3 * D3_2 * D3_3 * d1 * d2 - D2_2_sq * D3_1 * D3_3 * d1 * d3 +
          D2_2_sq * D3_3_sq * d1_sq + D2_2 * D2_3 * D3_1 * D3_2 * d1 * d3 + D2_2 * D2_3 * D3_1 * D3_3 * d1 * d2 -
          2 * D2_2 * D2_3 * D3_2 * D3_3 * d1_sq - D2_3_sq * D3_1 * D3_2 * d1 * d2 + D2_3_sq * D3_2_sq * d1_sq) /
        g_sq;
    coeff(5) =
        (-(2 * D1_1_sq * D2_2_sq * D3_3 * g_sq - 2 * D1_1_sq * D2_2 * D2_3 * D3_2 * g_sq + 2 * D1_1_sq * D2_2 * D3_3_sq * g_sq -
           2 * D1_1_sq * D2_2 * d3_sq - 2 * D1_1_sq * D2_3 * D3_2 * D3_3 * g_sq + 2 * D1_1_sq * D2_3 * d2 * d3 +
           2 * D1_1_sq * D3_2 * d2 * d3 - 2 * D1_1_sq * D3_3 * d2_sq - 4 * D1_1 * D1_2 * D2_1 * D2_2 * D3_3 * g_sq +
           2 * D1_1 * D1_2 * D2_1 * D2_3 * D3_2 * g_sq - 2 * D1_1 * D1_2 * D2_1 * D3_3_sq * g_sq + 2 * D1_1 * D1_2 * D2_1 * d3_sq +
           2 * D1_1 * D1_2 * D2_2 * D2_3 * D3_1 * g_sq + 2 * D1_1 * D1_2 * D2_3 * D3_1 * D3_3 * g_sq - 2 * D1_1 * D1_2 * D2_3 * d1 * d3 -
           2 * D1_1 * D1_2 * D3_1 * d2 * d3 + 2 * D1_1 * D1_2 * D3_3 * d1 * d2 + 2 * D1_1 * D1_3 * D2_1 * D2_2 * D3_2 * g_sq +
           2 * D1_1 * D1_3 * D2_1 * D3_2 * D3_3 * g_sq - 2 * D1_1 * D1_3 * D2_1 * d2 * d3 - 2 * D1_1 * D1_3 * D2_2_sq * D3_1 * g_sq -
           4 * D1_1 * D1_3 * D2_2 * D3_1 * D3_3 * g_sq + 2 * D1_1 * D1_3 * D2_2 * d1 * d3 + 2 * D1_1 * D1_3 * D2_3 * D3_1 * D3_2 * g_sq +
           2 * D1_1 * D1_3 * D3_1 * d2_sq - 2 * D1_1 * D1_3 * D3_2 * d1 * d2 - 2 * D1_1 * D2_1 * D3_2 * d1 * d3 +
           2 * D1_1 * D2_1 * D3_3 * d1 * d2 + 2 * D1_1 * D2_2_sq * D3_3_sq * g_sq - 2 * D1_1 * D2_2_sq * d3_sq -
           4 * D1_1 * D2_2 * D2_3 * D3_2 * D3_3 * g_sq + 2 * D1_1 * D2_2 * D2_3 * d2 * d3 + 2 * D1_1 * D2_2 * D3_1 * d1 * d3 +
           2 * D1_1 * D2_2 * D3_2 * d2 * d3 + 2 * D1_1 * D2_3_sq * D3_2_sq * g_sq - 2 * D1_1 * D2_3 * D3_1 * d1 * d2 -
           2 * D1_1 * D2_3 * D3_2 * d2_sq - 2 * D1_1 * D2_3 * D3_2 * d3_sq + 2 * D1_1 * D2_3 * D3_3 * d2 * d3 +
           2 * D1_1 * D3_2 * D3_3 * d2 * d3 - 2 * D1_1 * D3_3_sq * d2_sq + 2 * D1_2_sq * D2_1_sq * D3_3 * g_sq -
           2 * D1_2_sq * D2_1 * D2_3 * D3_1 * g_sq - 2 * D1_2 * D1_3 * D2_1_sq * D3_2 * g_sq + 2 * D1_2 * D1_3 * D2_1 * D2_2 * D3_1 * g_sq +
           2 * D1_2 * D1_3 * D2_1 * D3_1 * D3_3 * g_sq - 2 * D1_2 * D1_3 * D2_3 * D3_1_sq * g_sq - 2 * D1_2 * D2_1 * D2_2 * D3_3_sq * g_sq +
           2 * D1_2 * D2_1 * D2_2 * d3_sq + 2 * D1_2 * D2_1 * D2_3 * D3_2 * D3_3 * g_sq - 2 * D1_2 * D2_1 * D3_3 * d1_sq -
           2 * D1_2 * D2_1 * D3_3 * d2_sq + 2 * D1_2 * D2_2 * D2_3 * D3_1 * D3_3 * g_sq - 2 * D1_2 * D2_2 * D2_3 * d1 * d3 -
           2 * D1_2 * D2_2 * D3_1 * d2 * d3 + 2 * D1_2 * D2_2 * D3_3 * d1 * d2 - 2 * D1_2 * D2_3_sq * D3_1 * D3_2 * g_sq +
           2 * D1_2 * D2_3 * D3_1 * d1_sq + 2 * D1_2 * D2_3 * D3_1 * d2_sq + 2 * D1_2 * D2_3 * D3_1 * d3_sq -
           2 * D1_2 * D2_3 * D3_3 * d1 * d3 - 2 * D1_2 * D3_1 * D3_3 * d2 * d3 + 2 * D1_2 * D3_3_sq * d1 * d2 -
           2 * D1_3_sq * D2_1 * D3_1 * D3_2 * g_sq + 2 * D1_3_sq * D2_2 * D3_1_sq * g_sq + 2 * D1_3 * D2_1 * D2_2 * D3_2 * D3_3 * g_sq -
           2 * D1_3 * D2_1 * D2_2 * d2 * d3 - 2 * D1_3 * D2_1 * D2_3 * D3_2_sq * g_sq + 2 * D1_3 * D2_1 * D3_2 * d1_sq +
           2 * D1_3 * D2_1 * D3_2 * d2_sq + 2 * D1_3 * D2_1 * D3_2 * d3_sq - 2 * D1_3 * D2_1 * D3_3 * d2 * d3 -
           2 * D1_3 * D2_2_sq * D3_1 * D3_3 * g_sq + 2 * D1_3 * D2_2_sq * d1 * d3 + 2 * D1_3 * D2_2 * D2_3 * D3_1 * D3_2 * g_sq -
           2 * D1_3 * D2_2 * D3_1 * d1_sq - 2 * D1_3 * D2_2 * D3_1 * d3_sq - 2 * D1_3 * D2_2 * D3_2 * d1 * d2 +
           2 * D1_3 * D2_2 * D3_3 * d1 * d3 + 2 * D1_3 * D3_1 * D3_3 * d2_sq - 2 * D1_3 * D3_2 * D3_3 * d1 * d2 -
           2 * D2_1 * D2_2 * D3_2 * d1 * d3 + 2 * D2_1 * D2_2 * D3_3 * d1 * d2 - 2 * D2_1 * D3_2 * D3_3 * d1 * d3 +
           2 * D2_1 * D3_3_sq * d1 * d2 + 2 * D2_2_sq * D3_1 * d1 * d3 - 2 * D2_2_sq * D3_3 * d1_sq - 2 * D2_2 * D2_3 * D3_1 * d1 * d2 +
           2 * D2_2 * D2_3 * D3_2 * d1_sq + 2 * D2_2 * D3_1 * D3_3 * d1 * d3 - 2 * D2_2 * D3_3_sq * d1_sq -
           2 * D2_3 * D3_1 * D3_3 * d1 * d2 + 2 * D2_3 * D3_2 * D3_3 * d1_sq) /
         g_sq);
    coeff(4) =
        ((D1_1_sq * D2_2_sq * g_sq + 4 * D1_1_sq * D2_2 * D3_3 * g_sq - 2 * D1_1_sq * D2_3 * D3_2 * g_sq + D1_1_sq * D3_3_sq * g_sq -
          D1_1_sq * d2_sq - D1_1_sq * d3_sq - 2 * D1_1 * D1_2 * D2_1 * D2_2 * g_sq - 4 * D1_1 * D1_2 * D2_1 * D3_3 * g_sq +
          2 * D1_1 * D1_2 * D2_3 * D3_1 * g_sq + D1_1 * D1_2 * d1 * d2 + 2 * D1_1 * D1_3 * D2_1 * D3_2 * g_sq -
          4 * D1_1 * D1_3 * D2_2 * D3_1 * g_sq - 2 * D1_1 * D1_3 * D3_1 * D3_3 * g_sq + D1_1 * D1_3 * d1 * d3 + D1_1 * D2_1 * d1 * d2 +
          4 * D1_1 * D2_2_sq * D3_3 * g_sq - 4 * D1_1 * D2_2 * D2_3 * D3_2 * g_sq + 4 * D1_1 * D2_2 * D3_3_sq * g_sq -
          4 * D1_1 * D2_2 * d3_sq - 4 * D1_1 * D2_3 * D3_2 * D3_3 * g_sq + 4 * D1_1 * D2_3 * d2 * d3 + D1_1 * D3_1 * d1 * d3 +
          4 * D1_1 * D3_2 * d2 * d3 - 4 * D1_1 * D3_3 * d2_sq + D1_2_sq * D2_1_sq * g_sq + 2 * D1_2 * D1_3 * D2_1 * D3_1 * g_sq -
          4 * D1_2 * D2_1 * D2_2 * D3_3 * g_sq + 2 * D1_2 * D2_1 * D2_3 * D3_2 * g_sq - 2 * D1_2 * D2_1 * D3_3_sq * g_sq -
          D1_2 * D2_1 * d1_sq - D1_2 * D2_1 * d2_sq + 2 * D1_2 * D2_1 * d3_sq + 2 * D1_2 * D2_2 * D2_3 * D3_1 * g_sq +
          D1_2 * D2_2 * d1 * d2 + 2 * D1_2 * D2_3 * D3_1 * D3_3 * g_sq - 3 * D1_2 * D2_3 * d1 * d3 - 3 * D1_2 * D3_1 * d2 * d3 +
          4 * D1_2 * D3_3 * d1 * d2 + D1_3_sq * D3_1_sq * g_sq + 2 * D1_3 * D2_1 * D2_2 * D3_2 * g_sq +
          2 * D1_3 * D2_1 * D3_2 * D3_3 * g_sq - 3 * D1_3 * D2_1 * d2 * d3 - 2 * D1_3 * D2_2_sq * D3_1 * g_sq -
          4 * D1_3 * D2_2 * D3_1 * D3_3 * g_sq + 4 * D1_3 * D2_2 * d1 * d3 + 2 * D1_3 * D2_3 * D3_1 * D3_2 * g_sq - D1_3 * D3_1 * d1_sq +
          2 * D1_3 * D3_1 * d2_sq - D1_3 * D3_1 * d3_sq - 3 * D1_3 * D3_2 * d1 * d2 + D1_3 * D3_3 * d1 * d3 + D2_1 * D2_2 * d1 * d2 -
          3 * D2_1 * D3_2 * d1 * d3 + 4 * D2_1 * D3_3 * d1 * d2 + D2_2_sq * D3_3_sq * g_sq - D2_2_sq * d1_sq - D2_2_sq * d3_sq -
          2 * D2_2 * D2_3 * D3_2 * D3_3 * g_sq + D2_2 * D2_3 * d2 * d3 + 4 * D2_2 * D3_1 * d1 * d3 + D2_2 * D3_2 * d2 * d3 -
          4 * D2_2 * D3_3 * d1_sq + D2_3_sq * D3_2_sq * g_sq - 3 * D2_3 * D3_1 * d1 * d2 + 2 * D2_3 * D3_2 * d1_sq - D2_3 * D3_2 * d2_sq -
          D2_3 * D3_2 * d3_sq + D2_3 * D3_3 * d2 * d3 + D3_1 * D3_3 * d1 * d3 + D3_2 * D3_3 * d2 * d3 - D3_3_sq * d1_sq - D3_3_sq * d2_sq) /
         g_sq);
    coeff(3) =
        ((2 * D1_1 * d2_sq + 2 * D1_1 * d3_sq + 2 * D2_2 * d1_sq + 2 * D2_2 * d3_sq + 2 * D3_3 * d1_sq + 2 * D3_3 * d2_sq -
          2 * D1_1 * D2_2_sq * g_sq - 2 * D1_1_sq * D2_2 * g_sq - 2 * D1_1 * D3_3_sq * g_sq - 2 * D1_1_sq * D3_3 * g_sq -
          2 * D2_2 * D3_3_sq * g_sq - 2 * D2_2_sq * D3_3 * g_sq - 2 * D1_2 * d1 * d2 - 2 * D1_3 * d1 * d3 - 2 * D2_1 * d1 * d2 -
          2 * D2_3 * d2 * d3 - 2 * D3_1 * d1 * d3 - 2 * D3_2 * d2 * d3 + 2 * D1_1 * D1_2 * D2_1 * g_sq + 2 * D1_1 * D1_3 * D3_1 * g_sq +
          2 * D1_2 * D2_1 * D2_2 * g_sq - 8 * D1_1 * D2_2 * D3_3 * g_sq + 4 * D1_1 * D2_3 * D3_2 * g_sq + 4 * D1_2 * D2_1 * D3_3 * g_sq -
          2 * D1_2 * D2_3 * D3_1 * g_sq - 2 * D1_3 * D2_1 * D3_2 * g_sq + 4 * D1_3 * D2_2 * D3_1 * g_sq + 2 * D1_3 * D3_1 * D3_3 * g_sq +
          2 * D2_2 * D2_3 * D3_2 * g_sq + 2 * D2_3 * D3_2 * D3_3 * g_sq) /
         g_sq);
    coeff(2) =
        (-(d1_sq + d2_sq + d3_sq - D1_1_sq * g_sq - D2_2_sq * g_sq - D3_3_sq * g_sq - 4 * D1_1 * D2_2 * g_sq + 2 * D1_2 * D2_1 * g_sq -
           4 * D1_1 * D3_3 * g_sq + 2 * D1_3 * D3_1 * g_sq - 4 * D2_2 * D3_3 * g_sq + 2 * D2_3 * D3_2 * g_sq) /
         g_sq);
    coeff(1) = (-(2 * D1_1 * g_sq + 2 * D2_2 * g_sq + 2 * D3_3 * g_sq) / g_sq);
    coeff(0) = 1;  // 最高次项（6次项）的系数为1

    // 最终返回系数向量
    return coeff;
  }
};
} // namespace ov_init

#endif /* OV_INIT_HELPER */
