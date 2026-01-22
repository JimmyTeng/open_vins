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

#include "BsplineSE3.h"

using namespace ov_core;

void BsplineSE3::feed_trajectory(std::vector<Eigen::VectorXd> traj_points) {

  // 查找平均频率以用作我们的均匀时间步长
  double sumdt = 0;
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    sumdt += traj_points.at(i + 1)(0) - traj_points.at(i)(0);
  }
  dt = sumdt / (traj_points.size() - 1);
  dt = (dt < 0.05) ? 0.05 : dt;
  PRINT_DEBUG("[B-SPLINE]: control point dt = %.3f (original dt of %.3f)\n", dt, sumdt / (traj_points.size() - 1));

  // 将所有轨迹点转换为SE(3)矩阵
  // 我们被给定 [timestamp, p_IinG, q_GtoI]
  AlignedEigenMat4d trajectory_points;
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    Eigen::Matrix4d T_IinG = Eigen::Matrix4d::Identity();
    T_IinG.block(0, 0, 3, 3) = quat_2_Rot(traj_points.at(i).block(4, 0, 4, 1)).transpose();
    T_IinG.block(0, 3, 3, 1) = traj_points.at(i).block(1, 0, 3, 1);
    trajectory_points.insert({traj_points.at(i)(0), T_IinG});
  }

  // 获取最旧的时间戳
  double timestamp_min = INFINITY;
  double timestamp_max = -INFINITY;
  for (const auto &pose : trajectory_points) {
    if (pose.first <= timestamp_min) {
      timestamp_min = pose.first;
    }
    if (pose.first >= timestamp_max) {
      timestamp_max = pose.first;
    }
  }
  PRINT_DEBUG("[B-SPLINE]: trajectory start time = %.6f\n", timestamp_min);
  PRINT_DEBUG("[B-SPLINE]: trajectory end time = %.6f\n", timestamp_max);

  // 然后创建样条控制点
  double timestamp_curr = timestamp_min;
  while (true) {

    // 获取当前时间的边界位姿
    double t0, t1;
    Eigen::Matrix4d pose0, pose1;
    bool success = find_bounding_poses(timestamp_curr, trajectory_points, t0, pose0, t1, pose1);
    // PRINT_DEBUG("[SIM]: time curr = %.6f | lambda = %.3f | dt = %.3f | dtmeas =
    // %.3f\n",timestamp_curr,(timestamp_curr-t0)/(t1-t0),dt,(t1-t0));

    // 如果我们没有找到边界位姿，那意味着我们已到达数据集的末尾
    // 因此跳出循环，因为我们已经创建了最大数量的控制点
    if (!success)
      break;

    // 线性插值并追加到我们的控制点
    double lambda = (timestamp_curr - t0) / (t1 - t0);
    Eigen::Matrix4d pose_interp = exp_se3(lambda * log_se3(pose1 * Inv_se3(pose0))) * pose0;
    control_points.insert({timestamp_curr, pose_interp});
    timestamp_curr += dt;
    // std::stringstream ss;
    // ss << pose_interp(0,3) << "," << pose_interp(1,3) << "," << pose_interp(2,3) << std::endl;
    // PRINT_DEBUG(ss.str().c_str());
  }

  // 系统的开始时间是两个dt，因为我们需要至少两个更旧的控制点
  timestamp_start = timestamp_min + 2 * dt;
  PRINT_DEBUG("[B-SPLINE]: start trajectory time of %.6f\n", timestamp_start);
}

bool BsplineSE3::get_pose(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG) {

  // 获取期望时间戳的边界位姿
  double t0, t1, t2, t3;
  Eigen::Matrix4d pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);
  // PRINT_DEBUG("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success =
  // %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

  // 如果无法获取边界位姿则返回失败
  if (!success) {
    R_GtoI.setIdentity();
    p_IinG.setZero();
    return false;
  }

  // 我们的De Boor-Cox矩阵标量
  double DT = (t2 - t1);
  double u = (timestamp - t1) / DT;
  double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double b2 = 1.0 / 6.0 * (u * u * u);

  // 计算插值位姿
  Eigen::Matrix4d A0 = exp_se3(b0 * log_se3(Inv_se3(pose0) * pose1));
  Eigen::Matrix4d A1 = exp_se3(b1 * log_se3(Inv_se3(pose1) * pose2));
  Eigen::Matrix4d A2 = exp_se3(b2 * log_se3(Inv_se3(pose2) * pose3));

  // 最终获取插值位姿
  Eigen::Matrix4d pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::get_velocity(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG, Eigen::Vector3d &w_IinI,
                              Eigen::Vector3d &v_IinG) {

  // 获取期望时间戳的边界位姿
  double t0, t1, t2, t3;
  Eigen::Matrix4d pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);
  // PRINT_DEBUG("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success =
  // %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

  // 如果无法获取边界位姿则返回失败
  if (!success) {
    w_IinI.setZero();
    v_IinG.setZero();
    return false;
  }

  // 我们的De Boor-Cox矩阵标量
  double DT = (t2 - t1);
  double u = (timestamp - t1) / DT;
  double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double b2 = 1.0 / 6.0 * (u * u * u);
  double b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
  double b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
  double b2dot = 1.0 / (6.0 * DT) * (3 * u * u);

  // 缓存一些我们经常使用的值
  Eigen::Matrix<double, 6, 1> omega_10 = log_se3(Inv_se3(pose0) * pose1);
  Eigen::Matrix<double, 6, 1> omega_21 = log_se3(Inv_se3(pose1) * pose2);
  Eigen::Matrix<double, 6, 1> omega_32 = log_se3(Inv_se3(pose2) * pose3);

  // 计算插值位姿
  Eigen::Matrix4d A0 = exp_se3(b0 * omega_10);
  Eigen::Matrix4d A1 = exp_se3(b1 * omega_21);
  Eigen::Matrix4d A2 = exp_se3(b2 * omega_32);
  Eigen::Matrix4d A0dot = b0dot * hat_se3(omega_10) * A0;
  Eigen::Matrix4d A1dot = b1dot * hat_se3(omega_21) * A1;
  Eigen::Matrix4d A2dot = b2dot * hat_se3(omega_32) * A2;

  // 获取插值位姿
  Eigen::Matrix4d pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);

  // 最终获取插值速度
  // 注意: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
  Eigen::Matrix4d vel_interp = pose0 * (A0dot * A1 * A2 + A0 * A1dot * A2 + A0 * A1 * A2dot);
  w_IinI = vee(pose_interp.block(0, 0, 3, 3).transpose() * vel_interp.block(0, 0, 3, 3));
  v_IinG = vel_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::get_acceleration(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG, Eigen::Vector3d &w_IinI,
                                  Eigen::Vector3d &v_IinG, Eigen::Vector3d &alpha_IinI, Eigen::Vector3d &a_IinG) {

  // 获取期望时间戳的边界位姿
  double t0, t1, t2, t3;
  Eigen::Matrix4d pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

  // 如果无法获取边界位姿则返回失败
  if (!success) {
    alpha_IinI.setZero();
    a_IinG.setZero();
    return false;
  }

  // 我们的De Boor-Cox矩阵标量
  double DT = (t2 - t1);
  double u = (timestamp - t1) / DT;
  double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  double b2 = 1.0 / 6.0 * (u * u * u);
  double b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
  double b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
  double b2dot = 1.0 / (6.0 * DT) * (3 * u * u);
  double b0dotdot = 1.0 / (6.0 * DT * DT) * (-6 + 6 * u);
  double b1dotdot = 1.0 / (6.0 * DT * DT) * (6 - 12 * u);
  double b2dotdot = 1.0 / (6.0 * DT * DT) * (6 * u);

  // 缓存一些我们经常使用的值
  Eigen::Matrix<double, 6, 1> omega_10 = log_se3(Inv_se3(pose0) * pose1);
  Eigen::Matrix<double, 6, 1> omega_21 = log_se3(Inv_se3(pose1) * pose2);
  Eigen::Matrix<double, 6, 1> omega_32 = log_se3(Inv_se3(pose2) * pose3);
  Eigen::Matrix4d omega_10_hat = hat_se3(omega_10);
  Eigen::Matrix4d omega_21_hat = hat_se3(omega_21);
  Eigen::Matrix4d omega_32_hat = hat_se3(omega_32);

  // 计算插值位姿
  Eigen::Matrix4d A0 = exp_se3(b0 * omega_10);
  Eigen::Matrix4d A1 = exp_se3(b1 * omega_21);
  Eigen::Matrix4d A2 = exp_se3(b2 * omega_32);
  Eigen::Matrix4d A0dot = b0dot * omega_10_hat * A0;
  Eigen::Matrix4d A1dot = b1dot * omega_21_hat * A1;
  Eigen::Matrix4d A2dot = b2dot * omega_32_hat * A2;
  Eigen::Matrix4d A0dotdot = b0dot * omega_10_hat * A0dot + b0dotdot * omega_10_hat * A0;
  Eigen::Matrix4d A1dotdot = b1dot * omega_21_hat * A1dot + b1dotdot * omega_21_hat * A1;
  Eigen::Matrix4d A2dotdot = b2dot * omega_32_hat * A2dot + b2dotdot * omega_32_hat * A2;

  // 获取插值位姿
  Eigen::Matrix4d pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);

  // 获取插值速度
  // 注意: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
  Eigen::Matrix4d vel_interp = pose0 * (A0dot * A1 * A2 + A0 * A1dot * A2 + A0 * A1 * A2dot);
  w_IinI = vee(pose_interp.block(0, 0, 3, 3).transpose() * vel_interp.block(0, 0, 3, 3));
  v_IinG = vel_interp.block(0, 3, 3, 1);

  // 最终获取插值加速度
  // 注意: Rdot = R*skew(omega)
  // 注意: Rdotdot = Rdot*skew(omega) + R*skew(alpha) => R^T*(Rdotdot-Rdot*skew(omega))=skew(alpha)
  Eigen::Matrix4d acc_interp = pose0 * (A0dotdot * A1 * A2 + A0 * A1dotdot * A2 + A0 * A1 * A2dotdot + 2 * A0dot * A1dot * A2 +
                                        2 * A0 * A1dot * A2dot + 2 * A0dot * A1 * A2dot);
  Eigen::Matrix3d omegaskew = pose_interp.block(0, 0, 3, 3).transpose() * vel_interp.block(0, 0, 3, 3);
  alpha_IinI = vee(pose_interp.block(0, 0, 3, 3).transpose() * (acc_interp.block(0, 0, 3, 3) - vel_interp.block(0, 0, 3, 3) * omegaskew));
  a_IinG = acc_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::find_bounding_poses(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0, double &t1,
                                     Eigen::Matrix4d &pose1) {

  // 设置默认值
  t0 = -1;
  t1 = -1;
  pose0 = Eigen::Matrix4d::Identity();
  pose1 = Eigen::Matrix4d::Identity();

  // 查找边界位姿
  bool found_older = false;
  bool found_newer = false;

  // 查找用于插值的边界位姿
  auto lower_bound = poses.lower_bound(timestamp); // 查找时间戳，如果不可用则查找next(timestamp)
  auto upper_bound = poses.upper_bound(timestamp); // 查找next(timestamp)

  if (lower_bound != poses.end()) {
    // 检查下界是否为时间戳
    // 如果不是，则将迭代器移动到前一个时间戳，以便时间戳被边界包围
    if (lower_bound->first == timestamp) {
      found_older = true;
    } else if (lower_bound != poses.begin()) {
      --lower_bound;
      found_older = true;
    }
  }

  if (upper_bound != poses.end()) {
    found_newer = true;
  }

  // 如果我们找到了更旧的，设置它
  if (found_older) {
    t0 = lower_bound->first;
    pose0 = lower_bound->second;
  }

  // 如果我们找到了更新的，设置它
  if (found_newer) {
    t1 = upper_bound->first;
    pose1 = upper_bound->second;
  }

  // 断言时间戳
  if (found_older && found_newer)
    assert(t0 < t1);

  // 如果找到两个边界则返回true
  return (found_older && found_newer);
}

bool BsplineSE3::find_bounding_control_points(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0,
                                              double &t1, Eigen::Matrix4d &pose1, double &t2, Eigen::Matrix4d &pose2, double &t3,
                                              Eigen::Matrix4d &pose3) {

  // 设置默认值
  t0 = -1;
  t1 = -1;
  t2 = -1;
  t3 = -1;
  pose0 = Eigen::Matrix4d::Identity();
  pose1 = Eigen::Matrix4d::Identity();
  pose2 = Eigen::Matrix4d::Identity();
  pose3 = Eigen::Matrix4d::Identity();

  // 获取两个边界位姿
  bool success = find_bounding_poses(timestamp, poses, t1, pose1, t2, pose2);

  // 如果失败则返回false
  if (!success)
    return false;

  // 现在查找下方和上方的位姿
  auto iter_t1 = poses.find(t1);
  auto iter_t2 = poses.find(t2);

  // 检查t1不是第一个时间戳
  if (iter_t1 == poses.begin()) {
    return false;
  }

  // 将更旧的位姿向后移动
  // 将更新的位姿向前移动
  auto iter_t0 = --iter_t1;
  auto iter_t3 = ++iter_t2;

  // 检查它是否有效
  if (iter_t3 == poses.end()) {
    return false;
  }

  // 设置最旧的
  t0 = iter_t0->first;
  pose0 = iter_t0->second;

  // 设置最新的
  t3 = iter_t3->first;
  pose3 = iter_t3->second;

  // 断言时间戳
  if (success) {
    assert(t0 < t1);
    assert(t1 < t2);
    assert(t2 < t3);
  }

  // 如果找到所有四个边界位姿则返回true
  return success;
}
