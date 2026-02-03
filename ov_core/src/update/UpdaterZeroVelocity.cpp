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

#include "UpdaterZeroVelocity.h"

#include "UpdaterHelper.h"

#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterZeroVelocity::UpdaterZeroVelocity(UpdaterOptions &options, NoiseManager &noises, std::shared_ptr<ov_core::FeatureDatabase> db,
                                         std::shared_ptr<Propagator> prop, double gravity_mag, double zupt_max_velocity,
                                         double zupt_noise_multiplier, double zupt_max_disparity)
    : _options(options), _noises(noises), _db(db), _prop(prop), _zupt_max_velocity(zupt_max_velocity),
      _zupt_noise_multiplier(zupt_noise_multiplier), _zupt_max_disparity(zupt_max_disparity) {

  // 初始化重力向量
  _gravity << 0.0, 0.0, gravity_mag;

  // 保存原始像素噪声的平方值
  _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
  _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
  _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
  _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);

  // 初始化置信度为0.95的卡方检验表
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 1000; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterZeroVelocity::feed_imu(const ov_core::ImuData &message, double oldest_time) {

  // 将数据添加到向量中
  imu_data.emplace_back(message);

  // 对IMU数据进行排序（处理任何乱序的测量值）
  // std::sort(imu_data.begin(), imu_data.end(), [](const IMUDATA i, const IMUDATA j) {
  //    return i.timestamp < j.timestamp;
  //});

  // 清理旧的测量值
  // std::cout << "ZVUPT: imu_data.size() " << imu_data.size() << std::endl;
  clean_old_imu_measurements(oldest_time - 0.10);
}

void UpdaterZeroVelocity::clean_old_imu_measurements(double oldest_time) {
  if (oldest_time < 0)
    return;
  auto it0 = imu_data.begin();
  while (it0 != imu_data.end()) {
    if (it0->timestamp < oldest_time) {
      it0 = imu_data.erase(it0);
    } else {
      it0++;
    }
  }
}

bool UpdaterZeroVelocity::try_update(std::shared_ptr<State> state, double timestamp) {

  // 如果还没有IMU数据，则返回
  if (imu_data.empty()) {
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // 如果状态已经在期望的时间，则返回
  if (state->_timestamp == timestamp) {
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // 如果系统刚刚启动，设置上次时间偏移值
  if (!have_last_prop_time_offset) {
    last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
    have_last_prop_time_offset = true;
  }

  // 断言请求的时间在未来
  // assert(timestamp > state->_timestamp);

  // 获取IMU-相机偏移量（t_imu = t_cam + calib_dt）
  double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

  // 首先构建所需的IMU测量向量
  // double time0 = state->_timestamp+t_off_new;
  double time0 = state->_timestamp + last_prop_time_offset;
  double time1 = timestamp + t_off_new;

  // 选择边界内的惯性测量值
  std::vector<ov_core::ImuData> imu_recent = Propagator::select_imu_readings(imu_data, time0, time1);

  // 时间向前推进
  last_prop_time_offset = t_off_new;

  // 检查是否有至少一个测量值用于传播
  if (imu_recent.size() < 2) {
    PRINT_WARNING(RED "[ZUPT]: There are no IMU data to check for zero velocity with!!\n" RESET);
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // 是否应该积分加速度并说速度应该为零
  // 以及是否应该根据随机游走噪声来膨胀偏置
  bool integrated_accel_constraint = false; // 未测试
  bool model_time_varying_bias = true;
  bool override_with_disparity_check = true;
  bool explicitly_enforce_zero_motion = false;

  // 雅可比矩阵的顺序
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->bg());
  Hx_order.push_back(state->_imu->ba());
  if (integrated_accel_constraint) {
    Hx_order.push_back(state->_imu->v());
  }

  // 用于更新的大型最终矩阵（我们将压缩这些矩阵）
  int h_size = (integrated_accel_constraint) ? 12 : 9;
  int m_size = 6 * ((int)imu_recent.size() - 1);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);

  // IMU内参标定估计（静态）
  Eigen::Matrix3d Dw = State::Dm(state->_options.imu_model, state->_calib_imu_dw->value());
  Eigen::Matrix3d Da = State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
  Eigen::Matrix3d Tg = State::Tg(state->_calib_imu_tg->value());

  // 遍历所有IMU数据并构建残差和雅可比矩阵
  // TODO: 应该在这里添加关于IMU内参的雅可比矩阵！！
  // 状态顺序为: [q_GtoI, bg, ba, v_IinG]
  // 测量顺序为: [w_true = 0, a_true = 0 或 v_k+1 = 0]
  // w_true = w_m - bw - nw
  // a_true = a_m - ba - R*g - na
  // v_true = v_k - g*dt + R^T*(a_m - ba - na)*dt
  double dt_summed = 0;
  for (size_t i = 0; i < imu_recent.size() - 1; i++) {

    // 预计算值
    double dt = imu_recent.at(i + 1).timestamp - imu_recent.at(i).timestamp;
    Eigen::Vector3d a_hat = state->_calib_imu_ACCtoIMU->Rot() * Da * (imu_recent.at(i).am - state->_imu->bias_a());
    Eigen::Vector3d w_hat = state->_calib_imu_GYROtoIMU->Rot() * Dw * (imu_recent.at(i).wm - state->_imu->bias_g() - Tg * a_hat);

    // 测量噪声（从连续时间转换为离散时间）
    // 注意: 如果我们"截断"了任何IMU测量值，dt时间可能会不同
    // 注意: 我们正在执行"白化"，因此我们将分解 R_meas^-1 = L*L^t
    // 注意: 然后将其乘以残差和雅可比矩阵（等效于仅使用R_meas进行更新）
    // 注意: 参见Maybeck随机模型、估计和控制第1卷方程(7-21a)-(7-21c)
    double w_omega = std::sqrt(dt) / _noises.sigma_w;
    double w_accel = std::sqrt(dt) / _noises.sigma_a;
    double w_accel_v = 1.0 / (std::sqrt(dt) * _noises.sigma_a);

    // 测量残差（真实值为零）
    res.block(6 * i + 0, 0, 3, 1) = -w_omega * w_hat;
    if (!integrated_accel_constraint) {
      res.block(6 * i + 3, 0, 3, 1) = -w_accel * (a_hat - state->_imu->Rot() * _gravity);
    } else {
      res.block(6 * i + 3, 0, 3, 1) = -w_accel_v * (state->_imu->vel() - _gravity * dt + state->_imu->Rot().transpose() * a_hat * dt);
    }

    // 测量雅可比矩阵
    Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
    H.block(6 * i + 0, 3, 3, 3) = -w_omega * Eigen::Matrix3d::Identity();
    if (!integrated_accel_constraint) {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel * skew_x(R_GtoI_jacob * _gravity);
      H.block(6 * i + 3, 6, 3, 3) = -w_accel * Eigen::Matrix3d::Identity();
    } else {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * skew_x(a_hat) * dt;
      H.block(6 * i + 3, 6, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * dt;
      H.block(6 * i + 3, 9, 3, 3) = w_accel_v * Eigen::Matrix3d::Identity();
    }
    dt_summed += dt;
  }

  // 压缩系统（应该是超定的）
  UpdaterHelper::measurement_compress_inplace(H, res);
  if (H.rows() < 1) {
    return false;
  }

  // 将噪声矩阵乘以固定倍数
  // 我们通常需要将IMU视为"最差"情况以进行检测/避免过度自信
  Eigen::MatrixXd R = _zupt_noise_multiplier * Eigen::MatrixXd::Identity(res.rows(), res.rows());

  // 接下来将偏置向前传播
  // 注意: G*Qd*G^t = dt*Qd*dt = dt*(1/dt*Qc)*dt = dt*Qc
  Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6, 6);
  Q_bias.block(0, 0, 3, 3) *= dt_summed * _noises.sigma_wb_2;
  Q_bias.block(3, 3, 3, 3) *= dt_summed * _noises.sigma_ab_2;

  // Chi2距离检查
  // 注意: 如果这将被接受，我们还会附加在更新之前"将要做"的传播（仅偏置演化）
  // 注意: 我们不首先传播，因为如果我们chi2失败，我们只想返回并执行正常逻辑
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  if (model_time_varying_bias) {
    P_marg.block(3, 3, 6, 6) += Q_bias;
  }
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  double chi2 = res.dot(S.llt().solve(res));

  // 获取阈值（我们预计算到1000，但处理超过的情况）
  double chi2_check;
  if (res.rows() < 1000) {
    chi2_check = chi_squared_table[res.rows()];
  } else {
    boost::math::chi_squared chi_squared_dist(res.rows());
    chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    PRINT_WARNING(YELLOW "[ZUPT]: chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
  }

  // 检查图像视差
  bool disparity_passed = false;
  if (override_with_disparity_check) {

    // 获取从当前图像到前一图像的视差统计信息
    double time0_cam = state->_timestamp;
    double time1_cam = timestamp;
    int num_features = 0;
    double disp_avg = 0.0;
    double disp_var = 0.0;
    FeatureHelper::compute_disparity(_db, time0_cam, time1_cam, disp_avg, disp_var, num_features);

    // 检查此视差是否足以被分类为运动
    disparity_passed = (disp_avg < _zupt_max_disparity && num_features > 20);
    if (disparity_passed) {
      PRINT_DEBUG(CYAN "[ZUPT]: disparity passed(%.3f < %.3f, %d features)\n" RESET, disp_avg, _zupt_max_disparity, (int)num_features);
    } else {
      PRINT_INFO(YELLOW "[ZUPT]: disparity failed (%.3f > %.3f, %d features)\n" RESET, disp_avg, _zupt_max_disparity, (int)num_features);
    }
  }

  // 检查我们当前是否为零速度
  // 我们需要通过chi2检查且不超过速度阈值
  if (!disparity_passed && (chi2 > _options.chi2_multipler * chi2_check || state->_imu->vel().norm() > _zupt_max_velocity)) {
    last_zupt_state_timestamp = 0.0;
    last_zupt_count = 0;
    PRINT_INFO(YELLOW "[ZUPT]: rejected |v_IinG| = %.3f (chi2 %.3f > %.3f)\n" RESET, state->_imu->vel().norm(), chi2,
                _options.chi2_multipler * chi2_check);
    return false;
  }

  // 执行更新，只有在之前检测到的情况下才执行此更新
  // 如果成功，我们应该移除当前时间戳的特征轨迹
  // 这是因为我们不会在此时间步进行克隆，而是执行零速度更新
  // 注意: 我们希望保留第二次调用zv-upt时的轨迹，因为这次不会有克隆
  // 注意: 第二次调用此函数之后的所有未来时间也*不会*有克隆，因此我们可以移除那些
  if (last_zupt_count >= 2) {
    _db->cleanup_measurements_exact(last_zupt_state_timestamp);
  }

  // 否则我们没问题，更新系统
  // 1) 直接用IMU测量值更新
  // 2) 传播然后明确说明我们的姿态、位置和速度应该为零
  if (!explicitly_enforce_zero_motion) {

    // 接下来将偏置向前传播
    // 注意: G*Qd*G^t = dt*Qd*dt = dt*Qc
    if (model_time_varying_bias) {
      Eigen::MatrixXd Phi_bias = Eigen::MatrixXd::Identity(6, 6);
      std::vector<std::shared_ptr<Type>> Phi_order;
      Phi_order.push_back(state->_imu->bg());
      Phi_order.push_back(state->_imu->ba());
      StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_bias, Q_bias);
    }

    // 最后将状态时间向前推进
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    state->_timestamp = timestamp;

  } else {

    // 将状态向前传播
    double time0_cam = last_zupt_state_timestamp;
    double time1_cam = timestamp;
    _prop->propagate_and_clone(state, time1_cam);

    // 创建更新系统！
    H = Eigen::MatrixXd::Zero(9, 15);
    res = Eigen::VectorXd::Zero(9);
    R = Eigen::MatrixXd::Identity(9, 9);

    // 残差（顺序为姿态、位置、速度）
    Eigen::Matrix3d R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot();
    Eigen::Vector3d p_I0inG = state->_clones_IMU.at(time0_cam)->pos();
    Eigen::Matrix3d R_GtoI1 = state->_clones_IMU.at(time1_cam)->Rot();
    Eigen::Vector3d p_I1inG = state->_clones_IMU.at(time1_cam)->pos();
    res.block(0, 0, 3, 1) = -log_so3(R_GtoI0 * R_GtoI1.transpose());
    res.block(3, 0, 3, 1) = p_I1inG - p_I0inG;
    res.block(6, 0, 3, 1) = state->_imu->vel();
    res *= -1;

    // 雅可比矩阵（顺序为q0, p0, q1, p1, v0）
    Hx_order.clear();
    Hx_order.push_back(state->_clones_IMU.at(time0_cam));
    Hx_order.push_back(state->_clones_IMU.at(time1_cam));
    Hx_order.push_back(state->_imu->v());
    if (state->_options.do_fej) {
      R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot_fej();
    }
    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(0, 6, 3, 3) = -R_GtoI0;
    H.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();
    H.block(3, 9, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(6, 12, 3, 3) = Eigen::Matrix3d::Identity();

    // 噪声（顺序为姿态、位置、速度）
    R.block(0, 0, 3, 3) *= std::pow(1e-2, 2);
    R.block(3, 3, 3, 3) *= std::pow(1e-1, 2);
    R.block(6, 6, 3, 3) *= std::pow(1e-1, 2);

    // 最后更新并移除旧的克隆
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    StateHelper::marginalize(state, state->_clones_IMU.at(time1_cam));
    state->_clones_IMU.erase(time1_cam);
  }

  // 最后返回
  last_zupt_state_timestamp = timestamp;
  last_zupt_count++;
  return true;
}