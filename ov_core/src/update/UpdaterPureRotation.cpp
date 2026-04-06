/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 */

#include "UpdaterPureRotation.h"

#include "UpdaterHelper.h"
#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/chi_squared_quantile.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <algorithm>
#include <cmath>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterPureRotation::UpdaterPureRotation(
    UpdaterOptions &options, NoiseManager &noises,
    std::shared_ptr<ov_core::FeatureDatabase> db, double gravity_mag,
    double max_velocity, double gyro_mag_min, double gyro_mag_max,
    double vel_sigma, double noise_multiplier, bool print_pure_rot,
    bool use_image_gate, double flow_perp_min,
    double flow_sign_consistency_min, int min_flow_feats, double min_r_px,
    double min_flow_px, bool use_cam_z_gyro_gate,
    double min_omega_cam_z_ratio, size_t ref_cam_id)
    : _options(options), _noises(noises), _db(std::move(db)),
      _max_velocity(max_velocity), _gyro_mag_min(gyro_mag_min),
      _gyro_mag_max(gyro_mag_max), _vel_sigma(std::max(1e-6, vel_sigma)),
      _noise_multiplier(noise_multiplier), _print_pure_rot(print_pure_rot),
      _use_image_gate(use_image_gate), _flow_perp_min(flow_perp_min),
      _flow_sign_consistency_min(flow_sign_consistency_min),
      _min_flow_feats(min_flow_feats), _min_r_px(min_r_px),
      _min_flow_px(min_flow_px), _use_cam_z_gyro_gate(use_cam_z_gyro_gate),
      _min_omega_cam_z_ratio(min_omega_cam_z_ratio), _ref_cam_id(ref_cam_id) {
  _gravity << 0.0, 0.0, gravity_mag;
  _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
  _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
  _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
  _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);
}

void UpdaterPureRotation::feed_imu(const ov_core::ImuData &message,
                                   double oldest_time) {
  imu_data.emplace_back(message);
  clean_old_imu_measurements(oldest_time - 0.10);
}

void UpdaterPureRotation::clean_old_imu_measurements(double oldest_time) {
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

bool UpdaterPureRotation::try_update(std::shared_ptr<State> state,
                                     double timestamp) {
  auto clear_snap = [this]() {
    _last_debug = PureRotDebugSnapshot{};
  };

  if (imu_data.empty()) {
    clear_snap();
    return false;
  }
  if (state->_timestamp == timestamp)
    return false;

  if (!have_last_prop_time_offset) {
    last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
    have_last_prop_time_offset = true;
  }

  double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);
  double time0 = state->_timestamp + last_prop_time_offset;
  double time1 = timestamp + t_off_new;
  last_prop_time_offset = t_off_new;

  std::vector<ov_core::ImuData> imu_recent =
      Propagator::select_imu_readings(imu_data, time0, time1);
  if (imu_recent.size() < 2) {
    clear_snap();
    if (_print_pure_rot) {
      PRINT_WARNING(RED "[PureRot]: IMU 样本不足 (n=%zu)\n" RESET,
                    imu_recent.size());
    }
    return false;
  }

  Eigen::Matrix3d Dw =
      State::Dm(state->_options.imu_model, state->_calib_imu_dw->value());
  Eigen::Matrix3d Da =
      State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
  Eigen::Matrix3d Tg = State::Tg(state->_calib_imu_tg->value());

  const int nseg = (int)imu_recent.size() - 1;
  const int m_accel = 3 * nseg;
  const int m_vel = 3;

  Eigen::MatrixXd H =
      Eigen::MatrixXd::Zero(m_accel + m_vel, 12); // q, bg, ba, v
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_accel + m_vel);

  double dt_summed = 0;
  double gyro_mag_sum = 0.0;
  Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();

  for (int i = 0; i < nseg; i++) {
    double dt = imu_recent.at(i + 1).timestamp - imu_recent.at(i).timestamp;
    Eigen::Vector3d a_hat = state->_calib_imu_ACCtoIMU->Rot() * Da *
                            (imu_recent.at(i).am - state->_imu->bias_a());
    Eigen::Vector3d w_hat =
        state->_calib_imu_GYROtoIMU->Rot() * Dw *
        (imu_recent.at(i).wm - state->_imu->bias_g() - Tg * a_hat);

    w_sum += w_hat;
    gyro_mag_sum += w_hat.norm();

    double w_accel = std::sqrt(dt) / _noises.sigma_a;
    res.block(3 * i, 0, 3, 1) =
        -w_accel * (a_hat - state->_imu->Rot() * _gravity);

    Eigen::Matrix3d R_GtoI_jacob =
        (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
    H.block(3 * i, 0, 3, 3) = -w_accel * skew_x(R_GtoI_jacob * _gravity);
    H.block(3 * i, 3, 3, 3) = -w_accel * Eigen::Matrix3d::Identity();
    H.block(3 * i, 6, 3, 3) = -w_accel * Eigen::Matrix3d::Identity();
    dt_summed += dt;
  }

  const double gyro_avg = gyro_mag_sum / (double)imu_recent.size();
  const double w_vel = 1.0 / _vel_sigma;
  res.block(m_accel, 0, 3, 1) = -w_vel * state->_imu->vel();
  H.block(m_accel, 9, 3, 3) = w_vel * Eigen::Matrix3d::Identity();

  UpdaterHelper::measurement_compress_inplace(H, res);
  if (H.rows() < 1) {
    clear_snap();
    if (_print_pure_rot) {
      PRINT_WARNING(RED "[PureRot]: 测量压缩后无残差行\n" RESET);
    }
    return false;
  }

  Eigen::MatrixXd R = _noise_multiplier *
                      Eigen::MatrixXd::Identity(res.rows(), res.rows());

  Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6, 6);
  Q_bias.block(0, 0, 3, 3) *= dt_summed * _noises.sigma_wb_2;
  Q_bias.block(3, 3, 3, 3) *= dt_summed * _noises.sigma_ab_2;

  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->bg());
  Hx_order.push_back(state->_imu->ba());
  Hx_order.push_back(state->_imu->v());

  Eigen::MatrixXd P_marg =
      StateHelper::get_marginal_covariance(state, Hx_order);
  P_marg.block(3, 3, 6, 6) += Q_bias;

  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  double chi2 = res.dot(S.llt().solve(res));
  double chi2_check = ov_core::chi2_quantile_095(static_cast<int>(res.rows()));
  double chi2_lim = _options.chi2_multipler * chi2_check;

  double flow_perp_mean = -1.0;
  double flow_sign_consistency = -1.0;
  int n_flow = 0;
  if (_use_image_gate && _db != nullptr &&
      !state->_cam_intrinsics_cameras.empty()) {
    FeatureHelper::compute_optical_axis_flow_confidence(
        _db, state->_timestamp, timestamp, state->_cam_intrinsics_cameras,
        _min_r_px, _min_flow_px, flow_perp_mean, flow_sign_consistency, n_flow);
  }

  Eigen::Vector3d w_mean = w_sum / std::max(1.0, (double)nseg);
  double omega_cam_z_ratio = 0.0;
  if (_use_cam_z_gyro_gate &&
      state->_calib_IMUtoCAM.find(_ref_cam_id) !=
          state->_calib_IMUtoCAM.end()) {
    Eigen::Matrix3d R_ItoC =
        state->_calib_IMUtoCAM.at(_ref_cam_id)->Rot();
    Eigen::Vector3d w_cam = R_ItoC * w_mean;
    double wn = w_cam.norm();
    omega_cam_z_ratio = (wn > 1e-9) ? (std::abs(w_cam(2)) / wn) : 0.0;
  }

  const bool vel_ok = state->_imu->vel().norm() <= _max_velocity;
  const bool gyro_ok =
      (gyro_avg >= _gyro_mag_min) && (gyro_avg <= _gyro_mag_max);
  const bool chi2_ok = chi2 <= chi2_lim;

  const bool img_has_samples = (flow_perp_mean >= 0.0);
  const bool nf_ok = (n_flow >= _min_flow_feats);
  const bool perp_ok =
      img_has_samples && (flow_perp_mean >= _flow_perp_min);
  const bool sign_ok = (flow_sign_consistency >= 0.0) &&
                         (flow_sign_consistency >= _flow_sign_consistency_min);

  bool img_ok = true;
  if (_use_image_gate) {
    img_ok = img_has_samples && nf_ok && perp_ok && sign_ok;
  }

  bool imu_axis_ok = true;
  if (_use_cam_z_gyro_gate) {
    imu_axis_ok = omega_cam_z_ratio >= _min_omega_cam_z_ratio;
  }

  const bool gate_passed =
      vel_ok && gyro_ok && chi2_ok && img_ok && imu_axis_ok;

  _last_debug.has_data = true;
  _last_debug.gate_passed = gate_passed;
  _last_debug.ekf_updated = false;
  _last_debug.n_imu_samples = (int)imu_recent.size();
  _last_debug.residual_rows = (int)res.rows();
  _last_debug.vel_norm = state->_imu->vel().norm();
  _last_debug.gyro_avg_rad_s = gyro_avg;
  _last_debug.chi2 = chi2;
  _last_debug.chi2_lim = chi2_lim;
  _last_debug.vel_ok = vel_ok;
  _last_debug.gyro_ok = gyro_ok;
  _last_debug.chi2_ok = chi2_ok;
  _last_debug.flow_perp_mean = flow_perp_mean;
  _last_debug.flow_sign_consistency = flow_sign_consistency;
  _last_debug.n_flow_feats = n_flow;
  _last_debug.img_ok = img_ok;
  _last_debug.omega_cam_z_ratio = omega_cam_z_ratio;
  _last_debug.imu_axis_ok = imu_axis_ok;

  if (_print_pure_rot) {
    const char *ok = "通过";
    const char *no = "未通过";
    PRINT_INFO(CYAN "[PureRot] 总判定: %s\n" RESET,
               gate_passed ? ok : no);
    PRINT_INFO("  ├─ [速度] %s  |v|=%.4f m/s  门限≤%.4f\n",
               vel_ok ? ok : no, _last_debug.vel_norm, _max_velocity);
    PRINT_INFO("  ├─ [陀螺幅值] %s  avg|ω|=%.4f rad/s  区间[%.4f, %.4f]\n",
               gyro_ok ? ok : no, gyro_avg, _gyro_mag_min, _gyro_mag_max);
    PRINT_INFO("  ├─ [χ²] %s  %.3f / %.3f (mult×χ²_0.95)\n",
               chi2_ok ? ok : no, chi2, chi2_lim);
    if (_use_image_gate) {
      PRINT_INFO("  ├─ [图像] 合成 %s\n", img_ok ? ok : no);
      PRINT_INFO("  │    有样本 %s  perp均值=%.3f  门限≥%.3f → %s\n",
                 img_has_samples ? ok : no, flow_perp_mean, _flow_perp_min,
                 perp_ok ? ok : no);
      PRINT_INFO("  │    特征数 %s  nf=%d  门限≥%d → %s\n",
                 nf_ok ? ok : no, n_flow, _min_flow_feats, nf_ok ? ok : no);
      PRINT_INFO("  │    转向一致 %s  sign=%.3f  门限≥%.3f → %s  (r×f 符号)\n",
                 sign_ok ? ok : no, flow_sign_consistency,
                 _flow_sign_consistency_min, sign_ok ? ok : no);
    } else {
      PRINT_INFO("  ├─ [图像] 已关闭 (pure_rot_use_image_gate=0)\n");
    }
    if (_use_cam_z_gyro_gate) {
      PRINT_INFO("  ├─ [光轴陀螺] %s  |ωz|/|ω|=%.3f  门限≥%.3f (相机系 ref_cam=%zu)\n",
                 imu_axis_ok ? ok : no, omega_cam_z_ratio,
                 _min_omega_cam_z_ratio, (size_t)_ref_cam_id);
    } else {
      PRINT_INFO("  ├─ [光轴陀螺] 已关闭 (pure_rot_use_cam_z_gyro_gate=0)\n");
    }
    PRINT_INFO("  └─ [EKF] IMU样本=%d 残差维=%d  R×%.3f  vσ=%.4f m/s\n",
               _last_debug.n_imu_samples, _last_debug.residual_rows,
               _noise_multiplier, _vel_sigma);
  }

  if (!gate_passed)
    return false;

  Eigen::MatrixXd Phi_bias = Eigen::MatrixXd::Identity(6, 6);
  std::vector<std::shared_ptr<Type>> Phi_order;
  Phi_order.push_back(state->_imu->bg());
  Phi_order.push_back(state->_imu->ba());
  StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_bias, Q_bias);

  R = _noise_multiplier * Eigen::MatrixXd::Identity(res.rows(), res.rows());
  StateHelper::EKFUpdate(state, Hx_order, H, res, R);
  state->_timestamp = timestamp;
  _last_debug.ekf_updated = true;
  return true;
}
