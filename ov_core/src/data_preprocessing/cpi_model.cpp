// CPI Model 实现：解析均值/雅可比、协方差 RK4 积分。
// 使用 ov_core quat_ops（与 CpiV1 一致），无 Sophus 依赖。

#include "data_preprocessing/cpi_model.h"

#include <cmath>

#include "utils/quat_ops.h"

namespace data_preprocessing {

namespace {

constexpr double kSmallW = 0.008726646;  // ~0.5°/s，小角阈值。

// 旋转矩阵 -> 四元数 (w,x,y,z)，与 CpiModel 对外约定一致。
inline void RToQuatWxyz(const Eigen::Matrix3d& R, Eigen::Vector4d* q) {
  Eigen::Matrix<double, 4, 1> q_jpl = ov_core::rot_2_quat(R);
  (*q)(0) = q_jpl(3);  // w (JPL scalar)
  (*q)(1) = q_jpl(0);  // x
  (*q)(2) = q_jpl(1);  // y
  (*q)(3) = q_jpl(2);  // z
}

}  // namespace

CpiModel::CpiModel(double sigma_w, double sigma_wb, double sigma_a,
                   double sigma_ab, bool imu_avg) {
  imu_avg_ = imu_avg;
  Q_c_.setZero();
  Q_c_.block<3, 3>(0, 0) = (sigma_w * sigma_w) * Matrix3::Identity();
  Q_c_.block<3, 3>(3, 3) = (sigma_wb * sigma_wb) * Matrix3::Identity();
  Q_c_.block<3, 3>(6, 6) = (sigma_a * sigma_a) * Matrix3::Identity();
  Q_c_.block<3, 3>(9, 9) = (sigma_ab * sigma_ab) * Matrix3::Identity();
  e_1_ << 1, 0, 0;
  e_2_ << 0, 1, 0;
  e_3_ << 0, 0, 1;
  e_1x_ = ov_core::skew_x(e_1_);
  e_2x_ = ov_core::skew_x(e_2_);
  e_3x_ = ov_core::skew_x(e_3_);
}

void CpiModel::InitBias(const Vector3& b_w, const Vector3& b_a) {
  b_w_lin_ = b_w;
  b_a_lin_ = b_a;
}

void CpiModel::FeedImu(double t_0, double t_1, const Vector3& w_m_0,
                       const Vector3& a_m_0, const Vector3& w_m_1,
                       const Vector3& a_m_1) {
  const double delta_t = t_1 - t_0;
  DT += delta_t;

  if (delta_t == 0) {
    return;
  }

  Vector3 w_hat = w_m_0 - b_w_lin_;
  Vector3 a_hat = a_m_0 - b_a_lin_;
  if (imu_avg_) {
    w_hat += w_m_1 - b_w_lin_;
    w_hat *= 0.5;
    a_hat += a_m_1 - b_a_lin_;
    a_hat *= 0.5;
  }

  const Vector3 w_hatdt = w_hat * delta_t;
  const double w_1 = w_hat(0), w_2 = w_hat(1), w_3 = w_hat(2);
  const double mag_w = w_hat.norm();
  const double w_dt = mag_w * delta_t;
  const bool small_w = (mag_w < kSmallW);

  const double dt_2 = delta_t * delta_t;
  const double cos_wt = std::cos(w_dt);
  const double sin_wt = std::sin(w_dt);

  const Matrix3 w_x = ov_core::skew_x(w_hat);
  const Matrix3 a_x = ov_core::skew_x(a_hat);
  const Matrix3 w_tx = ov_core::skew_x(w_hatdt);
  const Matrix3 w_x_2 = w_x * w_x;

  // 测量均值
  Matrix3 R_tau2tau1;
  if (small_w) {
    R_tau2tau1 = eye3_ - delta_t * w_x + (dt_2 / 2.0) * w_x_2;
  } else {
    const double mag_w2 = mag_w * mag_w;
    R_tau2tau1 =
        eye3_ - (sin_wt / mag_w) * w_x + ((1.0 - cos_wt) / mag_w2) * w_x_2;
  }

  Matrix3 R_k2tau1 = R_tau2tau1 * R_k2tau;
  Matrix3 R_tau12k = R_k2tau1.transpose();

  double f_1, f_2, f_3, f_4;
  if (small_w) {
    f_1 = -(std::pow(delta_t, 3) / 3.0);
    f_2 = (std::pow(delta_t, 4) / 8.0);
    f_3 = -(dt_2 / 2.0);
    f_4 = (std::pow(delta_t, 3) / 6.0);
  } else {
    const double mag_w3 = mag_w * mag_w * mag_w;
    const double mag_w4 = mag_w3 * mag_w;
    f_1 = (w_dt * cos_wt - sin_wt) / mag_w3;
    f_2 = (w_dt * w_dt - 2.0 * cos_wt - 2.0 * w_dt * sin_wt + 2.0) /
          (2.0 * mag_w4);
    f_3 = -(1.0 - cos_wt) / (mag_w * mag_w);
    f_4 = (w_dt - sin_wt) / mag_w3;
  }

  const Matrix3 alpha_arg = (dt_2 / 2.0) * eye3_ + f_1 * w_x + f_2 * w_x_2;
  const Matrix3 Beta_arg = delta_t * eye3_ + f_3 * w_x + f_4 * w_x_2;
  const Matrix3 H_al = R_tau12k * alpha_arg;
  const Matrix3 H_be = R_tau12k * Beta_arg;

  alpha_tau += beta_tau * delta_t + H_al * a_hat;
  beta_tau += H_be * a_hat;

  // 偏置雅可比（解析）
  Matrix3 J_r_tau1;
  if (small_w) {
    J_r_tau1 = eye3_ - 0.5 * w_tx + (1.0 / 6.0) * w_tx * w_tx;
  } else {
    const double w_dt2 = w_dt * w_dt;
    const double w_dt3 = w_dt2 * w_dt;
    J_r_tau1 = eye3_ - ((1.0 - cos_wt) / w_dt2) * w_tx +
               ((w_dt - sin_wt) / w_dt3) * w_tx * w_tx;
  }

  J_q = R_tau2tau1 * J_q + J_r_tau1 * delta_t;
  H_a = H_a - H_al + delta_t * H_b;
  H_b = H_b - H_be;

  const Matrix3 d_R_bw_1 = -R_tau12k * ov_core::skew_x(J_q * e_1_);
  const Matrix3 d_R_bw_2 = -R_tau12k * ov_core::skew_x(J_q * e_2_);
  const Matrix3 d_R_bw_3 = -R_tau12k * ov_core::skew_x(J_q * e_3_);

  double df_1_dbw_1, df_1_dbw_2, df_1_dbw_3;
  double df_2_dbw_1, df_2_dbw_2, df_2_dbw_3;
  double df_3_dbw_1, df_3_dbw_2, df_3_dbw_3;
  double df_4_dbw_1, df_4_dbw_2, df_4_dbw_3;

  if (small_w) {
    const double df_1_dw = -(std::pow(delta_t, 5) / 15.0);
    df_1_dbw_1 = w_1 * df_1_dw;
    df_1_dbw_2 = w_2 * df_1_dw;
    df_1_dbw_3 = w_3 * df_1_dw;
    const double df_2_dw = (std::pow(delta_t, 6) / 72.0);
    df_2_dbw_1 = w_1 * df_2_dw;
    df_2_dbw_2 = w_2 * df_2_dw;
    df_2_dbw_3 = w_3 * df_2_dw;
    const double df_3_dw = -(std::pow(delta_t, 4) / 12.0);
    df_3_dbw_1 = w_1 * df_3_dw;
    df_3_dbw_2 = w_2 * df_3_dw;
    df_3_dbw_3 = w_3 * df_3_dw;
    const double df_4_dw = (std::pow(delta_t, 5) / 60.0);
    df_4_dbw_1 = w_1 * df_4_dw;
    df_4_dbw_2 = w_2 * df_4_dw;
    df_4_dbw_3 = w_3 * df_4_dw;
  } else {
    const double mag_w5 = std::pow(mag_w, 5);
    const double mag_w6 = mag_w5 * mag_w;
    const double mag_w4 = std::pow(mag_w, 4);
    df_1_dbw_1 = w_1 *
                 (w_dt * w_dt * sin_wt - 3.0 * sin_wt + 3.0 * w_dt * cos_wt) /
                 mag_w5;
    df_1_dbw_2 = w_2 *
                 (w_dt * w_dt * sin_wt - 3.0 * sin_wt + 3.0 * w_dt * cos_wt) /
                 mag_w5;
    df_1_dbw_3 = w_3 *
                 (w_dt * w_dt * sin_wt - 3.0 * sin_wt + 3.0 * w_dt * cos_wt) /
                 mag_w5;
    df_2_dbw_1 = w_1 *
                 (w_dt * w_dt - 4.0 * cos_wt - 4.0 * w_dt * sin_wt +
                  w_dt * w_dt * cos_wt + 4.0) /
                 mag_w6;
    df_2_dbw_2 = w_2 *
                 (w_dt * w_dt - 4.0 * cos_wt - 4.0 * w_dt * sin_wt +
                  w_dt * w_dt * cos_wt + 4.0) /
                 mag_w6;
    df_2_dbw_3 = w_3 *
                 (w_dt * w_dt - 4.0 * cos_wt - 4.0 * w_dt * sin_wt +
                  w_dt * w_dt * cos_wt + 4.0) /
                 mag_w6;
    df_3_dbw_1 = w_1 * (2.0 * (cos_wt - 1.0) + w_dt * sin_wt) / mag_w4;
    df_3_dbw_2 = w_2 * (2.0 * (cos_wt - 1.0) + w_dt * sin_wt) / mag_w4;
    df_3_dbw_3 = w_3 * (2.0 * (cos_wt - 1.0) + w_dt * sin_wt) / mag_w4;
    df_4_dbw_1 = w_1 * (2.0 * w_dt + w_dt * cos_wt - 3.0 * sin_wt) / mag_w5;
    df_4_dbw_2 = w_2 * (2.0 * w_dt + w_dt * cos_wt - 3.0 * sin_wt) / mag_w5;
    df_4_dbw_3 = w_3 * (2.0 * w_dt + w_dt * cos_wt - 3.0 * sin_wt) / mag_w5;
  }

  J_a += J_b * delta_t;
  J_a.col(0) +=
      (d_R_bw_1 * alpha_arg +
       R_tau12k * (df_1_dbw_1 * w_x - f_1 * e_1x_ + df_2_dbw_1 * w_x_2 -
                   f_2 * (e_1x_ * w_x + w_x * e_1x_))) *
      a_hat;
  J_a.col(1) +=
      (d_R_bw_2 * alpha_arg +
       R_tau12k * (df_1_dbw_2 * w_x - f_1 * e_2x_ + df_2_dbw_2 * w_x_2 -
                   f_2 * (e_2x_ * w_x + w_x * e_2x_))) *
      a_hat;
  J_a.col(2) +=
      (d_R_bw_3 * alpha_arg +
       R_tau12k * (df_1_dbw_3 * w_x - f_1 * e_3x_ + df_2_dbw_3 * w_x_2 -
                   f_2 * (e_3x_ * w_x + w_x * e_3x_))) *
      a_hat;
  J_b.col(0) +=
      (d_R_bw_1 * Beta_arg +
       R_tau12k * (df_3_dbw_1 * w_x - f_3 * e_1x_ + df_4_dbw_1 * w_x_2 -
                   f_4 * (e_1x_ * w_x + w_x * e_1x_))) *
      a_hat;
  J_b.col(1) +=
      (d_R_bw_2 * Beta_arg +
       R_tau12k * (df_3_dbw_2 * w_x - f_3 * e_2x_ + df_4_dbw_2 * w_x_2 -
                   f_4 * (e_2x_ * w_x + w_x * e_2x_))) *
      a_hat;
  J_b.col(2) +=
      (d_R_bw_3 * Beta_arg +
       R_tau12k * (df_3_dbw_3 * w_x - f_3 * e_3x_ + df_4_dbw_3 * w_x_2 -
                   f_4 * (e_3x_ * w_x + w_x * e_3x_))) *
      a_hat;

  // 测量协方差（RK4 积分 Lyapunov）
  Matrix3 R_mid;
  if (small_w) {
    const double half_dt = 0.5 * delta_t;
    R_mid = eye3_ - half_dt * w_x + (half_dt * half_dt / 2.0) * w_x_2;
  } else {
    const double half_wt = mag_w * 0.5 * delta_t;
    R_mid = eye3_ - (std::sin(half_wt) / mag_w) * w_x +
            ((1.0 - std::cos(half_wt)) / (mag_w * mag_w)) * w_x_2;
  }
  R_mid = R_mid * R_k2tau;

  using Matrix15 = Eigen::Matrix<double, 15, 15>;

  auto build_F = [&w_x, &a_x](const Matrix3& R) -> Matrix15 {
    Matrix15 F = Matrix15::Zero();
    F.block<3, 3>(0, 0) = -w_x;
    F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
    F.block<3, 3>(6, 0) = -R.transpose() * a_x;
    F.block<3, 3>(6, 9) = -R.transpose();
    F.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity();
    return F;
  };

  auto cov_dot = [this](const Matrix15& F,
                        const Eigen::Matrix<double, 15, 12>& Gk,
                        const Matrix15& P) -> Matrix15 {
    return F * P + P * F.transpose() + Gk * Q_c_ * Gk.transpose();
  };

  Eigen::Matrix<double, 15, 12> G_k1;
  G_k1.setZero();
  G_k1.block<3, 3>(0, 0) = -eye3_;
  G_k1.block<3, 3>(3, 3) = eye3_;
  G_k1.block<3, 3>(6, 6) = -R_k2tau.transpose();
  G_k1.block<3, 3>(9, 9) = eye3_;

  Matrix15 F_k1 = build_F(R_k2tau);
  Matrix15 P_dot_k1 = cov_dot(F_k1, G_k1, P_meas_);

  Eigen::Matrix<double, 15, 12> G_k2;
  G_k2.setZero();
  G_k2.block<3, 3>(0, 0) = -eye3_;
  G_k2.block<3, 3>(3, 3) = eye3_;
  G_k2.block<3, 3>(6, 6) = -R_mid.transpose();
  G_k2.block<3, 3>(9, 9) = eye3_;
  Matrix15 F_k2 = build_F(R_mid);
  Matrix15 P_k2 = P_meas_ + P_dot_k1 * (delta_t / 2.0);
  Matrix15 P_dot_k2 = cov_dot(F_k2, G_k2, P_k2);

  Matrix15 P_k3 = P_meas_ + P_dot_k2 * (delta_t / 2.0);
  Matrix15 P_dot_k3 = cov_dot(F_k2, G_k2, P_k3);

  Eigen::Matrix<double, 15, 12> G_k4;
  G_k4.setZero();
  G_k4.block<3, 3>(0, 0) = -eye3_;
  G_k4.block<3, 3>(3, 3) = eye3_;
  G_k4.block<3, 3>(6, 6) = -R_k2tau1.transpose();
  G_k4.block<3, 3>(9, 9) = eye3_;
  Matrix15 F_k4 = build_F(R_k2tau1);
  Matrix15 P_k4 = P_meas_ + P_dot_k3 * delta_t;
  Matrix15 P_dot_k4 = cov_dot(F_k4, G_k4, P_k4);

  P_meas_ +=
      (delta_t / 6.0) * (P_dot_k1 + 2.0 * P_dot_k2 + 2.0 * P_dot_k3 + P_dot_k4);
  P_meas_ = 0.5 * (P_meas_ + P_meas_.transpose());

  R_k2tau = R_k2tau1;
  RToQuatWxyz(R_k2tau, &q_k2tau);
}

}  // namespace data_preprocessing
