// Copyright 2025 the project authors. CPI Model：分段常数测量下的连续预积分。
// 详见 docs/design/连续预积分-CPI-V1-设计文档.md
//
// 参考文献: Eckenhoff K, Geneva P, Huang G. IJRR 2019; WAFR 2016.
//           http://udel.edu/~ghuang/papers/tr_cpi.pdf

#ifndef MYVIO_DATA_PREPROCESSING_CPI_MODEL_H_
#define MYVIO_DATA_PREPROCESSING_CPI_MODEL_H_

#include <Eigen/Dense>


namespace data_preprocessing {

// CPI Model：每步内角速度、加速度为常数或两端平均。使用顺序：(1) 本段前调用一次
// InitBias()；(2) 对每对 IMU 调用 FeedImu()；(3) 读取 DT、alpha_tau、beta_tau、
// R_k2tau、q_k2tau、J_*、H_*、P_meas。
class CpiModel {
 public:
  using Vector3 = Eigen::Vector3d;
  using Vector4 = Eigen::Vector4d;
  using Matrix3 = Eigen::Matrix3d;
  using Matrix12 = Eigen::Matrix<double, 12, 12>;
  using Matrix15 = Eigen::Matrix<double, 15, 15>;

  // sigma_w: 陀螺白噪声 (rad/s/sqrt(Hz))；sigma_wb: 陀螺随机游走；
  // sigma_a: 加表白噪声 (m/s^2/sqrt(Hz))；sigma_ab: 加表随机游走；
  // imu_avg: 为 true 时对两端 (w,a) 取平均。
  CpiModel(double sigma_w, double sigma_wb, double sigma_a, double sigma_ab,
           bool imu_avg = false);

  // 设置本段预积分使用的 bias。在 FeedImu 前调用一次。w_hat = w_m - b_w，
  // a_hat = a_m - b_a；雅可比围绕 (b_w, b_a)。q_k_lin、grav 未使用可省略。
  void InitBias(const Vector3& b_w, const Vector3& b_a);

  // 喂入一对 IMU 测量并更新预积分：(t_0,w_m_0,a_m_0) -> (t_1,w_m_1,a_m_1)。
  // imu_avg=false 时 (w_m_1,a_m_1) 可传零。
  void FeedImu(double t_0, double t_1, const Vector3& w_m_0,
               const Vector3& a_m_0, const Vector3& w_m_1 = Vector3::Zero(),
               const Vector3& a_m_1 = Vector3::Zero());

  bool imu_avg() const { return imu_avg_; }

  double DT = 0;                               // 已积分时长 (s)。
  Vector3 alpha_tau = Vector3::Zero();         // 位移增量（起始体坐标下）。
  Vector3 beta_tau = Vector3::Zero();          // 速度增量（起始体坐标下）。
  Vector4 q_k2tau;                             // 相对四元数 (w,x,y,z)。
  Matrix3 R_k2tau = Matrix3::Identity();       // 相对旋转 R_{k->τ}。
  Matrix3 J_q = Matrix3::Zero();               // 对陀螺 bias 的雅可比。
  Matrix3 J_a = Matrix3::Zero();
  Matrix3 J_b = Matrix3::Zero();
  Matrix3 H_a = Matrix3::Zero();               // 对加表 bias 的雅可比。
  Matrix3 H_b = Matrix3::Zero();

  const Vector3& b_w_lin() const { return b_w_lin_; }
  const Vector3& b_a_lin() const { return b_a_lin_; }
  const Matrix12& Q_c() const { return Q_c_; }
  const Matrix15& P_meas() const { return P_meas_; }

 private:
  bool imu_avg_ = false;
  Vector3 b_w_lin_; // 陀螺 bias
  Vector3 b_a_lin_; // 加速度计 bias
  Vector4 q_k_lin_; // 四元数 bias
  Vector3 grav_ = Vector3::Zero(); // 重力向量
  Matrix12 Q_c_; // 测量噪声协方差
  Matrix15 P_meas_ = Matrix15::Zero(); // 测量协方差
  Matrix3 eye3_ = Matrix3::Identity(); // 3x3 单位矩阵
  Vector3 e_1_, e_2_, e_3_; // 单位向量
  Matrix3 e_1x_, e_2x_, e_3x_; // 单位向量叉积矩阵
};

}  // namespace data_preprocessing

#endif  // MYVIO_DATA_PREPROCESSING_CPI_MODEL_H_
