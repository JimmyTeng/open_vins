/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * 纯旋转更新器（与 ZUPT 检测逻辑独立）：在 ZUPT 未通过静止门时，
 * 若 IMU 表明存在明显角速度且线速度较小，则仅用「加计对齐重力 + 线速度≈0」更新，
 * 不施加陀螺零速残差，避免与原地旋转矛盾。
 */

#ifndef OV_MSCKF_UPDATER_PURE_ROTATION_H
#define OV_MSCKF_UPDATER_PURE_ROTATION_H

#include <Eigen/Eigen>
#include <memory>

#include "utils/sensor_data.h"

#include "UpdaterOptions.h"
#include "utils/NoiseManager.h"

namespace ov_core {
class FeatureDatabase;
}

namespace ov_msckf {

class State;

/// 供日志与调试图像 overlay 使用的最近一次 try_update 快照
struct PureRotDebugSnapshot {
  bool has_data = false; ///< 是否已算过加计/χ²（本路径内）
  bool gate_passed = false;
  bool ekf_updated = false;
  int n_imu_samples = 0;
  int residual_rows = 0;
  double vel_norm = 0.0;
  double gyro_avg_rad_s = 0.0;
  double chi2 = 0.0;
  double chi2_lim = 0.0;
  bool vel_ok = false;
  bool gyro_ok = false;
  bool chi2_ok = false;
  /// 绕光轴：图像上光流与径向垂直度均值 [0,1]；-1 无样本
  double flow_perp_mean = -1.0;
  /// r×f 符号与多数方向一致的比例 [0,1]；-1 未算
  double flow_sign_consistency = -1.0;
  int n_flow_feats = 0;
  bool img_ok = false;
  /// 陀螺在参考相机系下 |ωz|/|ω|（光轴分量占比）
  double omega_cam_z_ratio = 0.0;
  bool imu_axis_ok = false;
  /// 是否并入视觉相对旋转残差（相对非纯旋转参考帧）
  bool visual_rotation_used = false;
  int visual_n_matches = 0;
  double visual_chi2 = 0.0;
  double visual_res_norm = 0.0;
};

/**
 * @brief 绕相机光轴旋转辅助更新（图像：光流近似垂直于相对主点的径向；IMU：角速度在相机系以光轴分量为主）
 *
 * 检测与 ZUPT 完全独立。门限：可选图像 perp 置信、可选 ω 光轴占比、|v|、陀螺幅值、χ²。
 * 测量模型：加计对齐重力 + v≈0，无陀螺零速残差。
 * 可选：相对「最近一次非纯旋转帧」保存的 IMU 姿态，用特征匹配估计的相机相对旋转
 * 与当前状态预测之差作 log-so3 残差，约束绕视轴分量（下视场景即航向）。
 */
class UpdaterPureRotation {

public:
  UpdaterPureRotation(
      UpdaterOptions &options, NoiseManager &noises,
      std::shared_ptr<ov_core::FeatureDatabase> db, double gravity_mag,
      double max_velocity, double gyro_mag_min, double gyro_mag_max,
      double vel_sigma, double noise_multiplier, bool print_pure_rot,
      bool use_image_gate, double flow_perp_min, double flow_sign_consistency_min,
      int min_flow_feats, double min_r_px, double min_flow_px,
      bool use_cam_z_gyro_gate,
      double min_omega_cam_z_ratio, size_t ref_cam_id,
      bool use_visual_ref_rotation, int visual_min_matches,
      int visual_min_inliers, double visual_sigma_rad,
      double visual_noise_multiplier, double visual_chi2_multiplier,
      double visual_max_ref_age_sec, double visual_ransac_thresh_norm);

  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1);

  void clean_old_imu_measurements(double oldest_time);

  /**
   * @brief 在普通 VIO / ZUPT 等「非本帧 PureRot 成功」路径上调用，保存参考相机时刻与 IMU 姿态
   */
  void record_reference(std::shared_ptr<State> state, double cam_timestamp);

  /**
   * @return 若本帧执行了纯旋转更新则为 true（并将 state 时间推进到 timestamp）
   */
  bool try_update(std::shared_ptr<State> state, double timestamp);

  PureRotDebugSnapshot get_last_debug_snapshot() const { return _last_debug; }

protected:
  UpdaterOptions _options;
  NoiseManager _noises;
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  Eigen::Vector3d _gravity;

  double _max_velocity = 0.2;
  double _gyro_mag_min = 0.05;
  double _gyro_mag_max = 3.0;
  double _vel_sigma = 0.1;
  double _noise_multiplier = 1.0;
  bool _print_pure_rot = false;

  bool _use_image_gate = true;
  double _flow_perp_min = 0.55;
  double _flow_sign_consistency_min = 0.75;
  int _min_flow_feats = 12;
  double _min_r_px = 25.0;
  double _min_flow_px = 0.4;
  bool _use_cam_z_gyro_gate = true;
  double _min_omega_cam_z_ratio = 0.65;
  size_t _ref_cam_id = 0;

  bool _use_visual_ref_rotation = false;
  int _visual_min_matches = 12;
  int _visual_min_inliers = 8;
  double _visual_sigma_rad = 0.02;
  double _visual_noise_multiplier = 1.0;
  double _visual_chi2_multiplier = 2.0;
  double _visual_max_ref_age_sec = 30.0;
  double _visual_ransac_thresh_norm = 0.01;

  bool _ref_valid = false;
  double _ref_cam_time = -1.0;
  Eigen::Matrix<double, 4, 1> _q_GtoI_ref =
      Eigen::Matrix<double, 4, 1>::Zero();

  std::vector<ov_core::ImuData> imu_data;

  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  PureRotDebugSnapshot _last_debug;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_PURE_ROTATION_H
