/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * 纯旋转更新器（与 ZUPT 检测逻辑独立）：
 * ① verify：陀螺幅值、图像三路择一、可选加计均值；相邻帧且门控走「③单应」时缓存 H 供更新复用。
 * ② apply：单应得 R_meas → IMU 传播到当前帧 → log_so3 EKF（需注入 Propagator）。
 * 参考帧模式：匹配 t_ref↔当前；门控仍基于上一状态时刻↔当前（与更新几何可能不一致，见类注释）。
 */

#ifndef OV_MSCKF_UPDATER_PURE_ROTATION_H
#define OV_MSCKF_UPDATER_PURE_ROTATION_H

#include <Eigen/Eigen>
#include <memory>

#include <opencv2/core/mat.hpp>

#include "utils/sensor_data.h"

namespace ov_core {
class FeatureDatabase;
}

namespace ov_msckf {

class State;
class Propagator;

/// 供日志与调试图像 overlay 使用的最近一次 try_update 快照
struct PureRotDebugSnapshot {
  bool has_data = false; ///< 是否已写入本帧检验/更新调试量
  bool gate_passed = false;
  bool ekf_updated = false;
  int n_imu_samples = 0;
  int residual_rows = 0;
  double vel_norm = 0.0;
  double gyro_avg_rad_s = 0.0;
  double chi2 = 0.0;
  double chi2_lim = 0.0;
  /// 不作速度门控，恒为 true（仅保留 |v| 于 vel_norm）
  bool vel_ok = true;
  bool gyro_ok = false;
  bool chi2_ok = false;
  /// IMU 窗口内比力（加计，已 Dm 与 bias 校正）分量均值 (m/s²)；检验阶段无加计残差，仅诊断
  double accel_mean_x = 0.0;
  double accel_mean_y = 0.0;
  double accel_mean_z = 0.0;
  /// |mean(a)|，用于与 |g| 比较（纯转无平移时均值模长应接近重力）
  double accel_mean_norm = 0.0;
  bool accel_mean_ok = false;
  /// 绕光轴：图像上光流与径向垂直度均值 [0,1]；-1 无样本
  double flow_perp_mean = -1.0;
  /// r×f 符号与多数方向一致的比例 [0,1]；-1 未算
  double flow_sign_consistency = -1.0;
  /// 各点位移 f 归一化后与平均方向对齐程度 [0,1]；-1 未算
  double flow_parallel_align = -1.0;
  int n_flow_feats = 0;
  bool img_ok = false;
  /// 图像门：0=未通过/未启用；通过时恒为 3（单应 H RANSAC）
  int img_gate_branch = 0;
  /// 第三路：单应 H RANSAC（归一化坐标）
  int h_matrix_pairs = 0;
  int h_matrix_inliers = 0;
  double h_matrix_inlier_ratio = 0.0;
  /// 门③：RANSAC 已得非空单应 H（诊断用；**不**要求 f_gate_ok 或总门通过）
  bool verify_h_ransac_has_matrix = false;
  /// 检验·门③：decomposeHomographyMat 成功则为 true，并填 R/t 等
  bool verify_h_decomposition_ok = false;
  /// 所选解的 R(3×3) 行优先，与 OpenCV rots[best_i] 一致
  double verify_h_r_00 = 0.0;
  double verify_h_r_01 = 0.0;
  double verify_h_r_02 = 0.0;
  double verify_h_r_10 = 0.0;
  double verify_h_r_11 = 0.0;
  double verify_h_r_12 = 0.0;
  double verify_h_r_20 = 0.0;
  double verify_h_r_21 = 0.0;
  double verify_h_r_22 = 0.0;
  /// 所选解的平移向量模长 |t|（归一化平面模型下，非米制物理位移）
  double verify_h_decomp_t_norm = 0.0;
  /// decomposeHomographyMat 所选解的平移 t（与 OpenCV tvecs 一致，归一化模型）
  double verify_h_t_x = 0.0;
  double verify_h_t_y = 0.0;
  double verify_h_t_z = 0.0;
  /// 由 H 得到的相机系相对旋转，∠R = ||log(R)||（度）
  double verify_h_rot_angle_deg = 0.0;
  /// 门③：内点上的 H 重投影误差均值（归一化坐标），与 apply 中算法一致；-1 未算
  double verify_h_reproj_mean = -1.0;
  /// 门③：由 R 分解得到的相机系 RPY(deg，rot2rpy)
  double verify_h_cam_rpy_roll_deg = 0.0;
  double verify_h_cam_rpy_pitch_deg = 0.0;
  double verify_h_cam_rpy_yaw_deg = 0.0;
  /// 门③：R 在 IMU 系等价 RPY(deg)；无 IMU↔CAM 标定时为 false 且不填
  bool verify_h_imu_rpy_valid = false;
  double verify_h_imu_rpy_roll_deg = 0.0;
  double verify_h_imu_rpy_pitch_deg = 0.0;
  double verify_h_imu_rpy_yaw_deg = 0.0;
  /// 保留字段（不再参与门控，恒为 0 / true）
  double omega_cam_z_ratio = 0.0;
  bool imu_axis_ok = true;
  /// 是否并入视觉相对旋转残差（相对非纯旋转参考帧）
  bool visual_rotation_used = false;
  int visual_n_matches = 0;
  double visual_chi2 = 0.0;
  double visual_res_norm = 0.0;
  /// 本帧是否已成功得到单应几何并进入 apply 的 H→传播→EKF 流程（早退失败前为 false）
  bool preint_homography_update = false;
  /// 单应 H 路径：RANSAC 内点数、由 H 分解得到的相机相对旋转角 (deg)、分解平移模长、H 重投影误差均值（归一化坐标）
  int preint_h_inliers = 0;
  double preint_h_rot_angle_deg = 0.0;
  double preint_h_decomp_t_norm = 0.0;
  double preint_h_reproj_mean = 0.0;
  /// 更新段 H：相机/IMU RPY(deg)；JPL 四元数仅写入快照供外部工具，终端不再打印
  double preint_h_rpy_roll_deg = 0.0;
  double preint_h_rpy_pitch_deg = 0.0;
  double preint_h_rpy_yaw_deg = 0.0;
  double preint_h_imu_rpy_roll_deg = 0.0;
  double preint_h_imu_rpy_pitch_deg = 0.0;
  double preint_h_imu_rpy_yaw_deg = 0.0;
  double preint_h_quat_jpl_0 = 0.0;
  double preint_h_quat_jpl_1 = 0.0;
  double preint_h_quat_jpl_2 = 0.0;
  double preint_h_quat_jpl_3 = 0.0;
};

/**
 * @brief 原地旋转场景下的姿态辅助更新（与 ZUPT 独立）
 *
 * 图像门控：归一化点对单应 H（RANSAC）；加计可选门控；总门 = 陀螺幅值 ∧ H 门 ∧ 加计（若开启）。
 * 检验与更新分离：verify 仅门控；apply 统一用单应分解 R_meas + 传播 + log_so3。
 *
 * @note 若开启参考帧（use_visual_ref_rotation 且参考有效），apply 在 t_ref↔当前 上估计 H，
 *       但 verify 的图像门仍用「上一状态时刻↔当前」——二者时间基不同，门控为启发式。
 */
class UpdaterPureRotation {

public:
  UpdaterPureRotation(
      std::shared_ptr<ov_core::FeatureDatabase> db, double gravity_mag,
      double gyro_mag_min, double gyro_mag_max,
      bool print_pure_rot, bool print_state_calib,
      bool use_image_gate, double flow_perp_min, double flow_sign_consistency_min,
      int min_flow_feats, double min_r_px, double min_flow_px,
      size_t ref_cam_id,
      bool use_accel_mean_gate, double accel_mean_g_tol,
      double accel_perp_g_max,
      bool img_branch_parallel, bool img_branch_z_rot, bool img_branch_f,
      double flow_parallel_align_min, int f_min_pairs, int f_min_inliers,
      double f_min_inlier_ratio, double f_ransac_thresh_norm,
      bool use_visual_ref_rotation, int visual_min_matches,
      int visual_min_inliers, double visual_sigma_rad,
      double visual_noise_multiplier, double visual_chi2_multiplier,
      double visual_max_ref_age_sec, double visual_ransac_thresh_norm,
      double h_verify_agree_max_reproj, double h_verify_strict_max_reproj,
      double h_verify_agree_max_t_norm, double h_verify_strict_max_t_norm,
      bool lock_vel_pos, double zero_vel_sigma, double vel_cov_diag_min,
      double anchor_pos_sigma, std::shared_ptr<Propagator> propagator);

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
  /// 陀螺幅值、单应 H 门、可选加计均值；相邻帧且 H 门通过时写入 _adjacent_gate_* 供 apply 复用
  bool verify_pure_rot_gates(std::shared_ptr<State> state, double timestamp,
                             const std::vector<ov_core::ImuData> &imu_recent);
  /// 单应 R_meas、重投影调试、传播、log_so3 χ²/EKF；需非空 Propagator
  bool apply_pure_rot_homography_update(
      std::shared_ptr<State> state, double timestamp,
      const std::vector<ov_core::ImuData> &imu_recent,
      double offset_used_for_time0);

  /// 段内伪观测：v→0、p→本段首帧锚点（在 verify 与单应更新之后调用）
  void apply_pure_rot_zero_vel_pos_anchor(std::shared_ptr<State> state);

  std::shared_ptr<ov_core::FeatureDatabase> _db;

  Eigen::Vector3d _gravity;

  double _gyro_mag_min = 0.05;
  double _gyro_mag_max = 3.0;
  bool _print_pure_rot = false;
  bool _print_state_calib = false;

  bool _use_image_gate = true;
  double _flow_perp_min = 0.55;
  double _flow_sign_consistency_min = 0.75;
  int _min_flow_feats = 12;
  double _min_r_px = 25.0;
  double _min_flow_px = 0.4;
  size_t _ref_cam_id = 0;

  /// 窗口内 mean(a) 的模长与 |g| 之差上限 (m/s²)；关闭 gate 时仍打印 mean(a)
  bool _use_accel_mean_gate = false;
  double _accel_mean_g_tol = 1.0;
  /// 世界系 |a_W⊥g| 上限 (m/s²)；与 _accel_mean_g_tol 为 OR（任一满足则加计均值门通过）
  double _accel_perp_g_max = 0.25;
  /// 加计门控诊断：|g| 加权 = (1/3)×上周期 + (2/3)×本帧 |mean(a)|，首帧上周期取 |g_标称|
  double _pure_rot_g_abs_blend = 0.0;
  bool _pure_rot_have_g_abs_blend = false;

  /// 保留配置项（光流三路已不在门控中使用；图像门仅 H RANSAC）
  bool _img_branch_parallel = true;
  bool _img_branch_z_rot = true;
  bool _img_branch_f = true;
  double _flow_parallel_align_min = 0.85;
  int _f_min_pairs = 15;
  int _f_min_inliers = 8;
  double _f_min_inlier_ratio = 0.30;
  /// 第三路单应 H：findHomography(RANSAC) 最大重投影误差（与归一化坐标一致）
  double _f_ransac_thresh_norm = 0.002;

  bool _use_visual_ref_rotation = false;
  int _visual_min_matches = 12;
  int _visual_min_inliers = 8;
  double _visual_sigma_rad = 0.02;
  double _visual_noise_multiplier = 1.0;
  double _visual_chi2_multiplier = 2.0;
  double _visual_max_ref_age_sec = 30.0;
  double _visual_ransac_thresh_norm = 0.01;

  /// 门③ 检验打印「赞同/否决」支路（YAML pure_rot_h_verify_*；仅诊断；重投影/|t| 为归一化平面绝对阈）
  double _h_verify_agree_max_reproj = 0.008;
  double _h_verify_strict_max_reproj = 0.003;
  double _h_verify_agree_max_t_norm = 0.1;
  double _h_verify_strict_max_t_norm = 0.04;

  bool _ref_valid = false;
  double _ref_cam_time = -1.0;
  Eigen::Matrix<double, 4, 1> _q_GtoI_ref =
      Eigen::Matrix<double, 4, 1>::Zero();

  std::vector<ov_core::ImuData> imu_data;

  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  std::shared_ptr<Propagator> _propagator;

  /// 连续总门通过段：[PureRot-H·检验]「时段」t0 钳到本段第一次通过时的 ref（state 上一相机时刻），t1 为当前帧
  bool _pure_rot_episode_active = false;
  double _pure_rot_episode_cam_t0 = -1.0;
  /// 本段首次 gate 通过时 state 中的 p_IinG，供位置软锚定
  bool _pure_rot_has_anchor_p = false;
  Eigen::Vector3d _pure_rot_anchor_p_in_G =
      Eigen::Vector3d::Zero();

  bool _pure_rot_lock_vel_pos = true;
  double _pure_rot_zero_vel_sigma = 0.05;
  /// 零速 EKF 后对 v 各轴方差下限 (m²/s²)
  double _pure_rot_vel_cov_diag_min = 0.01;
  double _pure_rot_anchor_pos_sigma = 0.15;

  /// 检验阶段若图像门选中「③单应」，缓存相邻帧 (t_imu_prev ↔ t_cur) 的 H 与内点，供 apply 跳过第二次 findHomography
  bool _adjacent_gate_h_valid = false;
  double _adjacent_gate_t0 = 0.0;
  double _adjacent_gate_t1 = 0.0;
  cv::Mat _adjacent_gate_H;
  cv::Mat _adjacent_gate_mask;
  std::vector<Eigen::Vector2d> _adjacent_gate_pref;
  std::vector<Eigen::Vector2d> _adjacent_gate_pcur;

  PureRotDebugSnapshot _last_debug;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_PURE_ROTATION_H
