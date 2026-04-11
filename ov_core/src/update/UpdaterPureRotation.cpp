/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * PureRot：verify（门控，可选缓存相邻帧单应）→ apply（单应 R_meas、IMU 传播、log_so3 EKF）。
 */

#include "UpdaterPureRotation.h"

#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/chi_squared_quantile.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

namespace {

/**
 * 对已知 3×3 单应 H（归一化平面、K=I）做 decomposeHomographyMat，选 det(R)>0 且平移范数最小的一解。
 * @param t_vec_out 可选，写入与 R 配对的平移 3 向量（OpenCV tvecs[best_i]）
 */
bool decompose_homography_mat_to_R(const cv::Mat &H_cv, Eigen::Matrix3d &R_meas,
                                   double *decomp_t_norm_out,
                                   Eigen::Vector3d *t_vec_out = nullptr) {
  if (H_cv.empty() || H_cv.rows != 3 || H_cv.cols != 3)
    return false;
  cv::Mat Keye = cv::Mat::eye(3, 3, CV_64F);
  std::vector<cv::Mat> rots, tvecs, normals;
  const int nsol =
      cv::decomposeHomographyMat(H_cv, Keye, rots, tvecs, normals);
  if (nsol < 1 || (int)rots.size() < 1)
    return false;
  int best_i = -1;
  double best_tn = 1e9;
  for (int i = 0; i < nsol && i < (int)rots.size(); i++) {
    if (rots[i].empty() || cv::determinant(rots[i]) < 0.5)
      continue;
    const double tn = cv::norm(tvecs[i], cv::NORM_L2);
    if (tn < best_tn) {
      best_tn = tn;
      best_i = i;
    }
  }
  if (best_i < 0) {
    best_i = 0;
    best_tn = tvecs.empty() ? 0.0 : cv::norm(tvecs[0], cv::NORM_L2);
  }
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      R_meas(r, c) = rots[best_i].at<double>(r, c);
  if (R_meas.determinant() <= 0.5)
    return false;
  if (decomp_t_norm_out != nullptr)
    *decomp_t_norm_out = best_tn;
  if (t_vec_out != nullptr && best_i >= 0 && best_i < (int)tvecs.size() &&
      !tvecs[best_i].empty()) {
    const cv::Mat &tv = tvecs[best_i];
    (*t_vec_out)(0) = tv.at<double>(0, 0);
    (*t_vec_out)(1) = tv.rows > 1 ? tv.at<double>(1, 0) : 0.0;
    (*t_vec_out)(2) = tv.rows > 2 ? tv.at<double>(2, 0) : 0.0;
  }
  return true;
}

/**
 * 归一化点对 → findHomography(RANSAC) → 分解得相对旋转 R_meas。
 * @param decomp_t_norm_out 分解得到的平移模长（辅助诊断，纯旋转下应较小）
 */
bool pure_rot_homography_estimate_R(
    const cv::Mat &pts1, const cv::Mat &pts2, double ransac_reproj_thresh,
    int min_inliers, Eigen::Matrix3d &R_meas, int &n_inliers,
    double *decomp_t_norm_out, cv::Mat *H_out, cv::Mat *mask_out) {
  cv::Mat mask;
  cv::Mat H_cv = cv::findHomography(pts1, pts2, cv::RANSAC,
                                     ransac_reproj_thresh, mask, 2000, 0.999);
  if (H_cv.empty() || H_cv.rows != 3 || H_cv.cols != 3)
    return false;
  n_inliers = mask.empty() ? pts1.rows : cv::countNonZero(mask);
  if (n_inliers < min_inliers)
    return false;
  if (H_out != nullptr)
    *H_out = H_cv;
  if (mask_out != nullptr)
    *mask_out = mask;
  return decompose_homography_mat_to_R(H_cv, R_meas, decomp_t_norm_out);
}

} // namespace

UpdaterPureRotation::UpdaterPureRotation(
    std::shared_ptr<ov_core::FeatureDatabase> db, double gravity_mag,
    double gyro_mag_min, double gyro_mag_max,
    bool print_pure_rot, bool print_state_calib,
    bool use_image_gate, double flow_perp_min,
    double flow_sign_consistency_min, int min_flow_feats, double min_r_px,
    double min_flow_px, size_t ref_cam_id,
    bool use_accel_mean_gate, double accel_mean_g_tol, double accel_perp_g_max,
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
    double anchor_pos_sigma, std::shared_ptr<Propagator> propagator)
    : _db(std::move(db)),
      _gyro_mag_min(gyro_mag_min),
      _gyro_mag_max(gyro_mag_max), _print_pure_rot(print_pure_rot),
      _print_state_calib(print_state_calib),
      _use_image_gate(use_image_gate), _flow_perp_min(flow_perp_min),
      _flow_sign_consistency_min(flow_sign_consistency_min),
      _min_flow_feats(min_flow_feats), _min_r_px(min_r_px),
      _min_flow_px(min_flow_px), _ref_cam_id(ref_cam_id),
      _use_accel_mean_gate(use_accel_mean_gate),
      _accel_mean_g_tol(std::max(0.0, accel_mean_g_tol)),
      _accel_perp_g_max(std::max(0.0, accel_perp_g_max)),
      _img_branch_parallel(img_branch_parallel),
      _img_branch_z_rot(img_branch_z_rot), _img_branch_f(img_branch_f),
      _flow_parallel_align_min(
          std::min(1.0, std::max(0.0, flow_parallel_align_min))),
      _f_min_pairs(std::max(8, f_min_pairs)),
      _f_min_inliers(std::max(6, f_min_inliers)),
      _f_min_inlier_ratio(std::min(1.0, std::max(0.0, f_min_inlier_ratio))),
      _f_ransac_thresh_norm(std::max(1e-6, f_ransac_thresh_norm)),
      _use_visual_ref_rotation(use_visual_ref_rotation),
      _visual_min_matches(std::max(8, visual_min_matches)),
      _visual_min_inliers(std::max(6, visual_min_inliers)),
      _visual_sigma_rad(std::max(1e-6, visual_sigma_rad)),
      _visual_noise_multiplier(
          std::max(1e-6, visual_noise_multiplier)),
      _visual_chi2_multiplier(std::max(1e-6, visual_chi2_multiplier)),
      _visual_max_ref_age_sec(
          std::max(0.1, visual_max_ref_age_sec)),
      _visual_ransac_thresh_norm(
          std::max(1e-6, visual_ransac_thresh_norm)),
      _h_verify_agree_max_reproj(std::max(1e-12, h_verify_agree_max_reproj)),
      _h_verify_strict_max_reproj(std::max(1e-12, h_verify_strict_max_reproj)),
      _h_verify_agree_max_t_norm(std::max(1e-12, h_verify_agree_max_t_norm)),
      _h_verify_strict_max_t_norm(std::max(1e-12, h_verify_strict_max_t_norm)),
      _pure_rot_lock_vel_pos(lock_vel_pos),
      _pure_rot_zero_vel_sigma(std::max(1e-6, zero_vel_sigma)),
      _pure_rot_vel_cov_diag_min(std::max(0.0, vel_cov_diag_min)),
      _pure_rot_anchor_pos_sigma(std::max(1e-6, anchor_pos_sigma)),
      _propagator(std::move(propagator)) {
  _gravity << 0.0, 0.0, gravity_mag;
}

void UpdaterPureRotation::record_reference(std::shared_ptr<State> state,
                                           double cam_timestamp) {
  if (state == nullptr)
    return;
  _ref_valid = true;
  _ref_cam_time = cam_timestamp;
  _q_GtoI_ref = state->_imu->quat();
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
  // CAM–IMU 时间偏移与 Propagator 对齐；IMU 段与 verify/apply 共用
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
  const double offset_used_for_time0 = last_prop_time_offset;
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


  if (!verify_pure_rot_gates(state, timestamp, imu_recent))
    return false;
  if (!_propagator) {
    _pure_rot_episode_active = false;
    if (_print_pure_rot) {
      PRINT_ERROR(RED "[PureRot]: 未注入 Propagator，无法进行 IMU 传播与单应更新\n" RESET);
    }
    return false;
  }
  if (!apply_pure_rot_homography_update(state, timestamp, imu_recent,
                                        offset_used_for_time0)) {
    _pure_rot_episode_active = false;
    return false;
  }
  return true;
}

bool UpdaterPureRotation::verify_pure_rot_gates(
    std::shared_ptr<State> state, double timestamp,
    const std::vector<ov_core::ImuData> &imu_recent) {

  _adjacent_gate_h_valid = false;

  Eigen::Matrix3d Dw =
      State::Dm(state->_options.imu_model, state->_calib_imu_dw->value());
  Eigen::Matrix3d Da =
      State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
  Eigen::Matrix3d Tg = State::Tg(state->_calib_imu_tg->value());

  const int nseg = (int)imu_recent.size() - 1;
  double gyro_mag_sum = 0.0;
  Eigen::Vector3d a_sum = Eigen::Vector3d::Zero();

  for (int i = 0; i < nseg; i++) {
    Eigen::Vector3d a_hat = state->_calib_imu_ACCtoIMU->Rot() * Da *
                            (imu_recent.at(i).am - state->_imu->bias_a());
    Eigen::Vector3d w_hat =
        state->_calib_imu_GYROtoIMU->Rot() * Dw *
        (imu_recent.at(i).wm - state->_imu->bias_g() - Tg * a_hat);
    gyro_mag_sum += w_hat.norm();
    a_sum += a_hat;
  }

  const double gyro_avg = gyro_mag_sum / (double)imu_recent.size();
  const Eigen::Vector3d a_mean = a_sum / (double)std::max(1, nseg);
  const double a_mean_norm = a_mean.norm();
  const double g_mag = _gravity.norm();
  // 世界系：R_ItoG * mean(a)，去掉与标称重力 _gravity 平行分量后的模（水平面内非重力分量）
  const Eigen::Matrix3d R_GtoI_a = state->_imu->Rot();
  const Eigen::Vector3d a_mean_in_G = R_GtoI_a.transpose() * a_mean;
  const Eigen::Vector3d g_hat =
      (g_mag > 1e-12) ? (_gravity / g_mag) : Eigen::Vector3d(0.0, 0.0, 1.0);
  const double a_mean_perp_g_norm =
      (a_mean_in_G - a_mean_in_G.dot(g_hat) * g_hat).norm();

  bool f_gate_ok = false;
  int f_pairs = 0;
  int f_inl = 0;
  double f_ratio = 0.0;
  /// RANSAC 得到合法 3×3 H 即保留（用于诊断打印/snapshot；apply 缓存仍要求 f_gate_ok）
  cv::Mat branch3_H;
  cv::Mat branch3_mask;
  std::vector<Eigen::Vector2d> branch3_pref, branch3_pcur;
  bool branch3_ransac_has_h = false;
  /// f_gate_ok：内点/比例达门限，供门③通过与相邻帧 H 复用
  bool branch3_geom_ready = false;
  if (_use_image_gate && _db != nullptr &&
      !state->_cam_intrinsics_cameras.empty()) {
    std::vector<Eigen::Vector2d> pref, pcur;
    f_pairs = FeatureHelper::collect_normalized_correspondences(
        _db, state->_timestamp, timestamp, _ref_cam_id,
        state->_cam_intrinsics_cameras, pref, pcur);
    if (f_pairs >= _f_min_pairs) {
      cv::Mat pts1(f_pairs, 2, CV_64F);
      cv::Mat pts2(f_pairs, 2, CV_64F);
      for (int i = 0; i < f_pairs; i++) {
        pts1.at<double>(i, 0) = pref[i](0);
        pts1.at<double>(i, 1) = pref[i](1);
        pts2.at<double>(i, 0) = pcur[i](0);
        pts2.at<double>(i, 1) = pcur[i](1);
      }
      cv::Mat mask;
      cv::Mat H_cv = cv::findHomography(pts1, pts2, cv::RANSAC,
                                         _f_ransac_thresh_norm, mask, 2000,
                                         0.999);
      if (!H_cv.empty() && H_cv.rows == 3 && H_cv.cols == 3 && !mask.empty()) {
        f_inl = cv::countNonZero(mask);
        f_ratio = (f_pairs > 0) ? ((double)f_inl / (double)f_pairs) : 0.0;
        f_gate_ok =
            (f_inl >= _f_min_inliers) && (f_ratio >= _f_min_inlier_ratio);
        branch3_ransac_has_h = true;
        branch3_H = H_cv.clone();
        branch3_mask = mask.clone();
        branch3_pref = std::move(pref);
        branch3_pcur = std::move(pcur);
        if (f_gate_ok)
          branch3_geom_ready = true;
      }
    }
  }

  const bool gyro_ok =
      (gyro_avg >= _gyro_mag_min) && (gyro_avg <= _gyro_mag_max);

  // |g| 加权：上周期占 1/3、本帧 |mean(a)| 占 2/3；首帧「上周期」用标称 |g|
  if (!_pure_rot_have_g_abs_blend) {
    _pure_rot_g_abs_blend = g_mag;
    _pure_rot_have_g_abs_blend = true;
  }
  _pure_rot_g_abs_blend =
      (1.0 / 3.0) * _pure_rot_g_abs_blend + (2.0 / 3.0) * a_mean_norm;
  const double g_weighted_abs = _pure_rot_g_abs_blend;

  _last_debug = PureRotDebugSnapshot{};
  _last_debug.has_data = true;
  _last_debug.ekf_updated = false;
  _last_debug.preint_homography_update = false;
  _last_debug.n_imu_samples = (int)imu_recent.size();
  _last_debug.residual_rows = 0;
  _last_debug.vel_norm = state->_imu->vel().norm();
  _last_debug.gyro_avg_rad_s = gyro_avg;
  _last_debug.chi2 = 0.0;
  _last_debug.chi2_lim = 0.0;
  _last_debug.vel_ok = true;
  _last_debug.gyro_ok = gyro_ok;
  _last_debug.chi2_ok = true;
  _last_debug.accel_mean_x = a_mean(0);
  _last_debug.accel_mean_y = a_mean(1);
  _last_debug.accel_mean_z = a_mean(2);
  _last_debug.accel_mean_norm = a_mean_norm;
  _last_debug.flow_perp_mean = -1.0;
  _last_debug.flow_sign_consistency = -1.0;
  _last_debug.flow_parallel_align = -1.0;
  _last_debug.n_flow_feats = 0;
  _last_debug.h_matrix_pairs = f_pairs;
  _last_debug.h_matrix_inliers = f_inl;
  _last_debug.h_matrix_inlier_ratio = f_ratio;
  _last_debug.visual_rotation_used = false;
  _last_debug.visual_n_matches = 0;

  _last_debug.verify_h_ransac_has_matrix = branch3_ransac_has_h;
  _last_debug.verify_h_decomposition_ok = false;
  _last_debug.verify_h_reproj_mean = -1.0;
  _last_debug.verify_h_t_x = 0.0;
  _last_debug.verify_h_t_y = 0.0;
  _last_debug.verify_h_t_z = 0.0;
  if (branch3_ransac_has_h && !branch3_H.empty()) {
    Eigen::Matrix3d R_verify;
    Eigen::Vector3d t_verify;
    double tn_decomp = 0.0;
    if (decompose_homography_mat_to_R(branch3_H, R_verify, &tn_decomp,
                                      &t_verify)) {
      _last_debug.verify_h_decomposition_ok = true;
      _last_debug.verify_h_decomp_t_norm = tn_decomp;
      _last_debug.verify_h_t_x = t_verify(0);
      _last_debug.verify_h_t_y = t_verify(1);
      _last_debug.verify_h_t_z = t_verify(2);
      _last_debug.verify_h_rot_angle_deg =
          log_so3(R_verify).norm() * 180.0 / M_PI;
      _last_debug.verify_h_r_00 = R_verify(0, 0);
      _last_debug.verify_h_r_01 = R_verify(0, 1);
      _last_debug.verify_h_r_02 = R_verify(0, 2);
      _last_debug.verify_h_r_10 = R_verify(1, 0);
      _last_debug.verify_h_r_11 = R_verify(1, 1);
      _last_debug.verify_h_r_12 = R_verify(1, 2);
      _last_debug.verify_h_r_20 = R_verify(2, 0);
      _last_debug.verify_h_r_21 = R_verify(2, 1);
      _last_debug.verify_h_r_22 = R_verify(2, 2);
      constexpr double rad2deg_v = 180.0 / M_PI;
      const Eigen::Matrix<double, 3, 1> rpy_cam_v = rot2rpy(R_verify);
      _last_debug.verify_h_cam_rpy_roll_deg = rpy_cam_v(0) * rad2deg_v;
      _last_debug.verify_h_cam_rpy_pitch_deg = rpy_cam_v(1) * rad2deg_v;
      _last_debug.verify_h_cam_rpy_yaw_deg = rpy_cam_v(2) * rad2deg_v;
      _last_debug.verify_h_imu_rpy_valid = false;
      if (state->_calib_IMUtoCAM.count(_ref_cam_id) > 0) {
        const Eigen::Matrix3d R_ItoC_v =
            state->_calib_IMUtoCAM.at(_ref_cam_id)->Rot();
        const Eigen::Matrix3d R_CtoI_v = R_ItoC_v.transpose();
        const Eigen::Matrix3d R_in_imu_v = R_CtoI_v * R_verify * R_ItoC_v;
        const Eigen::Matrix<double, 3, 1> rpy_imu_v = rot2rpy(R_in_imu_v);
        _last_debug.verify_h_imu_rpy_roll_deg = rpy_imu_v(0) * rad2deg_v;
        _last_debug.verify_h_imu_rpy_pitch_deg = rpy_imu_v(1) * rad2deg_v;
        _last_debug.verify_h_imu_rpy_yaw_deg = rpy_imu_v(2) * rad2deg_v;
        _last_debug.verify_h_imu_rpy_valid = true;
      }
    }
  }
  // 与 apply 相同：仅统计 RANSAC 内点上的重投影（归一化坐标）
  if (branch3_ransac_has_h && !branch3_H.empty() &&
      !branch3_pref.empty() &&
      (int)branch3_pref.size() == (int)branch3_pcur.size()) {
    Eigen::Matrix3d He;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        He(r, c) = branch3_H.at<double>(r, c);
    double reproj_sum = 0.0;
    int reproj_cnt = 0;
    const int npt = (int)branch3_pref.size();
    for (int i = 0; i < npt; i++) {
      if (!branch3_mask.empty() && !branch3_mask.at<unsigned char>(i))
        continue;
      Eigen::Vector3d x1(branch3_pref[i](0), branch3_pref[i](1), 1.0);
      Eigen::Vector3d xh = He * x1;
      if (std::abs(xh(2)) < 1e-12)
        continue;
      xh /= xh(2);
      const double ex = xh(0) - branch3_pcur[i](0);
      const double ey = xh(1) - branch3_pcur[i](1);
      reproj_sum += std::sqrt(ex * ex + ey * ey);
      reproj_cnt++;
    }
    if (reproj_cnt > 0)
      _last_debug.verify_h_reproj_mean = reproj_sum / (double)reproj_cnt;
  }

  /// 与下方 [PureRot-H·检验]「赞同||否决」合并一致：RANSAC 通过且（重投影+|t|）至少一路满足
  bool gate_agree_geom = false;
  bool h_merge_geom = false;
  if (branch3_ransac_has_h && f_gate_ok) {
    const double reproj_p = _last_debug.verify_h_reproj_mean;
    const bool decomp_ok = _last_debug.verify_h_decomposition_ok;
    const double thr_reproj_a = _h_verify_agree_max_reproj;
    const double thr_reproj_s = _h_verify_strict_max_reproj;
    const double thr_t_a = _h_verify_agree_max_t_norm;
    const double thr_t_s = _h_verify_strict_max_t_norm;
    const double tnorm = decomp_ok ? _last_debug.verify_h_decomp_t_norm : -1.0;

    const bool reproj_ok_agree =
        (reproj_p >= 0.0) && (reproj_p <= thr_reproj_a);
    const bool reproj_ok_strict =
        (reproj_p >= 0.0) && (reproj_p <= thr_reproj_s);

    const bool t_ok_agree = decomp_ok && (tnorm <= thr_t_a);
    const bool t_ok_strict = decomp_ok && (tnorm <= thr_t_s);

    gate_agree_geom = f_gate_ok && reproj_ok_agree && t_ok_agree;
    const bool gate_strict_geom =
        f_gate_ok && reproj_ok_strict && t_ok_strict;
    h_merge_geom = gate_agree_geom || gate_strict_geom;
  }

  // 加计均值子判据1：赞同支路通过时 ||mean|-|g|| 阈值为配置的 2 倍（与门③几何一致后再判定）
  bool accel_mean_ok = true;
  if (_use_accel_mean_gate) {
    const double tol_mean_g =
        gate_agree_geom ? (2.0 * _accel_mean_g_tol) : _accel_mean_g_tol;
    const bool tol_ok = std::abs(a_mean_norm - g_mag) <= tol_mean_g;
    const bool perp_ok = a_mean_perp_g_norm <= _accel_perp_g_max;
    accel_mean_ok = tol_ok || perp_ok;
  }
  _last_debug.accel_mean_ok = accel_mean_ok;

  int img_gate_branch = 0;
  bool img_ok = true;
  if (_use_image_gate) {
    if (!branch3_ransac_has_h || !f_gate_ok)
      img_ok = false;
    else
      img_ok = h_merge_geom;
    img_gate_branch = img_ok ? 3 : 0;
  }

  const bool gate_passed = gyro_ok && img_ok && accel_mean_ok;

  if (gate_passed) {
    if (!_pure_rot_episode_active) {
      _pure_rot_episode_cam_t0 = state->_timestamp;
      _pure_rot_anchor_p_in_G = state->_imu->pos();
      _pure_rot_has_anchor_p = true;
    }
    _pure_rot_episode_active = true;
  } else {
    _pure_rot_episode_active = false;
    _pure_rot_has_anchor_p = false;
  }
  /// 与单应 ref/cur 一致：总门未通过时仍为相邻对；通过时 t0 为本 PureRot 段首帧 ref（与「首次进入」对齐）
  const double pr_h_span_t0 =
      gate_passed ? _pure_rot_episode_cam_t0 : state->_timestamp;
  const double pr_h_span_t1 = timestamp;

  _last_debug.gate_passed = gate_passed;
  _last_debug.img_ok = img_ok;
  _last_debug.img_gate_branch = img_gate_branch;

  // 仅当总门通过且图像门选中③、且点数/内点满足更新段下限时缓存（与 apply 中 visual_* 一致）
  if (gate_passed && img_gate_branch == 3 && branch3_geom_ready &&
      (int)branch3_pref.size() >= _visual_min_matches) {
    const int nin =
        branch3_mask.empty() ? 0 : cv::countNonZero(branch3_mask);
    if (nin >= _visual_min_inliers) {
      _adjacent_gate_h_valid = true;
      _adjacent_gate_t0 = state->_timestamp;
      _adjacent_gate_t1 = timestamp;
      _adjacent_gate_H = branch3_H.clone();
      _adjacent_gate_mask = branch3_mask.clone();
      _adjacent_gate_pref = std::move(branch3_pref);
      _adjacent_gate_pcur = std::move(branch3_pcur);
    }
  }

  if (_print_pure_rot) {
    const char *ok = "通过";
    const char *no = "未通过";
    PRINT_INFO(CYAN "[PureRot] 检验: %s\n" RESET, gate_passed ? ok : no);
    if (_use_image_gate && _use_accel_mean_gate) {
      PRINT_INFO("  ├─ [判定条件] 总判定=(陀螺幅值&&H检验&&加计均值)\n");
    } else if (_use_image_gate) {
      PRINT_INFO("  ├─ [判定条件] 总判定=(陀螺幅值&&H检验)\n");
    } else if (_use_accel_mean_gate) {
      PRINT_INFO("  ├─ [判定条件] 总判定=(陀螺幅值&&加计均值)\n");
    } else {
      PRINT_INFO("  ├─ [判定条件] 总判定=(陀螺幅值)\n");
    }
    PRINT_INFO("  ├─ [陀螺幅值] %s  avg|ω|=%.4f rad/s  区间[%.4f, %.4f]\n",
               gyro_ok ? ok : no, gyro_avg, _gyro_mag_min, _gyro_mag_max);
    if (_use_accel_mean_gate) {
      const size_t n_imu_win = imu_recent.size();
      const double win_dt_s =
          (n_imu_win >= 2)
              ? (imu_recent.back().timestamp - imu_recent.front().timestamp)
              : 0.0;
      const double tol_mean_g_eff =
          gate_agree_geom ? (2.0 * _accel_mean_g_tol) : _accel_mean_g_tol;
      const bool tol_sub =
          std::abs(a_mean_norm - g_mag) <= tol_mean_g_eff;
      const bool perp_sub = a_mean_perp_g_norm <= _accel_perp_g_max;
      PRINT_INFO(
          "  ├─ [加计均值] %s  窗口 IMU=%zu点 参与均值累加=%d个 Δt=%.4fs "
          "mean(a)=[%.4f,%.4f,%.4f] |mean|=%.4f |g|加权=%.4f(|g|标称=%.4f,上1/3+本|mean|2/3) "
          " ||mean|-|g||=%.4f 阈%.4f%s→%s  |a_W⊥g|=%.4f 阈%.4f→%s  (OR 任一通过则本门通过)\n",
          accel_mean_ok ? ok : no, n_imu_win, nseg, win_dt_s, a_mean(0), a_mean(1),
          a_mean(2), a_mean_norm, g_weighted_abs, g_mag,
          std::abs(a_mean_norm - g_mag), tol_mean_g_eff,
          gate_agree_geom ? " [赞同支路|Δ|阈×2]" : "", tol_sub ? ok : no,
          a_mean_perp_g_norm, _accel_perp_g_max, perp_sub ? ok : no);
    } else {
      PRINT_INFO(
          "  ├─ [加计均值] 已关闭 gate  mean(a)=[%.4f,%.4f,%.4f] m/s²  "
          "|mean|=%.4f |g|加权=%.4f |g|标称=%.4f  ||mean|-|g||=%.4f  |a_W⊥g|=%.4f "
          "(若开门：阈%.4f 或 %.4f，OR)\n",
          a_mean(0), a_mean(1), a_mean(2), a_mean_norm, g_weighted_abs, g_mag,
          std::abs(a_mean_norm - g_mag), a_mean_perp_g_norm, _accel_mean_g_tol,
          _accel_perp_g_max);
    }
    if (_use_image_gate) {

      PRINT_INFO(
          "  └─ [H检验] %s  匹配=%d  内点=%d  "
          "比例=%.3f  需≥%d对 ≥%.0f%%内点 RANSAC阈=%.4f\n",
          img_ok ? ok : no, f_pairs, f_inl, f_ratio, _f_min_pairs,
          _f_min_inlier_ratio * 100.0, _f_ransac_thresh_norm);
    } else {
      PRINT_INFO("  └─ [图像] 已关闭 (pure_rot_use_image_gate=0)\n");
    }
    // 门③：相邻帧时间基；赞同/否决/合并与总门 img_ok 使用同一套几何判据
    if (_last_debug.verify_h_ransac_has_matrix) {
      const double reproj_p =
          _last_debug.verify_h_reproj_mean >= 0.0 ? _last_debug.verify_h_reproj_mean
                                                   : -1.0;
      const bool decomp_ok = _last_debug.verify_h_decomposition_ok;
      const double thr_reproj_a = _h_verify_agree_max_reproj;
      const double thr_reproj_s = _h_verify_strict_max_reproj;
      const double thr_t_a = _h_verify_agree_max_t_norm;
      const double thr_t_s = _h_verify_strict_max_t_norm;
      const bool inl_ok_agree = f_gate_ok;
      const bool inl_ok_strict = f_gate_ok;

      const bool reproj_ok_agree =
          (reproj_p >= 0.0) && (reproj_p <= thr_reproj_a);
      const bool reproj_ok_strict =
          (reproj_p >= 0.0) && (reproj_p <= thr_reproj_s);

      const double tnorm = decomp_ok ? _last_debug.verify_h_decomp_t_norm : -1.0;

      const bool t_ok_agree = decomp_ok && (tnorm <= thr_t_a);
      const bool t_ok_strict = decomp_ok && (tnorm <= thr_t_s);

      const bool gate_agree_geom =
          inl_ok_agree && reproj_ok_agree && t_ok_agree;
      const bool gate_strict_geom =
          inl_ok_strict && reproj_ok_strict && t_ok_strict;
      const bool merge_geom = gate_agree_geom || gate_strict_geom;

      const char *res_agree_geom = gate_agree_geom ? ok : no;
      const char *res_strict_geom = gate_strict_geom ? ok : no;
      const char *res_merge_geom = merge_geom ? ok : no;

      const char *inl_s = inl_ok_strict ? ok : no;
      const char *rp_s = reproj_ok_strict ? ok : no;
      const char *t_a = t_ok_agree ? ok : no;
      const char *t_s = t_ok_strict ? ok : no;

      // 树形：与上行「  ├─ [H检验]」对齐 — 「  │ 」为竖线，其后固定两格再接 ├/└
      PRINT_INFO("     ├─ 时段            t0=%.6f s → t1=%.6f s\n",
                 pr_h_span_t0, pr_h_span_t1);

      if (!decomp_ok) {
        PRINT_INFO(YELLOW "     ├─ H 分解          失败（det(R) 等），|t|/∠R_H "
                          "支路无有效值\n" RESET);
      }

      if (decomp_ok) {
        if (_last_debug.verify_h_imu_rpy_valid) {
          PRINT_INFO("     ├─ RPY (deg)       相机  r=%.4f  p=%.4f  y=%.4f\n",
                     _last_debug.verify_h_cam_rpy_roll_deg,
                     _last_debug.verify_h_cam_rpy_pitch_deg,
                     _last_debug.verify_h_cam_rpy_yaw_deg);
          PRINT_INFO(
              "     │  IMU等价           r=%.4f  p=%.4f  y=%.4f  "
              "(R_CtoI·R·R_ItoC)\n",
              _last_debug.verify_h_imu_rpy_roll_deg,
              _last_debug.verify_h_imu_rpy_pitch_deg,
              _last_debug.verify_h_imu_rpy_yaw_deg);
        } else {
          PRINT_INFO("     ├─ RPY (deg)       相机  r=%.4f  p=%.4f  y=%.4f\n",
                     _last_debug.verify_h_cam_rpy_roll_deg,
                     _last_debug.verify_h_cam_rpy_pitch_deg,
                     _last_debug.verify_h_cam_rpy_yaw_deg);
          PRINT_INFO("     │  IMU等价           n/a（无 IMU↔CAM 标定）\n");
        }
        PRINT_INFO(
            "     ├─ 分解平移 t      tx=%.8f  ty=%.8f  tz=%.8f  |t|=%.5f  "
            "（归一化模型，非米）\n",
            _last_debug.verify_h_t_x, _last_debug.verify_h_t_y,
            _last_debug.verify_h_t_z, _last_debug.verify_h_decomp_t_norm);
      }

      PRINT_INFO("     ├─ [赞同支路]      %s\n", res_agree_geom);
      if (decomp_ok) {
        PRINT_INFO("     │  |t|             %s  %.5f 门限≤%.5f "
                   "(赞同·归一化非米)\n",
                   t_a, tnorm, thr_t_a);
      } else {
        PRINT_INFO("     │  |t|             未通过  n/a\n");
      }

      PRINT_INFO("     ├─ [否决支路]      %s\n", res_strict_geom);
      PRINT_INFO(
          "     │  内点/占比         %s  需≥%d对 ≥%d内点 ≥%.0f%%(与门③ f_gate 同)  "
          "当前=%d/%d 占比=%.4f\n",
          inl_s, _f_min_pairs, _f_min_inliers, _f_min_inlier_ratio * 100.0,
          f_pairs, f_inl, f_ratio);
      if (reproj_p >= 0.0) {
        PRINT_INFO("     │  重投影           %s  mean=%.5f 门限≤%.5f "
                   "(否决·归一化绝对阈)\n",
                   rp_s, reproj_p, thr_reproj_s);
      } else {
        PRINT_INFO("     │  重投影           未通过  mean=n/a 门限≤%.5f (否决)\n",
                   thr_reproj_s);
      }
      if (decomp_ok) {
        PRINT_INFO("     │  |t|             %s  %.5f 门限≤%.5f "
                   "(否决·归一化非米)\n",
                   t_s, tnorm, thr_t_s);
      } else {
        PRINT_INFO("     │  |t|             未通过  n/a\n");
      }

      PRINT_INFO("     └─ [合并]          赞同||否决=%s  (f_gate+重投影+|t|，"
                 "仅诊断)\n",
                 res_merge_geom);
    }
  }

  return gate_passed;
}

bool UpdaterPureRotation::apply_pure_rot_homography_update(
    std::shared_ptr<State> state, double timestamp,
    const std::vector<ov_core::ImuData> &imu_recent,
    double offset_used_for_time0) {

  _last_debug.preint_homography_update = false;

  if (_db == nullptr || state->_calib_IMUtoCAM.find(_ref_cam_id) ==
                           state->_calib_IMUtoCAM.end() ||
      state->_cam_intrinsics_cameras.empty()) {
    if (_print_pure_rot) {
      PRINT_WARNING(RED "[PureRot-H]: 无特征库或标定，跳过\n" RESET);
    }
    return false;
  }

  // 参考帧：t_ref↔当前；否则上一 IMU 状态时刻↔当前（与 verify 门控时间基一致当且仅当非 ref）
  const bool use_ref =
      _use_visual_ref_rotation && _ref_valid && timestamp > _ref_cam_time + 1e-9 &&
      (timestamp - _ref_cam_time) <= _visual_max_ref_age_sec;

  const double t_cam0 = use_ref ? _ref_cam_time : state->_timestamp;
  const Eigen::Matrix3d R_GtoI0 =
      use_ref ? quat_2_Rot(_q_GtoI_ref) : state->_imu->Rot();
  const double homography_ransac =
      use_ref ? _visual_ransac_thresh_norm : _f_ransac_thresh_norm;

  std::vector<Eigen::Vector2d> pref, pcur;
  cv::Mat H_cv, h_mask;
  Eigen::Matrix3d R_meas;
  int n_h_inliers = 0;
  double best_tn = 0.0;
  int npair = 0;
  bool have_R_from_H = false;

  // 相邻帧且检验阶段已缓存门控③的单应：只分解 H，不再 findHomography
  const bool try_reuse_gate_h =
      !use_ref && _adjacent_gate_h_valid &&
      std::abs(_adjacent_gate_t0 - state->_timestamp) < 1e-9 &&
      std::abs(_adjacent_gate_t1 - timestamp) < 1e-9;
  if (try_reuse_gate_h) {
    npair = (int)_adjacent_gate_pref.size();
    const int nin = _adjacent_gate_mask.empty()
                        ? npair
                        : cv::countNonZero(_adjacent_gate_mask);
    if (npair >= _visual_min_matches && nin >= _visual_min_inliers &&
        decompose_homography_mat_to_R(_adjacent_gate_H, R_meas, &best_tn)) {
      pref = _adjacent_gate_pref;
      pcur = _adjacent_gate_pcur;
      H_cv = _adjacent_gate_H;
      h_mask = _adjacent_gate_mask;
      n_h_inliers = nin;
      have_R_from_H = true;
      _last_debug.visual_n_matches = npair;
    }
  }

  if (!have_R_from_H) {
    npair = FeatureHelper::collect_normalized_correspondences(
        _db, t_cam0, timestamp, _ref_cam_id, state->_cam_intrinsics_cameras,
        pref, pcur);
    _last_debug.visual_n_matches = npair;
    if (npair < _visual_min_matches) {
      if (_print_pure_rot) {
        PRINT_WARNING(RED "[PureRot-H]: 匹配对不足 %d < %d\n" RESET, npair,
                      _visual_min_matches);
      }
      return false;
    }

    cv::Mat pts1(npair, 2, CV_64F);
    cv::Mat pts2(npair, 2, CV_64F);
    for (int i = 0; i < npair; i++) {
      pts1.at<double>(i, 0) = pref[i](0);
      pts1.at<double>(i, 1) = pref[i](1);
      pts2.at<double>(i, 0) = pcur[i](0);
      pts2.at<double>(i, 1) = pcur[i](1);
    }
    if (!pure_rot_homography_estimate_R(pts1, pts2, homography_ransac,
                                        _visual_min_inliers, R_meas, n_h_inliers,
                                        &best_tn, &H_cv, &h_mask)) {
      if (_print_pure_rot) {
        PRINT_WARNING(RED "[PureRot-H]: 单应 H 估计或分解失败\n" RESET);
      }
      return false;
    }
  }

  _last_debug.preint_homography_update = true;

  // 外参在传播前后不变，一次读取供 H 调试量与残差雅可比共用
  const Eigen::Matrix3d R_ItoC =
      state->_calib_IMUtoCAM.at(_ref_cam_id)->Rot();
  const Eigen::Matrix3d R_CtoI = R_ItoC.transpose();

  Eigen::Matrix3d He;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      He(r, c) = H_cv.at<double>(r, c);
  double reproj_sum = 0.0;
  int reproj_cnt = 0;
  for (int i = 0; i < npair; i++) {
    if (!h_mask.empty() && !h_mask.at<unsigned char>(i))
      continue;
    Eigen::Vector3d x1(pref[i](0), pref[i](1), 1.0);
    Eigen::Vector3d xh = He * x1;
    if (std::abs(xh(2)) < 1e-12)
      continue;
    xh /= xh(2);
    const double ex = xh(0) - pcur[i](0);
    const double ey = xh(1) - pcur[i](1);
    reproj_sum += std::sqrt(ex * ex + ey * ey);
    reproj_cnt++;
  }
  const double reproj_mean =
      reproj_cnt > 0 ? (reproj_sum / (double)reproj_cnt) : -1.0;

  const Eigen::Vector3d omega_h = log_so3(R_meas);
  const double ang_h_deg = omega_h.norm() * 180.0 / M_PI;
  constexpr double rad2deg = 180.0 / M_PI;
  const Eigen::Matrix<double, 3, 1> rpy_cam_rad = rot2rpy(R_meas);
  const Eigen::Matrix<double, 4, 1> q_jpl_h = rot_2_quat(R_meas);
  const Eigen::Matrix3d R_H_in_I = R_CtoI * R_meas * R_ItoC;
  const Eigen::Matrix<double, 3, 1> rpy_imu_rad = rot2rpy(R_H_in_I);

  _last_debug.preint_h_inliers = n_h_inliers;
  _last_debug.preint_h_rot_angle_deg = ang_h_deg;
  _last_debug.preint_h_decomp_t_norm = best_tn;
  _last_debug.preint_h_reproj_mean = reproj_mean;
  _last_debug.preint_h_rpy_roll_deg = rpy_cam_rad(0) * rad2deg;
  _last_debug.preint_h_rpy_pitch_deg = rpy_cam_rad(1) * rad2deg;
  _last_debug.preint_h_rpy_yaw_deg = rpy_cam_rad(2) * rad2deg;
  _last_debug.preint_h_imu_rpy_roll_deg = rpy_imu_rad(0) * rad2deg;
  _last_debug.preint_h_imu_rpy_pitch_deg = rpy_imu_rad(1) * rad2deg;
  _last_debug.preint_h_imu_rpy_yaw_deg = rpy_imu_rad(2) * rad2deg;
  _last_debug.preint_h_quat_jpl_0 = q_jpl_h(0);
  _last_debug.preint_h_quat_jpl_1 = q_jpl_h(1);
  _last_debug.preint_h_quat_jpl_2 = q_jpl_h(2);
  _last_debug.preint_h_quat_jpl_3 = q_jpl_h(3);

  // 相邻帧时 RPY 已在 [PureRot-H·检验] 与 R、t 一并打印；参考帧几何与检验时间基不同，仅此时补一行 RPY
  if (_print_pure_rot && _print_state_calib && use_ref) {
    PRINT_INFO(CYAN "  [PureRot-H·更新·参考帧] RPY相机(deg) roll=%.4f pitch=%.4f "
                    "yaw=%.4f | IMU等价 roll=%.4f pitch=%.4f yaw=%.4f\n" RESET,
               _last_debug.preint_h_rpy_roll_deg, _last_debug.preint_h_rpy_pitch_deg,
               _last_debug.preint_h_rpy_yaw_deg, _last_debug.preint_h_imu_rpy_roll_deg,
               _last_debug.preint_h_imu_rpy_pitch_deg,
               _last_debug.preint_h_imu_rpy_yaw_deg);
  }

  _propagator->sync_last_prop_time_offset_for_sibling(offset_used_for_time0);
  if (!_propagator->propagate_using_selected_imu(state, timestamp, imu_recent,
                                                 nullptr)) {
    if (_print_pure_rot) {
      PRINT_WARNING(RED "[PureRot-H]: IMU 传播失败\n" RESET);
    }
    _last_debug.preint_homography_update = false;
    return false;
  }

  const Eigen::Matrix3d R_GtoI1 = state->_imu->Rot();
  const Eigen::Matrix3d R_pred = R_ItoC * R_GtoI1 * R_GtoI0.transpose() * R_CtoI;
  const Eigen::Vector3d r0 = log_so3(R_meas.transpose() * R_pred);

  // ∂(log_so3)/∂δθ：对 JPL 左扰动 exp(δθ) 做数值差分（3 列）
  const double eps = 1e-5;
  Eigen::Matrix3d Hq = Eigen::Matrix3d::Zero();
  for (int c = 0; c < 3; c++) {
    Eigen::Vector3d dq = Eigen::Vector3d::Zero();
    dq(c) = eps;
    const Eigen::Matrix3d R_GtoI1p = exp_so3(dq) * R_GtoI1;
    const Eigen::Matrix3d R_pred_p =
        R_ItoC * R_GtoI1p * R_GtoI0.transpose() * R_CtoI;
    const Eigen::Vector3d rp = log_so3(R_meas.transpose() * R_pred_p);
    Hq.col(c) = (rp - r0) / eps;
  }

  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->bg());
  Hx_order.push_back(state->_imu->ba());
  Hx_order.push_back(state->_imu->v());
  const Eigen::MatrixXd P_marg =
      StateHelper::get_marginal_covariance(state, Hx_order);
  const Eigen::MatrixXd P_q = P_marg.block(0, 0, 3, 3);
  const Eigen::Matrix3d R_noise =
      Eigen::Matrix3d::Identity() * std::pow(_visual_sigma_rad, 2) *
      _visual_noise_multiplier;
  const Eigen::Matrix3d S_vis = Hq * P_q * Hq.transpose() + R_noise;
  Eigen::LLT<Eigen::Matrix3d> llt_S(S_vis);
  if (llt_S.info() != Eigen::Success) {
    if (_print_pure_rot) {
      PRINT_WARNING(RED "[PureRot-H]: S 不正定，仅完成传播  "
                         "∠R_H=%.3f°  H重投影均值=%.5f  分解|t|=%.5f\n" RESET,
                    ang_h_deg, reproj_mean, best_tn);
    }
    apply_pure_rot_zero_vel_pos_anchor(state);
    return true;
  }
  const double chi2_v = r0.dot(llt_S.solve(r0));
  const double lim_v =
      _visual_chi2_multiplier * ov_core::chi2_quantile_095(3);
  _last_debug.visual_chi2 = chi2_v;
  _last_debug.visual_res_norm = r0.norm();

  bool did_meas = false;
  if (chi2_v <= lim_v && r0.norm() < 1.5) {
    Eigen::MatrixXd H_meas = Eigen::MatrixXd::Zero(3, 12);
    H_meas.block(0, 0, 3, 3) = Hq;
    Eigen::MatrixXd R_meas_mx = Eigen::MatrixXd::Zero(3, 3);
    R_meas_mx = R_noise;
    StateHelper::EKFUpdate(state, Hx_order, H_meas, r0, R_meas_mx);
    did_meas = true;
  } else if (_print_pure_rot) {
    PRINT_WARNING(YELLOW "[PureRot-H]: χ² 或 |log_so3| 过大，仅完成传播  "
                       "∠R_H=%.3f°  H重投影均值=%.5f  分解|t|=%.5f  "
                       "χ²=%.3f lim=%.3f |r|=%.4f\n" RESET,
                  ang_h_deg, reproj_mean, best_tn, chi2_v, lim_v, r0.norm());
  }

  apply_pure_rot_zero_vel_pos_anchor(state);

  _last_debug.ekf_updated = did_meas;
  _last_debug.residual_rows = did_meas ? 3 : 0;
  _last_debug.visual_rotation_used = did_meas;
  if (_print_pure_rot && _print_state_calib && did_meas) {
    PRINT_INFO(GREEN "  [PureRot-H] t0=%.6f→t1=%.6f  匹配=%d  H内点=%d  "
                     "∠R_H=%.3f°  H重投影均值=%.5f  分解|t|=%.5f  χ²=%.3f  "
                     "|log_so3|=%.4f rad\n" RESET,
               t_cam0, timestamp, npair, n_h_inliers, ang_h_deg, reproj_mean,
               best_tn, chi2_v, r0.norm());
  }
  return true;
}

void UpdaterPureRotation::apply_pure_rot_zero_vel_pos_anchor(
    std::shared_ptr<State> state) {
  if (!_pure_rot_lock_vel_pos || !_pure_rot_episode_active ||
      !_pure_rot_has_anchor_p || state == nullptr) {
    if (_print_pure_rot) {
      PRINT_INFO(
          YELLOW
          "  [PureRot·ZUPT锚] 跳过  lock_vel_pos=%d  episode=%d  has_anchor=%d\n"
          RESET,
          (int)_pure_rot_lock_vel_pos, (int)_pure_rot_episode_active,
          (int)_pure_rot_has_anchor_p);
    }
    return;
  }

  const double sv2 = _pure_rot_zero_vel_sigma * _pure_rot_zero_vel_sigma;
  const double sp2 = _pure_rot_anchor_pos_sigma * _pure_rot_anchor_pos_sigma;

  // 与 MSCKF / StateHelper::EKFUpdate 一致：残差 res = z - h(x)，H = ∂h/∂δx（此处对 v、p 为加性误差，H=I）。
  // 零速伪观测 z_v=0 ⇒ res_v = 0 - v = -v。
  // 位置软锚 z_p=p_anchor ⇒ res_p = p_anchor - p。
  const Eigen::Vector3d v_before = state->_imu->vel();
  const Eigen::Vector3d p_before = state->_imu->pos();
  const Eigen::Vector3d innov_v = -v_before;
  const Eigen::Vector3d innov_p = _pure_rot_anchor_p_in_G - p_before;

  if (_print_pure_rot) {
    PRINT_INFO(
        CYAN
        "  [PureRot·ZUPT锚] 假设: 线速度≈0 (σ=%.4f m/s) + 位置相对段首软锚 (σ=%.4f m)\n"
        "    锚点 p_anchor=[%.4f,%.4f,%.4f]  |v|前=%.5f  |p-p_anchor|前=%.5f\n"
        "    创新(=z-h):  v: z_v=0 ⇒ res=%.5f,%.5f,%.5f  |res|=%.5f\n"
        "                 p: res=p_anchor-p = %.5f,%.5f,%.5f  |res|=%.5f\n"
        RESET,
        _pure_rot_zero_vel_sigma, _pure_rot_anchor_pos_sigma,
        _pure_rot_anchor_p_in_G(0), _pure_rot_anchor_p_in_G(1),
        _pure_rot_anchor_p_in_G(2), v_before.norm(),
        (p_before - _pure_rot_anchor_p_in_G).norm(), innov_v(0), innov_v(1),
        innov_v(2), innov_v.norm(), innov_p(0), innov_p(1), innov_p(2),
        innov_p.norm());
  }

  std::vector<std::shared_ptr<Type>> Hv_order;
  Hv_order.push_back(state->_imu->v());
  Eigen::MatrixXd Hv = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd res_v = innov_v;
  Eigen::MatrixXd Rv = Eigen::MatrixXd::Identity(3, 3) * sv2;
  StateHelper::EKFUpdate(state, Hv_order, Hv, res_v, Rv);

  if (_pure_rot_vel_cov_diag_min > 0.0) {
    std::vector<std::shared_ptr<Type>> v_only = {state->_imu->v()};
    Eigen::MatrixXd Pv =
        StateHelper::get_marginal_covariance(state, v_only);
    for (int i = 0; i < 3; i++) {
      if (Pv(i, i) < _pure_rot_vel_cov_diag_min)
        Pv(i, i) = _pure_rot_vel_cov_diag_min;
    }
    StateHelper::set_initial_covariance(state, Pv, v_only);
  }

  std::vector<std::shared_ptr<Type>> Hp_order;
  Hp_order.push_back(state->_imu->p());
  Eigen::MatrixXd Hp = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd res_p = innov_p;
  Eigen::MatrixXd Rp = Eigen::MatrixXd::Identity(3, 3) * sp2;
  StateHelper::EKFUpdate(state, Hp_order, Hp, res_p, Rp);

  if (_print_pure_rot) {
    const Eigen::Vector3d v_after = state->_imu->vel();
    const Eigen::Vector3d p_after = state->_imu->pos();
    PRINT_INFO(
        GREEN
        "  [PureRot·ZUPT锚] 更新后 |v|=%.5f (Δ|v|=%.5f)  |p-p_anchor|=%.5f (Δ=%.5f)\n"
        RESET,
        v_after.norm(), v_after.norm() - v_before.norm(),
        (p_after - _pure_rot_anchor_p_in_G).norm(),
        (p_after - _pure_rot_anchor_p_in_G).norm() -
            (p_before - _pure_rot_anchor_p_in_G).norm());
  }
}
