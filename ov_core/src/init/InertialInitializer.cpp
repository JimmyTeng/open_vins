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

#include "InertialInitializer.h"

#include <algorithm>
#include <cmath>
#include <set>

#ifndef __ANDROID__
#include "dynamic/DynamicInitializer.h"
#endif
#include "feat/FeatureHelper.h"
#include "static/StaticInitializer.h"
#include "types/IMU.h"
#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

InertialInitializer::InertialInitializer(
    InertialInitializerOptions &params_,
    std::shared_ptr<ov_core::FeatureDatabase> db)
    : params(params_), _db(db) {
  // 创建IMU数据向量
  imu_data = std::make_shared<std::vector<ov_core::ImuData>>();

  // 创建初始化器
  // 静态初始化器：用于设备静止时的初始化
  init_static = std::make_shared<StaticInitializer>(params, _db, imu_data);
#ifndef __ANDROID__
  // 动态初始化器：用于设备运动时的初始化（Android平台不支持，因为需要Ceres
  // Solver）
  init_dynamic = std::make_shared<DynamicInitializer>(params, _db, imu_data);
#else
  init_dynamic = nullptr;
#endif
}

void InertialInitializer::feed_imu(const ov_core::ImuData &message,
                                   double oldest_time) {
  // 将IMU数据添加到向量中
  imu_data->emplace_back(message);

  // 对IMU数据进行排序（处理任何乱序的测量数据）
  // std::sort(imu_data->begin(), imu_data->end(), [](const IMUDATA i, const
  // IMUDATA j) {
  //    return i.timestamp < j.timestamp;
  //});

  // 遍历并删除早于请求时间的IMU消息
  // std::cout << "INIT: imu_data.size() " << imu_data->size() << std::endl;
  if (oldest_time != -1) {
    auto it0 = imu_data->begin();
    while (it0 != imu_data->end()) {
      if (it0->timestamp < oldest_time) {
        // 删除过旧的IMU数据
        it0 = imu_data->erase(it0);
      } else {
        it0++;
      }
    }
  }
}

void InertialInitializer::update_motion_still_state(bool is_still,
                                                     double cam_time) {
  if (!motion_state_initialized) {
    motion_state_initialized = true;
    if (is_still) {
      motion_still_state = MotionStillState::STILL;
      static_seen_still_phase = true;
      static_enter_still_cam_time = cam_time;
      static_last_still_cam_time = cam_time;
      static_leave_still_cam_time = -1.0;
      static_wait_log_last_time = cam_time;
      PRINT_INFO(
          CYAN
          "[InertialInitializer]: 状态初始化=静止, enter=%.6f s\n" RESET,
          cam_time);
    } else {
      motion_still_state = MotionStillState::MOVING;
      PRINT_INFO(
          CYAN
          "[InertialInitializer]: 状态初始化=运动, t=%.6f s\n" RESET, cam_time);
    }
    return;
  }

  const bool was_still =
      (motion_still_state == MotionStillState::STILL ||
       motion_still_state == MotionStillState::STOPPING);

  if (is_still) {
    if (!was_still) {
      motion_still_state = MotionStillState::STOPPING;
      static_seen_still_phase = true;
      static_enter_still_cam_time = cam_time;
      static_last_still_cam_time = cam_time;
      static_leave_still_cam_time = -1.0;
      static_wait_log_last_time = cam_time;
      PRINT_INFO(
          CYAN
          "[InertialInitializer]: 状态切换 运动->停止, enter=%.6f s\n" RESET,
          cam_time);
    } else {
      motion_still_state = MotionStillState::STILL;
      static_seen_still_phase = true;
      static_last_still_cam_time = cam_time;
    }
    return;
  }

  // 当前为运动
  if (was_still && static_last_still_cam_time >= 0.0 &&
      cam_time > static_last_still_cam_time) {
    motion_still_state = MotionStillState::STARTING;
    static_leave_still_cam_time = cam_time;
    const double still_duration = current_still_duration_sec();
    const double static_window_time = params.init_static_window_time;
    const bool static_gate_ok = (still_duration >= static_window_time);
    PRINT_INFO(
        CYAN
        "[InertialInitializer]: 状态切换 静止->启动, leave=%.6f s, 静止持续 %.2f s, "
        "判据[still_duration>=static_window_time]: %.2f>=%.2f => %s\n"
        RESET,
        cam_time, still_duration, still_duration, static_window_time,
        static_gate_ok ? "满足" : "不满足");
  } else {
    motion_still_state = MotionStillState::MOVING;
  }
}

double InertialInitializer::current_still_duration_sec() const {
  if (!static_seen_still_phase || static_enter_still_cam_time < 0.0 ||
      static_last_still_cam_time < static_enter_still_cam_time) {
    return 0.0;
  }
  return static_last_still_cam_time - static_enter_still_cam_time;
}

bool InertialInitializer::initialize(
    double &timestamp, Eigen::MatrixXd &covariance,
    std::vector<std::shared_ptr<ov_type::Type>> &order,
    std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk) {
  (void)wait_for_jerk;
  // 获取我们将尝试初始化的最新和最旧时间戳
  double newest_cam_time = -1;
  std::set<double> all_cam_times;
  // 遍历特征数据库，找到最新的相机时间戳
  for (auto const &feat : _db->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        all_cam_times.insert(time);
        newest_cam_time = std::max(newest_cam_time, time);
      }
    }
  }
  // 计算初始化窗口的最旧时间（静态/动态取更大窗口，加额外缓冲0.10秒）
  const double static_window_time = params.init_static_window_time;
  const double dynamic_window_time = params.init_dyn_window_time;
  const double init_window_time =
      std::max(static_window_time, dynamic_window_time);
  double oldest_time = newest_cam_time - init_window_time - 0.10;
  if (newest_cam_time < 0 || oldest_time < 0) {
    PRINT_INFO(
        YELLOW
        "[InertialInitializer]: 返回 false（原因: 时间戳无效 newest=%.6f oldest=%.6f）\n"
        RESET,
        newest_cam_time, oldest_time);
    return false;
  }

  // 删除初始化窗口之前的所有测量数据
  // 然后我们将尝试使用特征数据库中的所有特征！
  _db->cleanup_measurements(oldest_time);
  // 删除过旧的IMU数据（考虑相机-IMU时间偏移）
  auto it_imu = imu_data->begin();
  while (it_imu != imu_data->end() &&
         it_imu->timestamp < oldest_time + params.calib_camimu_dt) {
    it_imu = imu_data->erase(it_imu);
  }

  // 计算当前窗口的总体视差（直接判定静止/运动，不再分前后半段）
  bool disparity_detected_moving = false;
  double chi2_stat = 0.0;
  double chi2_p_upper = 1.0;
  bool chi2_confident_exceed = false;
  int disparity_num_features = 0;
  double disparity_avg = -1.0;
  double disparity_var = -1.0;
  if (params.init_max_disparity > 0) {
    FeatureHelper::compute_disparity(_db, disparity_avg, disparity_var,
                                     disparity_num_features, newest_cam_time);

    // 如果无法计算视差则返回
    int feat_thresh = 15;  // 特征数量阈值
    if (disparity_num_features < feat_thresh) {
      // PRINT_INFO(YELLOW "[InertialInitializer]:初始化失败,
      // 视差计算所需特征不足: min(前段特征 %d, 后段特征 %d) < %d\n" RESET,
      // num_features0, num_features1, feat_thresh);
      PRINT_INFO(
          YELLOW
          "[InertialInitializer]: 返回 false（原因: 视差特征不足 %d < %d）\n" RESET,
          disparity_num_features, feat_thresh);
      return false;
    }

    // 卡方检验（95%置信）：当均值高于阈值且显著性足够高时判定“超标运动”。
    // 这里使用 df=1 的统计量：chi2 = n * (mean-th)^2 / std^2（近似）。
    const double thr = params.init_max_disparity;
    const double delta = disparity_avg - thr;
    if (delta > 0.0) {
      const double std_eps = 1e-9;
      if (disparity_var > std_eps) {
        chi2_stat = static_cast<double>(disparity_num_features) * delta * delta /
                    (disparity_var * disparity_var);
        // df=1 时上尾概率 p = erfc(sqrt(chi2/2))
        chi2_p_upper = std::erfc(std::sqrt(0.5 * chi2_stat));
      } else {
        // 方差几乎为零且均值已超阈值，视为极显著超标
        chi2_stat = INFINITY;
        chi2_p_upper = 0.0;
      }
      chi2_confident_exceed = (chi2_p_upper < 0.05);
    }
    disparity_detected_moving = chi2_confident_exceed;
  }

  // 初始化分阶段策略：
  // 阶段1（静态）：只要收集到足够长时间的静止数据就完成静态初始化，得到可用 bias 起点。
  // 阶段2（动态）：再执行动态初始化，动态成功后才算整体初始化成功。
  // 这样可确保配置中的 init_dyn_bias_* 会被静态估计值更新后再参与动态求解。
  //
  bool is_still = !disparity_detected_moving;
  PRINT_INFO(
      CYAN
      "[InertialInitializer]: 静止判据(卡方95%%): avg=%.6f, thr=%.6f, std=%.6f, "
      "N=%d, chi2=%.6f, p_upper=%.6e, exceed95=%d => is_still=%d\n" RESET,
      disparity_avg, params.init_max_disparity, disparity_var,
      disparity_num_features, chi2_stat, chi2_p_upper,
      (int)chi2_confident_exceed, (int)is_still);
  if (params.init_max_disparity <= 0.0) {
    PRINT_INFO(
        YELLOW
        "[InertialInitializer]: 注意: init_max_disparity<=0，当前默认按静止处理\n"
        RESET);
  }
  if (print_debug) {
    PRINT_DEBUG(YELLOW
                "[InertialInitializer]: 直接视差判定: moving=%d, still=%d\n"
                RESET,
                disparity_detected_moving, is_still);
  }
  update_motion_still_state(is_still, newest_cam_time);
  const double still_duration = current_still_duration_sec();
  // 在“上一时刻静止、这一时刻离开静止”时，使用离开前静止段数据做静态初始化。
  // 这样能保证静态初始化仅基于上一静止阶段，不混入当前运动帧。
  const bool leaving_still =
      (motion_still_state == MotionStillState::STARTING &&
                              static_seen_still_phase &&
                              static_enter_still_cam_time >= 0.0 &&
                              static_last_still_cam_time >= 0.0 &&
                              static_last_still_cam_time < newest_cam_time);
  if (!static_stage_done && leaving_still) {
    const bool static_gate_ok = (still_duration >= static_window_time);
    PRINT_INFO(
        CYAN
        "[InertialInitializer]: leaving_still 检测到，静止持续时长=%.2f s（静态初始化需>=%.2f s）\n"
        RESET,
        still_duration, static_window_time);

    // 离开静止但静止时长不足：仅清理静止阶段状态，不执行静态初始化。
    if (!static_gate_ok) {
      static_seen_still_phase = false;
      static_last_still_cam_time = -1.0;
      static_enter_still_cam_time = -1.0;
      static_wait_log_last_time = -1.0;
      motion_still_state = MotionStillState::MOVING;
      PRINT_INFO(
          YELLOW
          "[InertialInitializer]: leaving_still 但静止时长不足，仅清理静止阶段状态并继续后续流程\n"
          RESET);
    } else {
    // 使用卡方判据后，静态初始化不使用静止段最后两帧，避免边沿帧污染。
    std::vector<double> still_cam_times;
    still_cam_times.reserve(all_cam_times.size());
    for (double t : all_cam_times) {
      if (t <= static_last_still_cam_time + 1e-9) {
        still_cam_times.push_back(t);
      }
    }
    if (still_cam_times.size() <= 2) {
      PRINT_INFO(
          YELLOW
          "[InertialInitializer]: 返回 false（原因: 静态段帧数=%zu，去掉最后两帧后无可用数据）\n"
          RESET,
          still_cam_times.size());
      return false;
    }
    const double static_usable_end_cam_time =
        still_cam_times[still_cam_times.size() - 3];
    PRINT_INFO(
        CYAN
        "[InertialInitializer]: 静态初始化数据裁剪: 总静态帧=%zu, 丢弃最后2帧, "
        "usable_end_cam=%.6f s（原last_still=%.6f s）\n" RESET,
        still_cam_times.size(), static_usable_end_cam_time,
        static_last_still_cam_time);

    const double last_still_imu_time =
        static_usable_end_cam_time + params.calib_camimu_dt;
    auto it_trim = imu_data->begin();
    while (it_trim != imu_data->end()) {
      if (it_trim->timestamp > last_still_imu_time) {
        it_trim = imu_data->erase(it_trim);
      } else {
        ++it_trim;
      }
    }

    const bool imu_count_ok = (imu_data->size() >= 2);
    double still_span = 0.0;
    bool still_span_ok = false;
    if (imu_count_ok) {
      still_span = imu_data->back().timestamp - imu_data->front().timestamp;
      still_span_ok = (still_span >= static_window_time);
    }
    PRINT_INFO(
        CYAN
        "[InertialInitializer]: leaving_still 条件检查: imu_ok=%d "
        "(samples=%zu), span_ok=%d (span=%.2f s, need>=%.2f s)\n" RESET,
        (int)imu_count_ok, imu_data->size(), (int)still_span_ok, still_span,
        static_window_time);

    if (!imu_count_ok || !still_span_ok) {
      if (!imu_count_ok) {
        PRINT_INFO(YELLOW
                   "[InertialInitializer]: leaving_still 失败: 静止段 IMU 样本不足\n"
                   RESET);
      } else {
        PRINT_INFO(
            YELLOW
            "[InertialInitializer]: leaving_still 失败: 静止段不足 %.2f s (当前 %.2f s)\n"
            RESET,
            static_window_time, still_span);
      }
      PRINT_INFO(
          YELLOW
          "[InertialInitializer]: 返回 false（原因: leaving_still 不满足静态初始化条件）\n"
          RESET);
      return false;
    }

    double gravity_mag_est = params.gravity_mag;
    const bool static_success = init_static->initialize(
        timestamp, covariance, order, t_imu, false, &gravity_mag_est);
    if (!static_success) {
      PRINT_INFO(
          YELLOW
          "[InertialInitializer]: 返回 false（原因: leaving_still 下静态初始化执行失败）\n"
          RESET);
      return false;
    }

    // 仅在离开静止触发成功后执行一次：裁剪静止段，避免其进入动态初始化窗口。
    const double dynamic_cut_cam_time = static_last_still_cam_time + 1e-6;
    const double dynamic_cut_imu_time =
        dynamic_cut_cam_time + params.calib_camimu_dt;
    _db->cleanup_measurements(dynamic_cut_cam_time);
    auto it_dyn = imu_data->begin();
    while (it_dyn != imu_data->end()) {
      if (it_dyn->timestamp < dynamic_cut_imu_time) {
        it_dyn = imu_data->erase(it_dyn);
      } else {
        ++it_dyn;
      }
    }
    PRINT_INFO(CYAN
               "[InertialInitializer]: 动态阶段已裁剪静止段数据，cut_cam=%.6f s\n"
               RESET,
               dynamic_cut_cam_time);

    static_stage_done = true;
    static_wait_log_last_time = -1.0;
    static_seen_still_phase = false;
    static_last_still_cam_time = -1.0;
    static_enter_still_cam_time = -1.0;
    motion_still_state = MotionStillState::MOVING;
    params.init_dyn_bias_g = t_imu->bias_g();
    params.init_dyn_bias_a = t_imu->bias_a();
    params.gravity_mag = gravity_mag_est;
    PRINT_INFO(CYAN
               "[InertialInitializer]: 离开静止触发静态阶段成功，更新动态初值 "
               "bg=(%.6f, %.6f, %.6f), ba=(%.6f, %.6f, %.6f), |g|=%.6f\n" RESET,
               params.init_dyn_bias_g(0), params.init_dyn_bias_g(1),
               params.init_dyn_bias_g(2), params.init_dyn_bias_a(0),
               params.init_dyn_bias_a(1), params.init_dyn_bias_a(2),
               params.gravity_mag);
#ifndef __ANDROID__
    if (init_dynamic) {
      init_dynamic->set_initial_bias_guess(params.init_dyn_bias_g,
                                           params.init_dyn_bias_a);
      init_dynamic->set_gravity_magnitude(params.gravity_mag);
    }
#endif
    PRINT_INFO(
        CYAN
        "[InertialInitializer]: 返回 false（原因: leaving_still 成功后等待下一轮动态初始化）\n"
        RESET);
    return false;
    }
  }
  // ----------------------------
  // 阶段2：动态初始化（如果本轮尚未尝试，则再尝试一次）
  // ----------------------------
  if (!is_still) {
#ifndef __ANDROID__  // Android平台不支持动态初始化，因为需要Ceres Solver
    if (init_dynamic) {
      std::map<double, std::shared_ptr<ov_type::PoseJPL>>
          _clones_IMU;  // IMU位姿克隆
      std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>>
          _features_SLAM;  // SLAM特征
      const bool dynamic_ok = init_dynamic->initialize(
          timestamp, covariance, order, t_imu, _clones_IMU, _features_SLAM);
      PRINT_INFO(
          CYAN
          "[InertialInitializer]: 返回 %s（原因: 动态初始化执行完成）\n" RESET,
          dynamic_ok ? "true" : "false");
      return dynamic_ok;
    }
    PRINT_ERROR(RED
                "[InertialInitializer]: 动态初始化器不可用 (Ceres "
                "Solver未包含)\n" RESET);
#else
    PRINT_ERROR(RED
                "[InertialInitializer]: DYNAMIC INITIALIZER not available on "
                "Android (Ceres Solver not included)\n" RESET);
#endif
    PRINT_INFO(
        YELLOW
        "[InertialInitializer]: 返回 false（原因: 动态初始化器不可用）\n" RESET);
    return false;
  }
  PRINT_INFO(
      YELLOW
      "[InertialInitializer]: 返回 false（原因: 当前判定静止，等待离开静止后触发初始化）\n"
      RESET);
  return false;
}
