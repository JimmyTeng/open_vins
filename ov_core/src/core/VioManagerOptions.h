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

#ifndef OV_MSCKF_VIOMANAGEROPTIONS_H
#define OV_MSCKF_VIOMANAGEROPTIONS_H

#include <algorithm>
#include <Eigen/Eigen>
#include "utils/fs_compat.h"
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "opencv2/highgui.hpp"
#include "state/StateOptions.h"
#include "update/UpdaterOptions.h"
#include "utils/NoiseManager.h"

#include "init/InertialInitializerOptions.h"

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "feat/FeatureInitializerOptions.h"
#include "track/TrackBase.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_msckf {

/**
 * @brief 存储状态估计所需的所有选项的结构体
 *
 * 这分为几个不同的部分：估计器、跟踪器和仿真。
 * 如果要在此处添加参数，需要将其添加到解析器中。
 * 还需要将其添加到每个部分底部的打印语句中。
 * 
 * @brief Struct which stores all options needed for state estimation.
 *
 * This is broken into a few different parts: estimator, trackers, and simulation.
 * If you are going to add a parameter here you will need to add it to the parsers.
 * You will also need to add it to the print statement at the bottom of each.
 */
struct VioManagerOptions {

  /**
   * @brief 加载并打印系统的非仿真参数
   * @param parser 如果不为空，将使用此解析器加载参数
   * 
   * @brief This function will load the non-simulation parameters of the system and print.
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    print_and_load_estimator(parser);
    print_and_load_trackers(parser);
    print_and_load_noise(parser);

    // needs to be called last
    print_and_load_state(parser);
  }

  // ESTIMATOR ===============================
  // 估计器参数 ===============================

  /// 核心状态选项（例如相机数量、是否使用FEJ、双目、启用哪些标定等）
  /// Core state options (e.g. number of cameras, use fej, stereo, what calibration to enable etc)
  StateOptions state_options;

  /// 状态初始化选项（例如窗口大小、特征点数量、是否获取标定参数）
  /// Our state initialization options (e.g. window size, num features, if we should get the calibration)
  ov_init::InertialInitializerOptions init_options;

  /// 延迟时间（秒），在初始化后等待此时间再开始估计SLAM特征点
  /// Delay, in seconds, that we should wait from init before we start estimating SLAM features
  double dt_slam_delay = 2.0;

  /// 是否尝试使用零速度更新
  /// If we should try to use zero velocity update
  bool try_zupt = false;

  /// 尝试执行零速度更新的最大速度（如果超过此值，则不执行零速度更新）
  /// Max velocity we will consider to try to do a zupt (i.e. if above this, don't do zupt)
  double zupt_max_velocity = 1.0;

  /// ZUPT 测量噪声 R 的标量倍率（马氏 χ² 与 EKF 更新共用，非 OR/AND 分支各自一份）
  /// Scalar on ZUPT measurement noise R (shared for χ² innovation and EKF update)
  double zupt_noise_multiplier = 1.0;

  /// 尝试执行零速度更新的最大视差（如果超过此值，则不执行零速度更新）
  /// Max disparity we will consider to try to do a zupt (i.e. if above this, don't do zupt)
  double zupt_max_disparity = 1.0;

  /// 是否仅在初始静态初始化阶段使用零速度更新
  /// If we should only use the zupt at the very beginning static initialization phase
  bool zupt_only_at_beginning = false;

  /// 延迟退出 ZUPT：仅当连续 n 回合检测到“离开静止门失败”才退出 ZUPT（n=1 表示立即退出）
  int zupt_exit_consecutive_failures = 1;

  /// 确认退出 ZUPT 时，对 IMU(15维)协方差整体倍化系数（1.0=关闭）
  double zupt_exit_cov_inflation = 1.0;

  /// 赞同阈值支路（gate_agree；可选 legacy zupt_* 先填两路，再由 zupt_agree_* / zupt_strict_* 或旧键 zupt_or_* / zupt_and_* 覆盖）
  double zupt_agree_max_velocity = 1.0;
  double zupt_agree_max_disparity = 1.0;
  double zupt_agree_chi2_multipler = 1.0;
  /// 否决阈值支路（gate_strict）
  double zupt_strict_max_velocity = 1.0;
  double zupt_strict_max_disparity = 1.0;
  double zupt_strict_chi2_multipler = 1.0;

  /// 是否打印 ZUPT 运行时 [ZUPT] 日志与启动时 ZUPT 参数说明（默认关闭）
  bool print_zupt = false;

  /// 是否启用纯旋转更新器（与 ZUPT 检测独立；默认在 ZUPT 本帧未成功后再尝试）
  bool try_pure_rot = false;

  /// 为 true 时：仅当本帧已启用 ZUPT 且 ZUPT 未通过静止门时才尝试纯旋转（推荐）
  bool pure_rot_only_after_zupt_fail = true;

  /// 纯旋转：滤波器估计线速度范数上限 (m/s)
  double pure_rot_max_velocity = 0.15;

  /// 纯旋转：IMU 窗口内陀螺幅值均值下限/上限 (rad/s)，用于与「静止」区分
  double pure_rot_gyro_min = 0.05;
  double pure_rot_gyro_max = 3.0;

  /// 纯旋转：线速度 v≈0 伪观测标准差 (m/s)
  double pure_rot_vel_sigma = 0.1;

  /// 纯旋转：测量噪声 R 标量倍率（与 ZUPT 的 zupt_noise_multiplier 独立）
  double pure_rot_noise_multiplier = 1.0;

  /// 是否启用图像门：光流与相对主点径向垂直度（绕光轴转）
  bool pure_rot_use_image_gate = true;

  /// 光流垂直置信：mean(1-|cos∠(r,f)|) 下限 [0,1]
  double pure_rot_flow_perp_min = 0.55;

  /// r×f 符号与多数一致的比例下限 [0,1]（向量叉积保证绕光轴转向一致）
  double pure_rot_flow_sign_consistency_min = 0.75;

  /// 参与上述统计的最少特征段数
  int pure_rot_min_flow_features = 12;

  /// 相对主点半径小于此像素则跳过（避免光心附近不可靠）
  double pure_rot_flow_min_r_px = 80.0;

  /// 位移模长小于此像素则跳过（噪声）
  double pure_rot_flow_min_mag_px = 0.4;

  /// 是否启用 IMU 门：角速度在 ref 相机系下光轴分量 |ωz|/|ω|
  bool pure_rot_use_cam_z_gyro_gate = true;

  /// 光轴角速度占比下限 [0,1]
  double pure_rot_min_omega_cam_z_ratio = 0.65;

  /// 上述陀螺门使用的参考相机 ID（与 T_imu_cam 一致）
  int pure_rot_ref_cam_id = 0;

  /// 是否打印 [PureRot] 门限与成功位姿
  bool print_pure_rot = false;

  /// 每次 IMU 传播后对速度误差协方差对角增加 (m^2/s^2)/轴；0=关闭。略增大可降低对纯 IMU 积分速度的置信度，使 MSCKF/SLAM 更易校正速度、减轻位置漂移
  double post_propagate_vel_cov_diag_add = 0.0;

  /// 是否将时间性能记录到文件
  /// If we should record the timing performance to file
  bool record_timing_information = false;

  /// 记录时间信息的文件路径
  /// The path to the file we will record the timing information into
  std::string record_timing_filepath = "ov_msckf_timing.txt";

  /// 是否将输入的 IMU / 相机数据记录为数据集（imu.csv、camera_index.csv、mosaic PNG）
  /// If true, log incoming IMU and camera measurements (see VioDataRecorder)
  bool record_vio_dataset = false;

  /// 为 true 时仅入队写盘，不执行 SLAM/跟踪/传播（须同时 record_vio_dataset）
  bool record_vio_dataset_record_only = false;

  /// 数据集记录根目录（须为绝对路径）；其下每次运行会再建子目录（见 record_vio_dataset_run_path）
  std::string record_vio_dataset_root;

  /// 本次运行实际写入目录（VioManager 构造时生成：根目录 / yyyymmddhhmmss，冲突则秒 +1）
  std::string record_vio_dataset_run_path;

  /// 大图水平方向格子数（默认 15，与 VioDataRecorder 一致）
  int record_vio_mosaic_cols = 1;

  /// 竖直条带行数（默认 15，即一包 15 张图）
  int record_vio_mosaic_rows = 15;

  /// 是否在每次更新后打印状态与标定信息（调试用）
  /// If we should print state and calibration info after each update (for debugging)
  bool print_state_calib = false;

  /// 是否打印各阶段耗时（跟踪、传播、MSCKF/SLAM 更新、边缘化等）
  /// If we should print per-stage timing (track, prop, msckf/slam update, marg, etc.)
  bool print_timing = false;

  /// 是否打印跟踪统计（[VM] 帧 #N 与 [跟踪] 更新 #N 的周期日志）
  /// If we should print tracking stats (periodic [VM] frame and [跟踪] update logs)
  bool print_tracking_stats = true;

  /// 是否在每次 retriangulate 后打印线性三角化得到的特征点全局坐标 p_FinG（米；含 MSCKF 活动轨迹与 SLAM 路标）
  /// If true, print global-frame triangulated feature positions after each retriangulate_active_tracks
  bool print_triangulated_points = false;

  /**
   * @brief 加载并打印所有估计器设置
   * 这允许直观检查是否从ROS/CMD解析器正确加载了所有内容。
   *
   * @param parser 如果不为空，将使用此解析器加载参数
   * 
   * @brief This function will load print out all estimator settings loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_estimator(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    PRINT_DEBUG("估计器参数:\n");
    state_options.print(parser);
    init_options.print_and_load(parser);
    if (parser != nullptr) {
      parser->parse_config("dt_slam_delay", dt_slam_delay);
      parser->parse_config("try_zupt", try_zupt);
      // 可选 legacy：仅写 zupt_max_velocity 等时，先赋给两路，再由 zupt_agree_* / zupt_strict_* 或旧键 zupt_or_* / zupt_and_* 覆盖
      parser->parse_config("zupt_max_velocity", zupt_agree_max_velocity, false);
      zupt_strict_max_velocity = zupt_agree_max_velocity;
      parser->parse_config("zupt_noise_multiplier", zupt_noise_multiplier, false);
      parser->parse_config("zupt_or_noise_multiplier", zupt_noise_multiplier,
                           false); // 废弃别名，与 zupt_noise_multiplier 同义
      parser->parse_config("zupt_max_disparity", zupt_agree_max_disparity, false);
      zupt_strict_max_disparity = zupt_agree_max_disparity;
      parser->parse_config("zupt_agree_max_velocity", zupt_agree_max_velocity, false);
      parser->parse_config("zupt_agree_max_disparity", zupt_agree_max_disparity, false);
      parser->parse_config("zupt_strict_max_velocity", zupt_strict_max_velocity, false);
      parser->parse_config("zupt_strict_max_disparity", zupt_strict_max_disparity, false);
      parser->parse_config("zupt_or_max_velocity", zupt_agree_max_velocity, false);
      parser->parse_config("zupt_or_max_disparity", zupt_agree_max_disparity, false);
      parser->parse_config("zupt_and_max_velocity", zupt_strict_max_velocity, false);
      parser->parse_config("zupt_and_max_disparity", zupt_strict_max_disparity, false);
      parser->parse_config("zupt_only_at_beginning", zupt_only_at_beginning);
      parser->parse_config("zupt_exit_consecutive_failures",
                           zupt_exit_consecutive_failures, false);
      parser->parse_config("zupt_exit_cov_inflation", zupt_exit_cov_inflation,
                           false);
      parser->parse_config("record_timing_information", record_timing_information);
      parser->parse_config("record_timing_filepath", record_timing_filepath);
      parser->parse_config("record_vio_dataset", record_vio_dataset, false);
      parser->parse_config("record_vio_dataset_record_only", record_vio_dataset_record_only, false);
      parser->parse_config("record_vio_dataset_root", record_vio_dataset_root, false);
      {
        std::string legacy_path;
        parser->parse_config("record_vio_dataset_path", legacy_path, false);
        if (!legacy_path.empty())
          record_vio_dataset_root = legacy_path;
      }
      parser->parse_config("record_vio_mosaic_cols", record_vio_mosaic_cols, false);
      parser->parse_config("record_vio_mosaic_rows", record_vio_mosaic_rows, false);
      if (record_vio_dataset_record_only && !record_vio_dataset) {
        PRINT_ERROR(RED "record_vio_dataset_record_only 为 true 时必须启用 record_vio_dataset。\n" RESET);
        std::exit(EXIT_FAILURE);
      }
      if (record_vio_dataset) {
        const std::filesystem::path rd_path(record_vio_dataset_root);
        if (record_vio_dataset_root.empty() || !rd_path.is_absolute()) {
          PRINT_ERROR(RED "启用 record_vio_dataset 时，record_vio_dataset_root 必须为非空绝对路径。\n" RESET);
          std::exit(EXIT_FAILURE);
        }
      }
      parser->parse_config("print_state_calib", print_state_calib);
      parser->parse_config("print_timing", print_timing);
      parser->parse_config("print_tracking_stats", print_tracking_stats);
      parser->parse_config("print_triangulated_points", print_triangulated_points,
                           false);
      parser->parse_config("print_zupt", print_zupt, false);
      parser->parse_config("try_pure_rot", try_pure_rot, false);
      parser->parse_config("pure_rot_only_after_zupt_fail",
                           pure_rot_only_after_zupt_fail, false);
      parser->parse_config("pure_rot_max_velocity", pure_rot_max_velocity,
                           false);
      parser->parse_config("pure_rot_gyro_min", pure_rot_gyro_min, false);
      parser->parse_config("pure_rot_gyro_max", pure_rot_gyro_max, false);
      parser->parse_config("pure_rot_vel_sigma", pure_rot_vel_sigma, false);
      parser->parse_config("pure_rot_noise_multiplier",
                           pure_rot_noise_multiplier, false);
      parser->parse_config("pure_rot_use_image_gate", pure_rot_use_image_gate,
                           false);
      parser->parse_config("pure_rot_flow_perp_min", pure_rot_flow_perp_min,
                           false);
      parser->parse_config("pure_rot_flow_sign_consistency_min",
                           pure_rot_flow_sign_consistency_min, false);
      parser->parse_config("pure_rot_min_flow_features",
                           pure_rot_min_flow_features, false);
      parser->parse_config("pure_rot_flow_min_r_px", pure_rot_flow_min_r_px,
                           false);
      parser->parse_config("pure_rot_flow_min_mag_px",
                           pure_rot_flow_min_mag_px, false);
      parser->parse_config("pure_rot_use_cam_z_gyro_gate",
                           pure_rot_use_cam_z_gyro_gate, false);
      parser->parse_config("pure_rot_min_omega_cam_z_ratio",
                           pure_rot_min_omega_cam_z_ratio, false);
      parser->parse_config("pure_rot_ref_cam_id", pure_rot_ref_cam_id, false);
      parser->parse_config("print_pure_rot", print_pure_rot, false);
      parser->parse_config("post_propagate_vel_cov_diag_add",
                           post_propagate_vel_cov_diag_add, false);
    }
    PRINT_DEBUG("  - SLAM延迟时间: %.1f\n", dt_slam_delay);
    if (print_zupt) {
      PRINT_DEBUG("  - 零速度更新 try_zupt: %d\n", try_zupt);
      PRINT_DEBUG("  - 仅在开始时使用零速度更新 zupt_only_at_beginning: %d\n",
                  zupt_only_at_beginning);
    }
    if (zupt_exit_consecutive_failures < 1) {
      PRINT_WARNING(
          YELLOW
          "  - zupt_exit_consecutive_failures=%d 非法，回退为 1（立即退出）\n"
          RESET,
          zupt_exit_consecutive_failures);
      zupt_exit_consecutive_failures = 1;
    }
    if (print_zupt) {
      PRINT_DEBUG("  - 延迟退出ZUPT 连续离开回合数 n: %d（1=立即退出）\n",
                  zupt_exit_consecutive_failures);
    }
    if (zupt_exit_cov_inflation < 1.0) {
      PRINT_WARNING(YELLOW
                    "  - zupt_exit_cov_inflation=%.3f 非法，回退为 1.0（不倍化）\n"
                    RESET,
                    zupt_exit_cov_inflation);
      zupt_exit_cov_inflation = 1.0;
    }
    if (print_zupt) {
      PRINT_DEBUG("  - 退出ZUPT时 IMU 协方差倍化系数: %.3f（1.0=关闭）\n",
                  zupt_exit_cov_inflation);
      PRINT_DEBUG("  - ZUPT 静止门 = 赞同阈值支路 || 否决阈值支路（χ² 倍率见「噪声参数」段）\n");
      PRINT_DEBUG("    赞同阈值（zupt_agree_*）：|v|≤%.3f m/s  disp≤%.4f px\n",
                  zupt_agree_max_velocity, zupt_agree_max_disparity);
      PRINT_DEBUG("    否决阈值（zupt_strict_*）：|v|≤%.3f m/s  disp≤%.4f px\n",
                  zupt_strict_max_velocity, zupt_strict_max_disparity);
      PRINT_DEBUG("    ZUPT 测量噪声 R 标量倍率（χ² 与 EKF 共用）: %.3f\n",
                  zupt_noise_multiplier);
    }
    PRINT_DEBUG("  - 记录时间信息?: %d\n", (int)record_timing_information);
    PRINT_DEBUG("  - 记录时间信息文件路径: %s\n", record_timing_filepath.c_str());
    PRINT_DEBUG("  - 记录VIO数据集?: %d\n", (int)record_vio_dataset);
    PRINT_DEBUG("  - 记录VIO仅记录不跑SLAM?: %d\n", (int)record_vio_dataset_record_only);
    PRINT_DEBUG("  - 记录VIO数据集根目录: %s\n", record_vio_dataset_root.c_str());
    if (!record_vio_dataset_run_path.empty())
      PRINT_DEBUG("  - 记录VIO本次运行目录: %s\n", record_vio_dataset_run_path.c_str());
    PRINT_DEBUG("  - 记录VIO mosaic 列x行: %d x %d\n", record_vio_mosaic_cols, record_vio_mosaic_rows);
    PRINT_DEBUG("  - 打印状态与标定?: %d\n", (int)print_state_calib);
    PRINT_DEBUG("  - 打印各阶段耗时?: %d\n", (int)print_timing);
    PRINT_DEBUG("  - 打印跟踪统计([VM]帧/[跟踪]更新)?: %d\n", (int)print_tracking_stats);
    PRINT_DEBUG("  - 打印三角化点云 p_FinG?: %d\n", (int)print_triangulated_points);
    PRINT_DEBUG("  - 打印 ZUPT 日志?: %d\n", (int)print_zupt);
    PRINT_DEBUG("  - 纯旋转更新 try_pure_rot: %d（仅 ZUPT 失败后: %d）\n",
                (int)try_pure_rot, (int)pure_rot_only_after_zupt_fail);
    PRINT_DEBUG("  - 传播后速度协方差对角膨胀: %.6f (m^2/s^2)/轴（0=关）\n",
                post_propagate_vel_cov_diag_add);
  }

  // NOISE / CHI2 ============================
  // 噪声/卡方检验参数 ============================

  /// 连续时间IMU噪声（陀螺仪和加速度计）
  /// Continuous-time IMU noise (gyroscope and accelerometer)
  NoiseManager imu_noises;

  /// MSCKF特征点的更新选项（像素噪声和chi2乘数）
  /// Update options for MSCKF features (pixel noise and chi2 multiplier)
  UpdaterOptions msckf_options;

  /// SLAM特征点的更新选项（像素噪声和chi2乘数）
  /// Update options for SLAM features (pixel noise and chi2 multiplier)
  UpdaterOptions slam_options;

  /// ARUCO特征点的更新选项（像素噪声和chi2乘数）
  /// Update options for ARUCO features (pixel noise and chi2 multiplier)
  UpdaterOptions aruco_options;

  /// 零速度更新的选项（chi2乘数）
  /// Update options for zero velocity (chi2 multiplier)
  UpdaterOptions zupt_options;

  /// 纯旋转更新的 χ² 倍率（见噪声段 pure_rot_chi2_multipler）
  UpdaterOptions pure_rot_options;

  /// 静止门恒为 gate_agree||gate_strict；将辅助量写回 zupt_*（须在解析完 chi² 后调用）
  void apply_zupt_active_branch() {
    zupt_max_velocity =
        std::min(zupt_agree_max_velocity, zupt_strict_max_velocity);
    zupt_max_disparity =
        std::min(zupt_agree_max_disparity, zupt_strict_max_disparity);
    zupt_options.chi2_multipler = zupt_agree_chi2_multipler;
  }

  /**
   * @brief 加载并打印所有噪声参数
   * 这允许直观检查是否从ROS/CMD解析器正确加载了所有内容。
   *
   * @param parser 如果不为空，将使用此解析器加载参数
   * 
   * @brief This function will load print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_noise(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    PRINT_DEBUG("噪声参数:\n");
    if (parser != nullptr) {
      parser->parse_external("relative_config_imu", "imu0", "gyroscope_noise_density", imu_noises.sigma_w);
      parser->parse_external("relative_config_imu", "imu0", "gyroscope_random_walk", imu_noises.sigma_wb);
      parser->parse_external("relative_config_imu", "imu0", "accelerometer_noise_density", imu_noises.sigma_a);
      parser->parse_external("relative_config_imu", "imu0", "accelerometer_random_walk", imu_noises.sigma_ab);
    }
    imu_noises.print();
    if (parser != nullptr) {
      parser->parse_config("up_msckf_sigma_px", msckf_options.sigma_pix);
      parser->parse_config("up_msckf_chi2_multipler", msckf_options.chi2_multipler);
      parser->parse_config("up_slam_sigma_px", slam_options.sigma_pix);
      parser->parse_config("up_slam_chi2_multipler", slam_options.chi2_multipler);
      parser->parse_config("up_aruco_sigma_px", aruco_options.sigma_pix);
      parser->parse_config("up_aruco_chi2_multipler", aruco_options.chi2_multipler);
      msckf_options.sigma_pix_sq = std::pow(msckf_options.sigma_pix, 2);
      slam_options.sigma_pix_sq = std::pow(slam_options.sigma_pix, 2);
      aruco_options.sigma_pix_sq = std::pow(aruco_options.sigma_pix, 2);
      // 可选 legacy zupt_chi2_multipler：同时赋两路；再由 zupt_agree_* / zupt_strict_* 或旧键 zupt_or_* / zupt_and_* 覆盖
      parser->parse_config("zupt_chi2_multipler", zupt_agree_chi2_multipler, false);
      zupt_strict_chi2_multipler = zupt_agree_chi2_multipler;
      parser->parse_config("zupt_agree_chi2_multipler", zupt_agree_chi2_multipler,
                           false);
      parser->parse_config("zupt_strict_chi2_multipler", zupt_strict_chi2_multipler,
                           false);
      parser->parse_config("zupt_or_chi2_multipler", zupt_agree_chi2_multipler,
                           false);
      parser->parse_config("zupt_and_chi2_multipler", zupt_strict_chi2_multipler,
                           false);
      parser->parse_config("pure_rot_chi2_multipler",
                           pure_rot_options.chi2_multipler, false);
    } else {
      zupt_agree_chi2_multipler = zupt_options.chi2_multipler;
      zupt_strict_chi2_multipler = zupt_options.chi2_multipler;
    }
    apply_zupt_active_branch();
    PRINT_DEBUG("  MSCKF特征更新器:\n");
    msckf_options.print();
    PRINT_DEBUG("  SLAM特征更新器:\n");
    slam_options.print();
    PRINT_DEBUG("  ARUCO标签更新器:\n");
    aruco_options.print();
    if (print_pure_rot && try_pure_rot) {
      PRINT_DEBUG("  纯旋转更新器 (PureRot):\n");
      PRINT_DEBUG("    - chi2_mult: %.4f\n", pure_rot_options.chi2_multipler);
      PRINT_DEBUG("    - |v|≤%.4f m/s  |ω|∈[%.4f, %.4f] rad/s  v_sigma=%.4f m/s  R倍率=%.4f\n",
                  pure_rot_max_velocity, pure_rot_gyro_min, pure_rot_gyro_max,
                  pure_rot_vel_sigma, pure_rot_noise_multiplier);
      PRINT_DEBUG("    - 图像：perp_min=%.3f  sign_consistency_min=%.3f\n",
                  pure_rot_flow_perp_min, pure_rot_flow_sign_consistency_min);
    }
    if (print_zupt) {
      PRINT_DEBUG("  零速度更新器 (ZUPT):\n");
      PRINT_DEBUG("    - 门限：chi2 ≤ mult × χ²_0.95(残差维)；赞同/否决各用各自 mult 得到 gate_agree / gate_strict\n");
      PRINT_DEBUG("    - 马氏 χ² 中 S 与 EKF 更新 R 共用 zupt_noise_multiplier\n");
      PRINT_DEBUG("    - 赞同 chi2_mult: %.4f\n", zupt_agree_chi2_multipler);
      PRINT_DEBUG("    - 否决 chi2_mult: %.4f\n", zupt_strict_chi2_multipler);
      PRINT_DEBUG("    - EKF 测量噪声 R 与 χ² 中 R 相同（gate_agree||gate_strict 为真才进入更新）\n");
      PRINT_DEBUG("    - 兼容字段：zupt_options.chi2_multipler=%.4f（=赞同，供内部一致性）\n",
                  zupt_options.chi2_multipler);
      PRINT_DEBUG("    - 辅助量：zupt_max_velocity=%.3f (=min(赞同,否决)，如 has_moved)；"
                  "zupt_max_disparity=%.4f (=min(赞同,否决))；"
                  "zupt_noise_multiplier=%.3f\n",
                  zupt_max_velocity, zupt_max_disparity, zupt_noise_multiplier);
    }
  }

  // STATE DEFAULTS ==========================
  // 状态默认值 ==========================

  /// 全局坐标系中的重力大小（通常应为9.81）
  /// Gravity magnitude in the global frame (i.e. should be 9.81 typically)
  double gravity_mag = 9.81;

  /// 陀螺仪IMU内参（标度不完美和轴不对齐，按列，逆矩阵）
  /// Gyroscope IMU intrinsics (scale imperfection and axis misalignment, column-wise, inverse)
  Eigen::Matrix<double, 6, 1> vec_dw;

  /// 加速度计IMU内参（标度不完美和轴不对齐，按列，逆矩阵）
  /// Accelerometer IMU intrinsics (scale imperfection and axis misalignment, column-wise, inverse)
  Eigen::Matrix<double, 6, 1> vec_da;

  /// 陀螺仪重力敏感度（标度不完美和轴不对齐，按列）
  /// Gyroscope gravity sensitivity (scale imperfection and axis misalignment, column-wise)
  Eigen::Matrix<double, 9, 1> vec_tg;

  /// 从陀螺仪坐标系到"IMU"加速度计坐标系的旋转
  /// Rotation from gyroscope frame to the "IMU" accelerometer frame
  Eigen::Matrix<double, 4, 1> q_ACCtoIMU;

  /// 从加速度计到"IMU"陀螺仪坐标系的旋转
  /// Rotation from accelerometer to the "IMU" gyroscope frame frame
  Eigen::Matrix<double, 4, 1> q_GYROtoIMU;

  /// 相机和IMU之间的时间偏移
  /// Time offset between camera and IMU.
  double calib_camimu_dt = 0.0;

  /// 相机ID到相机内参的映射（fx, fy, cx, cy, d1...d4, cam_w, cam_h）
  /// Map between camid and camera intrinsics (fx, fy, cx, cy, d1...d4, cam_w, cam_h)
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> camera_intrinsics;

  /// 相机ID到相机外参的映射（q_ItoC, p_IinC）
  /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
  std::map<size_t, Eigen::VectorXd> camera_extrinsics;

  /// 是否尝试加载掩码并使用它来拒绝无效特征点
  /// If we should try to load a mask and use it to reject invalid features
  bool use_mask = false;

  /// 每个相机的掩码图像
  /// Mask images for each camera
  std::map<size_t, cv::Mat> masks;

  /**
   * @brief 加载并打印所有状态参数（例如传感器外参）
   * 这允许直观检查是否从ROS/CMD解析器正确加载了所有内容。
   *
   * @param parser 如果不为空，将使用此解析器加载参数
   * 
   * @brief This function will load and print all state parameters (e.g. sensor extrinsics)
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_state(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("gravity_mag", gravity_mag);
      for (int i = 0; i < state_options.num_cameras; i++) {

        // Time offset (use the first one)
        // TODO: support multiple time offsets between cameras
        if (i == 0) {
          parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "timeshift_cam_imu", calib_camimu_dt, false);
        }

        // Distortion model
        std::string dist_model = "radtan";
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "distortion_model", dist_model);

        // Distortion parameters
        std::vector<double> cam_calib1 = {1, 1, 0, 0};
        std::vector<double> cam_calib2 = {0, 0, 0, 0};
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "intrinsics", cam_calib1);
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "distortion_coeffs", cam_calib2);
        Eigen::VectorXd cam_calib = Eigen::VectorXd::Zero(8);
        cam_calib << cam_calib1.at(0), cam_calib1.at(1), cam_calib1.at(2), cam_calib1.at(3), cam_calib2.at(0), cam_calib2.at(1),
            cam_calib2.at(2), cam_calib2.at(3);
        cam_calib(0) /= (downsample_cameras) ? 2.0 : 1.0;
        cam_calib(1) /= (downsample_cameras) ? 2.0 : 1.0;
        cam_calib(2) /= (downsample_cameras) ? 2.0 : 1.0;
        cam_calib(3) /= (downsample_cameras) ? 2.0 : 1.0;

        // FOV / resolution
        std::vector<int> matrix_wh = {1, 1};
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "resolution", matrix_wh);
        matrix_wh.at(0) /= (downsample_cameras) ? 2.0 : 1.0;
        matrix_wh.at(1) /= (downsample_cameras) ? 2.0 : 1.0;

        // Extrinsics
        Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
        parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "T_imu_cam", T_CtoI);

        // Load these into our state
        Eigen::Matrix<double, 7, 1> cam_eigen;
        cam_eigen.block(0, 0, 4, 1) = ov_core::rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
        cam_eigen.block(4, 0, 3, 1) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);

        // Create intrinsics model
        if (dist_model == "equidistant") {
          camera_intrinsics.insert({i, std::make_shared<ov_core::CamEqui>(matrix_wh.at(0), matrix_wh.at(1))});
          camera_intrinsics.at(i)->set_value(cam_calib);
        } else {
          camera_intrinsics.insert({i, std::make_shared<ov_core::CamRadtan>(matrix_wh.at(0), matrix_wh.at(1))});
          camera_intrinsics.at(i)->set_value(cam_calib);
        }
        camera_extrinsics.insert({i, cam_eigen});
      }
      parser->parse_config("use_mask", use_mask);
      if (use_mask) {
        for (int i = 0; i < state_options.num_cameras; i++) {
          std::string mask_path;
          std::string mask_node = "mask" + std::to_string(i);
          parser->parse_config(mask_node, mask_path);
          std::string total_mask_path = parser->get_config_folder() + mask_path;
          if (!std::filesystem::exists(total_mask_path)) {
            PRINT_ERROR(RED "VioManager(): 无效的掩码路径:\n" RESET);
            PRINT_ERROR(RED "\t- 掩码%d - %s\n" RESET, i, total_mask_path.c_str());
            std::exit(EXIT_FAILURE);
          }
          cv::Mat mask = cv::imread(total_mask_path, cv::IMREAD_GRAYSCALE);
          masks.insert({i, mask});
          if (mask.cols != camera_intrinsics.at(i)->w() || mask.rows != camera_intrinsics.at(i)->h()) {
            PRINT_ERROR(RED "VioManager(): 掩码尺寸与相机不匹配！\n" RESET);
            PRINT_ERROR(RED "\t- 掩码%d - %s\n" RESET, i, total_mask_path.c_str());
            PRINT_ERROR(RED "\t- 掩码%d - %d x %d\n" RESET, mask.cols, mask.rows);
            PRINT_ERROR(RED "\t- 相机%d - %d x %d\n" RESET, camera_intrinsics.at(i)->w(), camera_intrinsics.at(i)->h());
            std::exit(EXIT_FAILURE);
          }
        }
      }

      // IMU intrinsics
      Eigen::Matrix3d Tw = Eigen::Matrix3d::Identity();
      parser->parse_external("relative_config_imu", "imu0", "Tw", Tw);
      Eigen::Matrix3d Ta = Eigen::Matrix3d::Identity();
      parser->parse_external("relative_config_imu", "imu0", "Ta", Ta);
      Eigen::Matrix3d R_IMUtoACC = Eigen::Matrix3d::Identity();
      parser->parse_external("relative_config_imu", "imu0", "R_IMUtoACC", R_IMUtoACC);
      Eigen::Matrix3d R_IMUtoGYRO = Eigen::Matrix3d::Identity();
      parser->parse_external("relative_config_imu", "imu0", "R_IMUtoGYRO", R_IMUtoGYRO);
      Eigen::Matrix3d Tg = Eigen::Matrix3d::Zero();
      parser->parse_external("relative_config_imu", "imu0", "Tg", Tg);

      // Generate the parameters we need
      // TODO: error here if this returns a NaN value (i.e. invalid matrix specified)
      Eigen::Matrix3d Dw = Tw.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
      Eigen::Matrix3d Da = Ta.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
      Eigen::Matrix3d R_ACCtoIMU = R_IMUtoACC.transpose();
      Eigen::Matrix3d R_GYROtoIMU = R_IMUtoGYRO.transpose();
      if (std::isnan(Tw.norm()) || std::isnan(Dw.norm())) {
        std::stringstream ss;
        ss << "陀螺仪内参值无效！" << std::endl;
        ss << "Tw - " << std::endl << Tw << std::endl << std::endl;
        ss << "Dw - " << std::endl << Dw << std::endl << std::endl;
        PRINT_DEBUG(RED "" RESET, ss.str().c_str());
        std::exit(EXIT_FAILURE);
      }
      if (std::isnan(Ta.norm()) || std::isnan(Da.norm())) {
        std::stringstream ss;
        ss << "加速度计内参值无效！" << std::endl;
        ss << "Ta - " << std::endl << Ta << std::endl << std::endl;
        ss << "Da - " << std::endl << Da << std::endl << std::endl;
        PRINT_DEBUG(RED "" RESET, ss.str().c_str());
        std::exit(EXIT_FAILURE);
      }

      // kalibr model: lower triangular of the matrix and R_GYROtoI
      // rpng model: upper triangular of the matrix and R_ACCtoI
      if (state_options.imu_model == StateOptions::ImuModel::KALIBR) {
        vec_dw << Dw.block<3, 1>(0, 0), Dw.block<2, 1>(1, 1), Dw(2, 2);
        vec_da << Da.block<3, 1>(0, 0), Da.block<2, 1>(1, 1), Da(2, 2);
      } else {
        vec_dw << Dw(0, 0), Dw.block<2, 1>(0, 1), Dw.block<3, 1>(0, 2);
        vec_da << Da(0, 0), Da.block<2, 1>(0, 1), Da.block<3, 1>(0, 2);
      }
      vec_tg << Tg.block<3, 1>(0, 0), Tg.block<3, 1>(0, 1), Tg.block<3, 1>(0, 2);
      q_GYROtoIMU = ov_core::rot_2_quat(R_GYROtoIMU);
      q_ACCtoIMU = ov_core::rot_2_quat(R_ACCtoIMU);
    }
    PRINT_DEBUG("状态参数:\n");
    PRINT_DEBUG("  - 重力大小: %.4f\n", gravity_mag);
    PRINT_DEBUG("  - 重力: %.3f, %.3f, %.3f\n", 0.0, 0.0, gravity_mag);
    PRINT_DEBUG("  - 使用相机掩码?: %d\n", use_mask);
    if (state_options.num_cameras != (int)camera_intrinsics.size() || state_options.num_cameras != (int)camera_extrinsics.size()) {
      PRINT_ERROR(RED "[仿真]: 相机标定数量与最大相机数不匹配...\n" RESET);
      PRINT_ERROR(RED "[仿真]: 相机内参得到 %d 个，但期望 %d 个最大相机数\n" RESET, (int)camera_intrinsics.size(),
                  state_options.num_cameras);
      PRINT_ERROR(RED "[仿真]: 相机外参得到 %d 个，但期望 %d 个最大相机数\n" RESET, (int)camera_extrinsics.size(),
                  state_options.num_cameras);
      std::exit(EXIT_FAILURE);
    }
    PRINT_DEBUG("  - 相机-IMU时间偏移: %.4f\n", calib_camimu_dt);
    PRINT_DEBUG("相机参数:\n");
    for (int n = 0; n < state_options.num_cameras; n++) {
      std::stringstream ss;
      ss << "cam_" << n << "_fisheye:" << (std::dynamic_pointer_cast<ov_core::CamEqui>(camera_intrinsics.at(n)) != nullptr) << std::endl;
      ss << "cam_" << n << "_wh:" << std::endl << camera_intrinsics.at(n)->w() << " x " << camera_intrinsics.at(n)->h() << std::endl;
      ss << "cam_" << n << "_intrinsic(0:3):" << std::endl
         << camera_intrinsics.at(n)->get_value().block(0, 0, 4, 1).transpose() << std::endl;
      ss << "cam_" << n << "_intrinsic(4:7):" << std::endl
         << camera_intrinsics.at(n)->get_value().block(4, 0, 4, 1).transpose() << std::endl;
      ss << "cam_" << n << "_extrinsic(0:3):" << std::endl << camera_extrinsics.at(n).block(0, 0, 4, 1).transpose() << std::endl;
      ss << "cam_" << n << "_extrinsic(4:6):" << std::endl << camera_extrinsics.at(n).block(4, 0, 3, 1).transpose() << std::endl;
      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
      T_CtoI.block(0, 0, 3, 3) = ov_core::quat_2_Rot(camera_extrinsics.at(n).block(0, 0, 4, 1)).transpose();
      T_CtoI.block(0, 3, 3, 1) = -T_CtoI.block(0, 0, 3, 3) * camera_extrinsics.at(n).block(4, 0, 3, 1);
      ss << "T_C" << n << "toI:" << std::endl << T_CtoI << std::endl << std::endl;
      PRINT_DEBUG(ss.str().c_str());
    }
    PRINT_DEBUG("IMU参数:\n");
    std::stringstream ss;
    ss << "imu model:" << ((state_options.imu_model == StateOptions::ImuModel::KALIBR) ? "kalibr" : "rpng") << std::endl;
    ss << "Dw (columnwise):" << vec_dw.transpose() << std::endl;
    ss << "Da (columnwise):" << vec_da.transpose() << std::endl;
    ss << "Tg (columnwise):" << vec_tg.transpose() << std::endl;
    ss << "q_GYROtoI: " << q_GYROtoIMU.transpose() << std::endl;
    ss << "q_ACCtoI: " << q_ACCtoIMU.transpose() << std::endl;
    PRINT_DEBUG(ss.str().c_str());
  }

  // TRACKERS ===============================
  // 跟踪器参数 ===============================

  /// 是否将两个相机处理为双目或双相机。如果是双相机，我们在每个图像上进行单目特征跟踪。
  /// If we should process two cameras are being stereo or binocular. If binocular, we do monocular feature tracking on each image.
  bool use_stereo = true;

  /// 是否使用KLT跟踪或描述符匹配器
  /// If we should use KLT tracking, or descriptor matcher
  bool use_klt = true;

  /// 是否提取aruco标签并估计它们
  /// If should extract aruco tags and estimate them
  bool use_aruco = true;

  /// 是否将aruco标签图像的分辨率减半（会更快）
  /// Will half the resolution of the aruco tag image (will be faster)
  bool downsize_aruco = true;

  /// 是否将所有跟踪图像的分辨率减半（如果同时启用downsize_aruco，aruco将是1/4而不是减半）
  /// Will half the resolution all tracking image (aruco will be 1/4 instead of halved if dowsize_aruoc also enabled)
  bool downsample_cameras = false;

  /// 前端应尝试使用的线程数（opencv也使用此值）
  /// Threads our front-end should try to use (opencv uses this also)
  int num_opencv_threads = 4;

  /// ROS图像发布器是否应为异步（如果是仿真则应为否！）
  /// If our ROS image publisher should be async (if sim this should be no!)
  bool use_multi_threading_pubs = true;

  /// ROS订阅器回调是否应为异步（如果是仿真和串行则应为否！）
  /// If our ROS subscriber callbacks should be async (if sim and serial then this should be no!)
  bool use_multi_threading_subs = false;

  /// 在*每个*图像帧中应提取和跟踪的点数。这极大地影响跟踪所需的计算量。
  /// The number of points we should extract and track in *each* image frame. This highly effects the computation required for tracking.
  int num_pts = 150;

  /// FAST特征提取阈值
  /// Fast extraction threshold
  int fast_threshold = 20;

  /// 列方向分割的网格数量，用于特征提取
  /// Number of grids we should split column-wise to do feature extraction in
  int grid_x = 5;

  /// 行方向分割的网格数量，用于特征提取
  /// Number of grids we should split row-wise to do feature extraction in
  int grid_y = 5;

  /// 在执行KLT跟踪后检查并移除距离小于此值的特征点
  /// Will check after doing KLT track and remove any features closer than this
  int min_px_dist = 10;

  /// 应用于图像的预处理直方图方法类型
  /// What type of pre-processing histogram method should be applied to images
  ov_core::TrackBase::HistogramMethod histogram_method = ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  /// 描述符匹配器中前两个匹配的KNN比率，需要此比率才能成为良好匹配
  /// KNN ration between top two descriptor matcher which is required to be a good match
  double knn_ratio = 0.85;

  /// 我们希望跟踪图像的频率（更高频率的图像将被丢弃）
  /// Frequency we want to track images at (higher freq ones will be dropped)
  double track_frequency = 20.0;

  /// 调试显示窗口 waitKey 等待时长（毫秒，0=不等待/仅处理事件）
  int debug_display_waitkey_ms = 1;

  /// 是否启用 IMU 先验来初始化 KLT 光流（仅前端初值，不改变后端模型）
  bool use_imu_klt_prior = false;

  /// 使用 IMU 先验的最小帧间隔（秒）
  double imu_klt_prior_min_dt = 0.002;

  /// 使用 IMU 先验的最大帧间隔（秒）
  double imu_klt_prior_max_dt = 0.08;

  /// 打印 IMU-KLT 先验调试信息
  bool imu_klt_prior_debug = false;

  /// 打印 KLT 几何校验 [KLT-GEOM]（H/F RANSAC 与包络收缩统计，默认关闭）
  bool print_klt_geom = false;

  /// 特征初始化/三角化器使用的参数
  /// Parameters used by our feature initialize / triangulator
  ov_core::FeatureInitializerOptions featinit_options;

  /**
   * @brief 加载并打印所有与视觉跟踪相关的参数
   * 这允许直观检查是否从ROS/CMD解析器正确加载了所有内容。
   *
   * @param parser 如果不为空，将使用此解析器加载参数
   * 
   * @brief This function will load print out all parameters related to visual tracking
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_trackers(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("use_stereo", use_stereo);
      parser->parse_config("use_klt", use_klt);
      parser->parse_config("use_aruco", use_aruco);
      parser->parse_config("downsize_aruco", downsize_aruco);
      parser->parse_config("downsample_cameras", downsample_cameras);
      parser->parse_config("num_opencv_threads", num_opencv_threads);
      parser->parse_config("multi_threading_pubs", use_multi_threading_pubs, false);
      parser->parse_config("multi_threading_subs", use_multi_threading_subs, false);
      parser->parse_config("num_pts", num_pts);
      parser->parse_config("fast_threshold", fast_threshold);
      parser->parse_config("grid_x", grid_x);
      parser->parse_config("grid_y", grid_y);
      parser->parse_config("min_px_dist", min_px_dist);
      std::string histogram_method_str = "HISTOGRAM";
      parser->parse_config("histogram_method", histogram_method_str);
      if (histogram_method_str == "NONE") {
        histogram_method = ov_core::TrackBase::NONE;
      } else if (histogram_method_str == "HISTOGRAM") {
        histogram_method = ov_core::TrackBase::HISTOGRAM;
      } else if (histogram_method_str == "CLAHE") {
        histogram_method = ov_core::TrackBase::CLAHE;
      } else {
        printf(RED "VioManager(): 指定的特征直方图方法无效:\n" RESET);
        printf(RED "\t- NONE\n" RESET);
        printf(RED "\t- HISTOGRAM\n" RESET);
        printf(RED "\t- CLAHE\n" RESET);
        std::exit(EXIT_FAILURE);
      }
      parser->parse_config("knn_ratio", knn_ratio);
      parser->parse_config("track_frequency", track_frequency);
      parser->parse_config("debug_display_waitkey_ms", debug_display_waitkey_ms,
                           false);
      parser->parse_config("use_imu_klt_prior", use_imu_klt_prior, false);
      parser->parse_config("imu_klt_prior_min_dt", imu_klt_prior_min_dt, false);
      parser->parse_config("imu_klt_prior_max_dt", imu_klt_prior_max_dt, false);
      parser->parse_config("imu_klt_prior_debug", imu_klt_prior_debug, false);
      parser->parse_config("print_klt_geom", print_klt_geom, false);
    }
    PRINT_DEBUG("特征跟踪参数:\n");
    PRINT_DEBUG("  - 使用双目: %d\n", use_stereo);
    PRINT_DEBUG("  - 使用KLT: %d\n", use_klt);
    PRINT_DEBUG("  - 使用ArUco: %d\n", use_aruco);
    PRINT_DEBUG("  - 缩小ArUco: %d\n", downsize_aruco);
    PRINT_DEBUG("  - 缩小相机: %d\n", downsample_cameras);
    PRINT_DEBUG("  - OpenCV线程数: %d\n", num_opencv_threads);
    PRINT_DEBUG("  - 使用多线程发布: %d\n", use_multi_threading_pubs);
    PRINT_DEBUG("  - 使用多线程订阅: %d\n", use_multi_threading_subs);
    PRINT_DEBUG("  - 特征点数: %d\n", num_pts);
    PRINT_DEBUG("  - FAST阈值: %d\n", fast_threshold);
    PRINT_DEBUG("  - 网格 X x Y: %d x %d\n", grid_x, grid_y);
    PRINT_DEBUG("  - 最小像素距离: %d\n", min_px_dist);
    PRINT_DEBUG("  - 直方图方法: %d\n", (int)histogram_method);
    PRINT_DEBUG("  - KNN比率: %.3f\n", knn_ratio);
    PRINT_DEBUG("  - 跟踪频率: %.1f\n", track_frequency);
    PRINT_DEBUG("  - 调试显示 waitKey 时长(ms): %d\n",
                debug_display_waitkey_ms);
    PRINT_DEBUG("  - 启用IMU-KLT先验: %d\n", (int)use_imu_klt_prior);
    PRINT_DEBUG("  - IMU-KLT先验帧间隔范围: [%.4f, %.4f] s\n",
                imu_klt_prior_min_dt, imu_klt_prior_max_dt);
    PRINT_DEBUG("  - IMU-KLT先验调试打印: %d\n", (int)imu_klt_prior_debug);
    PRINT_DEBUG("  - 打印KLT几何校验[KLT-GEOM]: %d\n", (int)print_klt_geom);
    featinit_options.print(parser);
  }

  // SIMULATOR ===============================
  // 仿真器参数 ===============================

  /// 初始状态的随机种子（即生成地图中随机特征点3D位置）
  /// Seed for initial states (i.e. random feature 3d positions in the generated map)
  int sim_seed_state_init = 0;

  /// 标定扰动的随机种子。如果启用了扰动，更改此值以使用不同的随机值进行扰动。
  /// Seed for calibration perturbations. Change this to perturb by different random values if perturbations are enabled.
  int sim_seed_preturb = 0;

  /// 测量噪声的随机种子。在蒙特卡洛仿真中，每次运行应递增此值，以生成相同的真实测量值，但不同的噪声值。
  /// Measurement noise seed. This should be incremented for each run in the Monte-Carlo simulation to generate the same true measurements,
  /// but diffferent noise values.
  int sim_seed_measurements = 0;

  /// 是否应该扰动估计器开始时的标定参数
  /// If we should perturb the calibration that the estimator starts with
  bool sim_do_perturbation = false;

  /// 我们将进行B样条和仿真的轨迹路径。应为time(s),pos(xyz),ori(xyzw)格式。
  /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw) format.
  std::string sim_traj_path = "src/open_vins/ov_data/sim/udel_gore.txt";

  /// 在沿B样条移动此距离后，我们将开始仿真。这可以防止静态启动，因为我们在仿真中从真值初始化。
  /// We will start simulating after we have moved this much along the b-spline. This prevents static starts as we init from groundtruth in
  /// simulation.
  double sim_distance_threshold = 1.2;

  /// 仿真相机的频率（Hz）
  /// Frequency (Hz) that we will simulate our cameras
  double sim_freq_cam = 10.0;

  /// 仿真惯性测量单元的频率（Hz）
  /// Frequency (Hz) that we will simulate our inertial measurement unit
  double sim_freq_imu = 400.0;

  /// 生成特征点的距离（最小值）
  /// Feature distance we generate features from (minimum)
  double sim_min_feature_gen_distance = 5;

  /// 生成特征点的距离（最大值）
  /// Feature distance we generate features from (maximum)
  double sim_max_feature_gen_distance = 10;

  /**
   * @brief 加载并打印所有仿真参数
   * 这允许直观检查是否从ROS/CMD解析器正确加载了所有内容。
   *
   * @param parser 如果不为空，将使用此解析器加载参数
   * 
   * @brief This function will load print out all simulated parameters.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_simulation(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("sim_seed_state_init", sim_seed_state_init);
      parser->parse_config("sim_seed_preturb", sim_seed_preturb);
      parser->parse_config("sim_seed_measurements", sim_seed_measurements);
      parser->parse_config("sim_do_perturbation", sim_do_perturbation);
      parser->parse_config("sim_traj_path", sim_traj_path);
      parser->parse_config("sim_distance_threshold", sim_distance_threshold);
      parser->parse_config("sim_freq_cam", sim_freq_cam);
      parser->parse_config("sim_freq_imu", sim_freq_imu);
      parser->parse_config("sim_min_feature_gen_dist", sim_min_feature_gen_distance);
      parser->parse_config("sim_max_feature_gen_dist", sim_max_feature_gen_distance);
    }
    PRINT_DEBUG("仿真参数:\n");
    PRINT_WARNING(BOLDRED "  - 状态初始化种子: %d \n" RESET, sim_seed_state_init);
    PRINT_WARNING(BOLDRED "  - 扰动种子: %d \n" RESET, sim_seed_preturb);
    PRINT_WARNING(BOLDRED "  - 测量噪声种子: %d \n" RESET, sim_seed_measurements);
    PRINT_WARNING(BOLDRED "  - 是否进行扰动?: %d\n" RESET, sim_do_perturbation);
    PRINT_DEBUG("  - 轨迹路径: %s\n", sim_traj_path.c_str());
    PRINT_DEBUG("  - 距离阈值: %.2f\n", sim_distance_threshold);
    PRINT_DEBUG("  - 相机频率: %.2f\n", sim_freq_cam);
    PRINT_DEBUG("  - IMU频率: %.2f\n", sim_freq_imu);
    PRINT_DEBUG("  - 最小特征距离: %.2f\n", sim_min_feature_gen_distance);
    PRINT_DEBUG("  - 最大特征距离: %.2f\n", sim_max_feature_gen_distance);
  }
};

} // namespace ov_msckf

#endif // OV_MSCKF_VIOMANAGEROPTIONS_H