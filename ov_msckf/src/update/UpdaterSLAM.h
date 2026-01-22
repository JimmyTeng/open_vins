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

#ifndef OV_MSCKF_UPDATER_SLAM_H
#define OV_MSCKF_UPDATER_SLAM_H

#include <Eigen/Eigen>
#include <memory>

#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"

namespace ov_core {
class Feature;
class FeatureInitializer;
} // namespace ov_core
namespace ov_type {
class Landmark;
} // namespace ov_type

namespace ov_msckf {

class State;

/**
 * @brief 计算稀疏SLAM特征系统并更新滤波器
 *
 * 此类负责执行延迟特征初始化、SLAM更新和
 * 锚定特征表示的SLAM锚点更改。
 */
class UpdaterSLAM {

public:
  /**
   * @brief SLAM更新器的默认构造函数
   *
   * 更新器具有特征初始化器，用于根据需要初始化特征。
   * 选项允许调整更新的不同参数。
   *
   * @param options_slam SLAM特征的更新器选项（包括测量噪声值）
   * @param options_aruco ARUCO特征的更新器选项（包括测量噪声值）
   * @param feat_init_options 特征初始化器选项
   */
  UpdaterSLAM(UpdaterOptions &options_slam, UpdaterOptions &options_aruco, ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief 给定跟踪的SLAM特征，尝试使用它们更新状态
   * @param state 滤波器的状态
   * @param feature_vec 可用于更新的特征
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  /**
   * @brief 给定最大跟踪特征，尝试在状态中初始化它们
   * @param state 滤波器的状态
   * @param feature_vec 可用于更新的特征
   */
  void delayed_init(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  /**
   * @brief 如果SLAM特征将被边缘化，则更改其锚点
   *
   * 确保如果任何克隆即将被边缘化，则更改锚点表示。
   * 默认情况下，这将把锚点转移到最新的IMU克隆，并保持相机标定锚点不变。
   *
   * @param state 滤波器的状态
   */
  void change_anchors(std::shared_ptr<State> state);

protected:
  /**
   * @brief 将地标锚点转移到新克隆
   * @param state 滤波器状态
   * @param landmark 正在转移锚点的地标
   * @param new_anchor_timestamp 要转移到的克隆时间戳
   * @param new_cam_id 要转移到的相机帧
   */
  void perform_anchor_change(std::shared_ptr<State> state, std::shared_ptr<ov_type::Landmark> landmark, double new_anchor_timestamp,
                             size_t new_cam_id);

  /// SLAM特征更新期间使用的选项
  UpdaterOptions _options_slam;

  /// ARUCO特征更新期间使用的选项
  UpdaterOptions _options_aruco;

  /// 特征初始化器类对象
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

  /// 卡方分布95百分位表（查找键为残差的大小）
  std::map<int, double> chi_squared_table;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_SLAM_H
