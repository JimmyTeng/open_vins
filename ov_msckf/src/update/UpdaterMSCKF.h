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

#ifndef OV_MSCKF_UPDATER_MSCKF_H
#define OV_MSCKF_UPDATER_MSCKF_H

#include <Eigen/Eigen>
#include <memory>

#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"

namespace ov_core {
class Feature;
class FeatureInitializer;
} // namespace ov_core

namespace ov_msckf {

class State;

/**
 * @brief 计算稀疏特征系统并更新滤波器
 *
 * 此类负责计算用于更新的所有特征的整个线性系统。
 * 这遵循原始MSCKF，首先对特征进行三角化，然后对特征雅可比矩阵进行零空间投影。
 * 之后压缩所有测量值以实现高效更新并更新状态。
 */
class UpdaterMSCKF {

public:
  /**
   * @brief MSCKF更新器的默认构造函数
   *
   * 更新器具有特征初始化器，用于根据需要初始化特征。
   * 选项允许调整更新的不同参数。
   *
   * @param options 更新器选项（包括测量噪声值）
   * @param feat_init_options 特征初始化器选项
   */
  UpdaterMSCKF(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief 给定跟踪的特征，尝试使用它们更新状态
   *
   * @param state 滤波器的状态
   * @param feature_vec 可用于更新的特征
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

protected:
  /// 更新期间使用的选项
  UpdaterOptions _options;

  /// 特征初始化器类对象
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

  /// 卡方分布95百分位表（查找键为残差的大小）
  std::map<int, double> chi_squared_table;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_H
