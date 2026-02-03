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

#ifndef OV_MSCKF_UPDATER_HELPER_H
#define OV_MSCKF_UPDATER_HELPER_H

#include <Eigen/Eigen>
#include <memory>
#include <unordered_map>

#include "types/LandmarkRepresentation.h"

namespace ov_type {
class Type;
} // namespace ov_type

namespace ov_msckf {

class State;

/**
 * @brief 包含更新器辅助函数的类
 *
 * 可以计算单个特征表示的雅可比矩阵。
 * 这将根据状态使用的表示形式创建雅可比矩阵。
 * 如果使用锚点表示，则还需要计算相对于锚点状态的额外雅可比矩阵。
 * 还包含零空间投影和完整雅可比矩阵构建等函数。
 * 有关推导，请参阅 @ref update-feat 页面，其中包含详细的方程。
 *
 */
class UpdaterHelper {
public:
  /**
   * @brief UpdaterHelper使用的特征对象，包含所有测量值和均值
   */
  struct UpdaterHelperFeature {

    /// 此特征的唯一ID
    size_t featid;

    /// 从各个相机看到的UV坐标（按相机ID映射）
    std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs;

    /// 从各个相机看到的归一化UV坐标（按相机ID映射）
    std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs_norm;

    /// 每个UV测量的时间戳（按相机ID映射）
    std::unordered_map<size_t, std::vector<double>> timestamps;

    /// 特征使用的表示形式
    ov_type::LandmarkRepresentation::Representation feat_representation;

    /// 姿态锚定的相机ID！！默认情况下第一个测量值是锚点
    int anchor_cam_id = -1;

    /// 锚点克隆的时间戳
    double anchor_clone_timestamp = -1;

    /// 此特征在锚点坐标系中的三角化位置
    Eigen::Vector3d p_FinA;

    /// 此特征在锚点坐标系中的三角化位置（首次估计）
    Eigen::Vector3d p_FinA_fej;

    /// 此特征在全局坐标系中的三角化位置
    Eigen::Vector3d p_FinG;

    /// 此特征在全局坐标系中的三角化位置（首次估计）
    Eigen::Vector3d p_FinG_fej;
  };

  /**
   * @brief 获取相对于特征表示的特征和状态雅可比矩阵
   *
   * @param[in] state 滤波器系统的状态
   * @param[in] feature 要获取雅可比矩阵的特征（必须具有特征均值）
   * @param[out] H_f 相对于特征误差状态的雅可比矩阵（对于单深度将是3x3或3x1）
   * @param[out] H_x 相对于状态的额外雅可比矩阵（例如锚定姿态）
   * @param[out] x_order 额外雅可比矩阵的额外变量（例如锚定姿态）
   */
  static void get_feature_jacobian_representation(std::shared_ptr<State> state, UpdaterHelperFeature &feature, Eigen::MatrixXd &H_f,
                                                  std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<ov_type::Type>> &x_order);

  /**
   * @brief 将从所有测量值构建单个特征的"堆叠"雅可比矩阵
   *
   * @param[in] state 滤波器系统的状态
   * @param[in] feature 要获取雅可比矩阵的特征（必须具有特征均值）
   * @param[out] H_f 相对于特征误差状态的雅可比矩阵
   * @param[out] H_x 相对于状态的额外雅可比矩阵（例如锚定姿态）
   * @param[out] res 此特征的测量残差
   * @param[out] x_order 额外雅可比矩阵的额外变量（例如锚定姿态）
   */
  static void get_feature_jacobian_full(std::shared_ptr<State> state, UpdaterHelperFeature &feature, Eigen::MatrixXd &H_f,
                                        Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<ov_type::Type>> &x_order);

  /**
   * @brief 将H_f的左零空间投影到线性系统上
   *
   * 有关工作原理的详细信息，请参阅 @ref update-null。
   * 这是MSCKF零空间投影，它消除了对特征状态的依赖。
   * 注意这是**原地**完成的，因此函数调用后所有矩阵都会不同。
   *
   * @param H_f 要投影到系统上的具有零空间的雅可比矩阵 [res = Hx*(x-xhat)+Hf(f-fhat)+n]
   * @param H_x 状态雅可比矩阵
   * @param res 测量残差
   */
  static void nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

  /**
   * @brief 执行测量压缩
   *
   * 有关工作原理的详细信息，请参阅 @ref update-compress。
   * 注意这是**原地**完成的，因此函数调用后所有矩阵都会不同。
   *
   * @param H_x 状态雅可比矩阵
   * @param res 测量残差
   */
  static void measurement_compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_HELPER_H