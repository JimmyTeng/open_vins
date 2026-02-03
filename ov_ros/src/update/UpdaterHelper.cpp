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

#include "UpdaterHelper.h"

#include "state/State.h"

#include "utils/quat_ops.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

void UpdaterHelper::get_feature_jacobian_representation(std::shared_ptr<State> state, UpdaterHelperFeature &feature, Eigen::MatrixXd &H_f,
                                                        std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<Type>> &x_order) {

  // 全局XYZ表示
  if (feature.feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D) {
    H_f.resize(3, 3);
    H_f.setIdentity();
    return;
  }

  // 全局逆深度表示
  if (feature.feat_representation == LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH) {

    // 获取特征线性化点
    Eigen::Matrix<double, 3, 1> p_FinG = (state->_options.do_fej) ? feature.p_FinG_fej : feature.p_FinG;

    // 获取逆深度表示（应与Landmark.cpp中的内容匹配）
    double g_rho = 1 / p_FinG.norm();
    double g_phi = std::acos(g_rho * p_FinG(2));
    // double g_theta = std::asin(g_rho*p_FinG(1)/std::sin(g_phi));
    double g_theta = std::atan2(p_FinG(1), p_FinG(0));
    Eigen::Matrix<double, 3, 1> p_invFinG;
    p_invFinG(0) = g_theta;
    p_invFinG(1) = g_phi;
    p_invFinG(2) = g_rho;

    // 获取逆深度方向
    double sin_th = std::sin(p_invFinG(0, 0));
    double cos_th = std::cos(p_invFinG(0, 0));
    double sin_phi = std::sin(p_invFinG(1, 0));
    double cos_phi = std::cos(p_invFinG(1, 0));
    double rho = p_invFinG(2, 0);

    // 构建雅可比矩阵
    H_f.resize(3, 3);
    H_f << -(1.0 / rho) * sin_th * sin_phi, (1.0 / rho) * cos_th * cos_phi, -(1.0 / (rho * rho)) * cos_th * sin_phi,
        (1.0 / rho) * cos_th * sin_phi, (1.0 / rho) * sin_th * cos_phi, -(1.0 / (rho * rho)) * sin_th * sin_phi, 0.0,
        -(1.0 / rho) * sin_phi, -(1.0 / (rho * rho)) * cos_phi;
    return;
  }

  //======================================================================
  //======================================================================
  //======================================================================

  // 断言此特征具有锚点姿态
  assert(feature.anchor_cam_id != -1);

  // 锚点姿态的方向和位置，以及锚点相机的相机标定
  Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(feature.anchor_cam_id)->Rot();
  Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(feature.anchor_cam_id)->pos();
  Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(feature.anchor_clone_timestamp)->Rot();
  Eigen::Vector3d p_IinG = state->_clones_IMU.at(feature.anchor_clone_timestamp)->pos();
  Eigen::Vector3d p_FinA = feature.p_FinA;

  // 如果执行FEJ，应该对锚点状态执行FEJ（是否应该对标定执行FEJ???）
  // 如果执行FEJ，还要获取特征的FEJ位置
  if (state->_options.do_fej) {
    // 全局坐标系中的"最佳"特征
    Eigen::Vector3d p_FinG_best = R_GtoI.transpose() * R_ItoC.transpose() * (feature.p_FinA - p_IinC) + p_IinG;
    // 使用FEJ将最佳值转换到锚点坐标系
    R_GtoI = state->_clones_IMU.at(feature.anchor_clone_timestamp)->Rot_fej();
    p_IinG = state->_clones_IMU.at(feature.anchor_clone_timestamp)->pos_fej();
    p_FinA = (R_GtoI.transpose() * R_ItoC.transpose()).transpose() * (p_FinG_best - p_IinG) + p_IinC;
  }
  Eigen::Matrix3d R_CtoG = R_GtoI.transpose() * R_ItoC.transpose();

  // 锚点姿态的雅可比矩阵
  Eigen::Matrix<double, 3, 6> H_anc;
  H_anc.block(0, 0, 3, 3).noalias() = -R_GtoI.transpose() * skew_x(R_ItoC.transpose() * (p_FinA - p_IinC));
  H_anc.block(0, 3, 3, 3).setIdentity();

  // 将锚点雅可比矩阵添加到返回向量
  x_order.push_back(state->_clones_IMU.at(feature.anchor_clone_timestamp));
  H_x.push_back(H_anc);

  // 获取标定雅可比矩阵（用于锚点克隆）
  if (state->_options.do_calib_camera_pose) {
    Eigen::Matrix<double, 3, 6> H_calib;
    H_calib.block(0, 0, 3, 3).noalias() = -R_CtoG * skew_x(p_FinA - p_IinC);
    H_calib.block(0, 3, 3, 3) = -R_CtoG;
    x_order.push_back(state->_calib_IMUtoCAM.at(feature.anchor_cam_id));
    H_x.push_back(H_calib);
  }

  // 如果使用锚定XYZ特征
  if (feature.feat_representation == LandmarkRepresentation::Representation::ANCHORED_3D) {
    H_f = R_CtoG;
    return;
  }

  // 如果使用完整逆深度
  if (feature.feat_representation == LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH) {

    // 获取逆深度表示（应与Landmark.cpp中的内容匹配）
    double a_rho = 1 / p_FinA.norm();
    double a_phi = std::acos(a_rho * p_FinA(2));
    double a_theta = std::atan2(p_FinA(1), p_FinA(0));
    Eigen::Matrix<double, 3, 1> p_invFinA;
    p_invFinA(0) = a_theta;
    p_invFinA(1) = a_phi;
    p_invFinA(2) = a_rho;

    // 使用锚定逆深度
    double sin_th = std::sin(p_invFinA(0, 0));
    double cos_th = std::cos(p_invFinA(0, 0));
    double sin_phi = std::sin(p_invFinA(1, 0));
    double cos_phi = std::cos(p_invFinA(1, 0));
    double rho = p_invFinA(2, 0);
    // assert(p_invFinA(2,0)>=0.0);

    // 锚定3D位置相对于逆深度参数的雅可比矩阵
    Eigen::Matrix<double, 3, 3> d_pfinA_dpinv;
    d_pfinA_dpinv << -(1.0 / rho) * sin_th * sin_phi, (1.0 / rho) * cos_th * cos_phi, -(1.0 / (rho * rho)) * cos_th * sin_phi,
        (1.0 / rho) * cos_th * sin_phi, (1.0 / rho) * sin_th * cos_phi, -(1.0 / (rho * rho)) * sin_th * sin_phi, 0.0,
        -(1.0 / rho) * sin_phi, -(1.0 / (rho * rho)) * cos_phi;
    H_f = R_CtoG * d_pfinA_dpinv;
    return;
  }

  // 如果使用MSCKF版本的逆深度
  if (feature.feat_representation == LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH) {

    // 获取逆深度表示（应与Landmark.cpp中的内容匹配）
    Eigen::Matrix<double, 3, 1> p_invFinA_MSCKF;
    p_invFinA_MSCKF(0) = p_FinA(0) / p_FinA(2);
    p_invFinA_MSCKF(1) = p_FinA(1) / p_FinA(2);
    p_invFinA_MSCKF(2) = 1 / p_FinA(2);

    // 使用MSCKF版本的逆深度
    double alpha = p_invFinA_MSCKF(0, 0);
    double beta = p_invFinA_MSCKF(1, 0);
    double rho = p_invFinA_MSCKF(2, 0);

    // 锚定3D位置相对于逆深度参数的雅可比矩阵
    Eigen::Matrix<double, 3, 3> d_pfinA_dpinv;
    d_pfinA_dpinv << (1.0 / rho), 0.0, -(1.0 / (rho * rho)) * alpha, 0.0, (1.0 / rho), -(1.0 / (rho * rho)) * beta, 0.0, 0.0,
        -(1.0 / (rho * rho));
    H_f = R_CtoG * d_pfinA_dpinv;
    return;
  }

  /// 情况：使用初始方向估计特征的单个深度
  if (feature.feat_representation == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {

    // 获取逆深度表示（应与Landmark.cpp中的内容匹配）
    double rho = 1.0 / p_FinA(2);
    Eigen::Vector3d bearing = rho * p_FinA;

    // 锚定3D位置相对于逆深度参数的雅可比矩阵
    Eigen::Vector3d d_pfinA_drho;
    d_pfinA_drho << -(1.0 / (rho * rho)) * bearing;
    H_f = R_CtoG * d_pfinA_drho;
    return;
  }

  // 失败，未编程的无效表示
  assert(false);
}

void UpdaterHelper::get_feature_jacobian_full(std::shared_ptr<State> state, UpdaterHelperFeature &feature, Eigen::MatrixXd &H_f,
                                              Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<Type>> &x_order) {

  // 此特征的总测量数
  int total_meas = 0;
  for (auto const &pair : feature.timestamps) {
    total_meas += (int)pair.second.size();
  }

  // 计算与此特征相关的状态大小
  int total_hx = 0;
  std::unordered_map<std::shared_ptr<Type>, size_t> map_hx;
  for (auto const &pair : feature.timestamps) {

    // 外参和内参
    std::shared_ptr<PoseJPL> calibration = state->_calib_IMUtoCAM.at(pair.first);
    std::shared_ptr<Vec> distortion = state->_cam_intrinsics.at(pair.first);

    // 如果进行外参标定
    if (state->_options.do_calib_camera_pose) {
      map_hx.insert({calibration, total_hx});
      x_order.push_back(calibration);
      total_hx += calibration->size();
    }

    // 如果进行内参标定
    if (state->_options.do_calib_camera_intrinsics) {
      map_hx.insert({distortion, total_hx});
      x_order.push_back(distortion);
      total_hx += distortion->size();
    }

    // 遍历此特定相机的所有测量值
    for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++) {

      // 如果尚未添加，则添加此克隆
      std::shared_ptr<PoseJPL> clone_Ci = state->_clones_IMU.at(feature.timestamps[pair.first].at(m));
      if (map_hx.find(clone_Ci) == map_hx.end()) {
        map_hx.insert({clone_Ci, total_hx});
        x_order.push_back(clone_Ci);
        total_hx += clone_Ci->size();
      }
    }
  }

  // 如果使用锚定表示，确保也添加锚点
  if (LandmarkRepresentation::is_relative_representation(feature.feat_representation)) {

    // 断言我们有一个克隆
    assert(feature.anchor_cam_id != -1);

    // 如果尚未添加，则添加此锚点
    std::shared_ptr<PoseJPL> clone_Ai = state->_clones_IMU.at(feature.anchor_clone_timestamp);
    if (map_hx.find(clone_Ai) == map_hx.end()) {
      map_hx.insert({clone_Ai, total_hx});
      x_order.push_back(clone_Ai);
      total_hx += clone_Ai->size();
    }

    // 如果进行标定，也添加其标定
    if (state->_options.do_calib_camera_pose) {
      // 如果尚未添加，则添加此锚点
      std::shared_ptr<PoseJPL> clone_calib = state->_calib_IMUtoCAM.at(feature.anchor_cam_id);
      if (map_hx.find(clone_calib) == map_hx.end()) {
        map_hx.insert({clone_calib, total_hx});
        x_order.push_back(clone_calib);
        total_hx += clone_calib->size();
      }
    }
  }

  //=========================================================================
  //=========================================================================

  // 计算此特征在全局坐标系中的位置
  // 如果锚定，则需要计算特征在全局坐标系中的位置
  Eigen::Vector3d p_FinG = feature.p_FinG;
  if (LandmarkRepresentation::is_relative_representation(feature.feat_representation)) {
    // 断言此特征具有锚点姿态
    assert(feature.anchor_cam_id != -1);
    // 获取锚点相机的标定
    Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(feature.anchor_cam_id)->Rot();
    Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(feature.anchor_cam_id)->pos();
    // 锚点姿态的方向和位置
    Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(feature.anchor_clone_timestamp)->Rot();
    Eigen::Vector3d p_IinG = state->_clones_IMU.at(feature.anchor_clone_timestamp)->pos();
    // 全局坐标系中的特征
    p_FinG = R_GtoI.transpose() * R_ItoC.transpose() * (feature.p_FinA - p_IinC) + p_IinG;
  }

  // 计算此特征在全局坐标系中的位置FEJ
  // 如果锚定，则可以使用"最佳"p_FinG，因为p_FinA的值无关紧要
  Eigen::Vector3d p_FinG_fej = feature.p_FinG_fej;
  if (LandmarkRepresentation::is_relative_representation(feature.feat_representation)) {
    p_FinG_fej = p_FinG;
  }

  //=========================================================================
  //=========================================================================

  // 分配残差和雅可比矩阵
  int c = 0;
  int jacobsize = (feature.feat_representation != LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 3 : 1;
  res = Eigen::VectorXd::Zero(2 * total_meas);
  H_f = Eigen::MatrixXd::Zero(2 * total_meas, jacobsize);
  H_x = Eigen::MatrixXd::Zero(2 * total_meas, total_hx);

  // p_FinG相对于特征表示的导数
  // 这只需要计算一次，因此我们将其从循环中提取出来
  Eigen::MatrixXd dpfg_dlambda;
  std::vector<Eigen::MatrixXd> dpfg_dx;
  std::vector<std::shared_ptr<Type>> dpfg_dx_order;
  UpdaterHelper::get_feature_jacobian_representation(state, feature, dpfg_dlambda, dpfg_dx, dpfg_dx_order);

  // 断言顺序中的所有项都已在我们本地雅可比映射中
#ifndef NDEBUG
  for (auto &type : dpfg_dx_order) {
    assert(map_hx.find(type) != map_hx.end());
  }
#endif

  // 遍历此特征的每个相机
  for (auto const &pair : feature.timestamps) {

    // IMU和CAMi坐标系之间的标定
    std::shared_ptr<Vec> distortion = state->_cam_intrinsics.at(pair.first);
    std::shared_ptr<PoseJPL> calibration = state->_calib_IMUtoCAM.at(pair.first);
    Eigen::Matrix3d R_ItoC = calibration->Rot();
    Eigen::Vector3d p_IinC = calibration->pos();

    // 遍历此特定相机的所有测量值
    for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++) {

      //=========================================================================
      //=========================================================================

      // 获取当前IMU克隆状态
      std::shared_ptr<PoseJPL> clone_Ii = state->_clones_IMU.at(feature.timestamps[pair.first].at(m));
      Eigen::Matrix3d R_GtoIi = clone_Ii->Rot();
      Eigen::Vector3d p_IiinG = clone_Ii->pos();

      // 获取IMU中的当前特征
      Eigen::Vector3d p_FinIi = R_GtoIi * (p_FinG - p_IiinG);

      // 将当前特征投影到当前参考坐标系
      Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;
      Eigen::Vector2d uv_norm;
      uv_norm << p_FinCi(0) / p_FinCi(2), p_FinCi(1) / p_FinCi(2);

      // 对归一化坐标进行畸变（radtan或fisheye）
      Eigen::Vector2d uv_dist;
      uv_dist = state->_cam_intrinsics_cameras.at(pair.first)->distort_d(uv_norm);

      // 残差
      Eigen::Vector2d uv_m;
      uv_m << (double)feature.uvs[pair.first].at(m)(0), (double)feature.uvs[pair.first].at(m)(1);
      res.block(2 * c, 0, 2, 1) = uv_m - uv_dist;

      //=========================================================================
      //=========================================================================

      // 如果执行首次估计雅可比矩阵，则用首次估计覆盖
      if (state->_options.do_fej) {
        R_GtoIi = clone_Ii->Rot_fej();
        p_IiinG = clone_Ii->pos_fej();
        // R_ItoC = calibration->Rot_fej();
        // p_IinC = calibration->pos_fej();
        p_FinIi = R_GtoIi * (p_FinG_fej - p_IiinG);
        p_FinCi = R_ItoC * p_FinIi + p_IinC;
        // uv_norm << p_FinCi(0)/p_FinCi(2),p_FinCi(1)/p_FinCi(2);
        // cam_d = state->get_intrinsics_CAM(pair.first)->fej();
      }

      // 计算相对于归一化图像坐标和可能的相机内参的雅可比矩阵
      Eigen::MatrixXd dz_dzn, dz_dzeta;
      state->_cam_intrinsics_cameras.at(pair.first)->compute_distort_jacobian(uv_norm, dz_dzn, dz_dzeta);

      // 相对于投影函数的归一化坐标
      Eigen::MatrixXd dzn_dpfc = Eigen::MatrixXd::Zero(2, 3);
      dzn_dpfc << 1 / p_FinCi(2), 0, -p_FinCi(0) / (p_FinCi(2) * p_FinCi(2)), 0, 1 / p_FinCi(2), -p_FinCi(1) / (p_FinCi(2) * p_FinCi(2));

      // p_FinCi相对于p_FinIi的导数
      Eigen::MatrixXd dpfc_dpfg = R_ItoC * R_GtoIi;

      // p_FinCi相对于相机克隆状态的导数
      Eigen::MatrixXd dpfc_dclone = Eigen::MatrixXd::Zero(3, 6);
      dpfc_dclone.block(0, 0, 3, 3).noalias() = R_ItoC * skew_x(p_FinIi);
      dpfc_dclone.block(0, 3, 3, 3) = -dpfc_dpfg;

      //=========================================================================
      //=========================================================================

      // 预计算一些矩阵
      Eigen::MatrixXd dz_dpfc = dz_dzn * dzn_dpfc;
      Eigen::MatrixXd dz_dpfg = dz_dpfc * dpfc_dpfg;

      // 链式法则：获取总特征雅可比矩阵
      H_f.block(2 * c, 0, 2, H_f.cols()).noalias() = dz_dpfg * dpfg_dlambda;

      // 链式法则：获取状态克隆雅可比矩阵
      H_x.block(2 * c, map_hx[clone_Ii], 2, clone_Ii->size()).noalias() = dz_dpfc * dpfc_dclone;

      // 链式法则：遍历所有额外状态并添加它们的雅可比矩阵
      // 注意：我们在这里添加雅可比矩阵，因为此测量可能处于锚定姿态
      for (size_t i = 0; i < dpfg_dx_order.size(); i++) {
        H_x.block(2 * c, map_hx[dpfg_dx_order.at(i)], 2, dpfg_dx_order.at(i)->size()).noalias() += dz_dpfg * dpfg_dx.at(i);
      }

      //=========================================================================
      //=========================================================================

      // p_FinCi相对于相机标定的导数（R_ItoC, p_IinC）
      if (state->_options.do_calib_camera_pose) {

        // 计算雅可比矩阵
        Eigen::MatrixXd dpfc_dcalib = Eigen::MatrixXd::Zero(3, 6);
        dpfc_dcalib.block(0, 0, 3, 3) = skew_x(p_FinCi - p_IinC);
        dpfc_dcalib.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

        // 应用链式法则并将其添加到大型雅可比矩阵
        H_x.block(2 * c, map_hx[calibration], 2, calibration->size()).noalias() += dz_dpfc * dpfc_dcalib;
      }

      // 测量相对于畸变参数的导数
      if (state->_options.do_calib_camera_intrinsics) {
        H_x.block(2 * c, map_hx[distortion], 2, distortion->size()) = dz_dzeta;
      }

      // 将雅可比矩阵和残差索引向前移动
      c++;
    }
  }
}

void UpdaterHelper::nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // 将H_f的左零空间应用到所有变量
  // 基于"Matrix Computations 4th Edition by Golub and Van Loan"
  // 有关这两个循环的工作原理，请参阅第252页，算法5.2.4
  // 它们使用"matlab"索引表示法，因此我们需要从所有索引中减去1
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_f.cols(); ++n) {
    for (int m = (int)H_f.rows() - 1; m > n; m--) {
      // Givens矩阵G
      tempHo_GR.makeGivens(H_f(m - 1, n), H_f(m, n));
      // 将G乘以每个矩阵中的相应行(m-1,m)
      // 注意：我们只将G应用于非零列[n:Ho.cols()-n-1]，而
      //       这等效于将G应用于整个列[0:Ho.cols()-1]
      (H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // 如果H_f是3D位置，则H_f雅可比矩阵的最大秩为3，因此左零空间的大小为Hf.rows()-3
  // 注意：这里需要eigen3 eval，因为这会遇到别名！
  // H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols()).eval();
  H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
  res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();

  // 合理性检查
  assert(H_x.rows() == res.rows());
}

void UpdaterHelper::measurement_compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // 如果H_x是宽矩阵，则返回（在这种情况下不需要压缩）
  if (H_x.rows() <= H_x.cols())
    return;

  // 通过Givens旋转进行测量压缩
  // 基于"Matrix Computations 4th Edition by Golub and Van Loan"
  // 有关这两个循环的工作原理，请参阅第252页，算法5.2.4
  // 它们使用"matlab"索引表示法，因此我们需要从所有索引中减去1
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_x.cols(); n++) {
    for (int m = (int)H_x.rows() - 1; m > n; m--) {
      // Givens矩阵G
      tempHo_GR.makeGivens(H_x(m - 1, n), H_x(m, n));
      // 将G乘以每个矩阵中的相应行(m-1,m)
      // 注意：我们只将G应用于非零列[n:Ho.cols()-n-1]，而
      //       这等效于将G应用于整个列[0:Ho.cols()-1]
      (H_x.block(m - 1, n, 2, H_x.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // 如果H是宽矩阵，则使用行数
  // 否则它应该与我们的状态大小相同
  int r = std::min(H_x.rows(), H_x.cols());

  // 在测量压缩后构建较小的雅可比矩阵和残差
  assert(r <= H_x.rows());
  H_x.conservativeResize(r, H_x.cols());
  res.conservativeResize(r, res.cols());
}
