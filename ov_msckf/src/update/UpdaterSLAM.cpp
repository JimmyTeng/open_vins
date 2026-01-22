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

#include "UpdaterSLAM.h"

#include "UpdaterHelper.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterSLAM::UpdaterSLAM(UpdaterOptions &options_slam, UpdaterOptions &options_aruco, ov_core::FeatureInitializerOptions &feat_init_options)
    : _options_slam(options_slam), _options_aruco(options_aruco) {

  // 保存原始像素噪声的平方值
  _options_slam.sigma_pix_sq = std::pow(_options_slam.sigma_pix, 2);
  _options_aruco.sigma_pix_sq = std::pow(_options_aruco.sigma_pix, 2);

  // 保存特征初始化器
  initializer_feat = std::shared_ptr<ov_core::FeatureInitializer>(new ov_core::FeatureInitializer(feat_init_options));

  // 初始化置信度为0.95的卡方检验表
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterSLAM::delayed_init(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // 如果没有特征则返回
  if (feature_vec.empty())
    return;

  // 开始计时
  boost::posix_time::ptime rT0, rT1, rT2, rT3;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. 获取所有克隆所在的时间戳（即有效测量时间）
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. 清理所有特征测量值，确保它们都有有效的克隆时间
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // 清理特征
    (*it0)->clean_old_measurements(clonetimes);

    // 计算测量数量
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // 如果测量值不足则移除
    if (ct_meas < 2) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 2. 在每个克隆时间步创建克隆的*相机*姿态向量
  std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
  for (const auto &clone_calib : state->_calib_IMUtoCAM) {

    // 对于此相机，创建相机姿态向量
    std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
    for (const auto &clone_imu : state->_clones_IMU) {

      // 获取当前相机姿态
      Eigen::Matrix<double, 3, 3> R_GtoCi = clone_calib.second->Rot() * clone_imu.second->Rot();
      Eigen::Matrix<double, 3, 1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose() * clone_calib.second->pos();

      // 添加到映射中
      clones_cami.insert({clone_imu.first, FeatureInitializer::ClonePose(R_GtoCi, p_CioinG)});
    }

    // 添加到映射中
    clones_cam.insert({clone_calib.first, clones_cami});
  }

  // 3. 尝试对具有测量值的所有MSCKF或新SLAM特征进行三角化
  auto it1 = feature_vec.begin();
  while (it1 != feature_vec.end()) {

    // 对特征进行三角化，如果失败则移除
    bool success_tri = true;
    if (initializer_feat->config().triangulate_1d) {
      success_tri = initializer_feat->single_triangulation_1d(*it1, clones_cam);
    } else {
      success_tri = initializer_feat->single_triangulation(*it1, clones_cam);
    }

    // 使用高斯-牛顿法优化特征
    bool success_refine = true;
    if (initializer_feat->config().refine_features) {
      success_refine = initializer_feat->single_gaussnewton(*it1, clones_cam);
    }

    // 如果不成功则移除特征
    if (!success_tri || !success_refine) {
      (*it1)->to_delete = true;
      it1 = feature_vec.erase(it1);
      continue;
    }
    it1++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // 4. 计算每个特征的线性系统，进行零空间投影，并拒绝
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // 将特征转换为当前格式
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // 如果使用单逆深度，则等效于使用msckf逆深度
    auto feat_rep =
        ((int)feat.featid < state->_options.max_aruco_features) ? state->_options.feat_rep_aruco : state->_options.feat_rep_slam;
    feat.feat_representation = feat_rep;
    if (feat_rep == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    }

    // 保存位置及其fej值
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = (*it2)->anchor_cam_id;
      feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
      feat.p_FinA = (*it2)->p_FinA;
      feat.p_FinA_fej = (*it2)->p_FinA;
    } else {
      feat.p_FinG = (*it2)->p_FinG;
      feat.p_FinG_fej = (*it2)->p_FinG;
    }

    // 返回值（特征雅可比矩阵、状态雅可比矩阵、残差和状态雅可比矩阵的顺序）
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // 获取此特征的雅可比矩阵
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    // 如果使用单特征表示，则需要移除方向部分
    // 为此，我们将方向部分投影到状态和深度雅可比矩阵以及残差上
    // 这允许我们直接将特征初始化为深度-旧特征
    if (feat_rep == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {

      // 添加相对于特征深度的雅可比矩阵
      Eigen::MatrixXd H_xf = H_x;
      H_xf.conservativeResize(H_x.rows(), H_x.cols() + 1);
      H_xf.block(0, H_x.cols(), H_x.rows(), 1) = H_f.block(0, H_f.cols() - 1, H_f.rows(), 1);
      H_f.conservativeResize(H_f.rows(), H_f.cols() - 1);

      // 对方向部分进行零空间投影
      // 这考虑了我们已经边缘化了方向
      // 因此这对于确保估计器一致性至关重要，因为我们不将方向视为真值
      UpdaterHelper::nullspace_project_inplace(H_f, H_xf, res);

      // 分离状态部分和特征部分
      H_x = H_xf.block(0, 0, H_xf.rows(), H_xf.cols() - 1);
      H_f = H_xf.block(0, H_xf.cols() - 1, H_xf.rows(), 1);
    }

    // 创建特征指针（我们总是创建大小为3的，因为我们将单逆深度初始化为msckf锚定表示）
    int landmark_size = (feat_rep == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 1 : 3;
    auto landmark = std::make_shared<Landmark>(landmark_size);
    landmark->_featid = feat.featid;
    landmark->_feat_representation = feat_rep;
    landmark->_unique_camera_id = (*it2)->anchor_cam_id;
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      landmark->_anchor_cam_id = feat.anchor_cam_id;
      landmark->_anchor_clone_timestamp = feat.anchor_clone_timestamp;
      landmark->set_from_xyz(feat.p_FinA, false);
      landmark->set_from_xyz(feat.p_FinA_fej, true);
    } else {
      landmark->set_from_xyz(feat.p_FinG, false);
      landmark->set_from_xyz(feat.p_FinG_fej, true);
    }

    // 测量噪声矩阵
    double sigma_pix_sq =
        ((int)feat.featid < state->_options.max_aruco_features) ? _options_aruco.sigma_pix_sq : _options_slam.sigma_pix_sq;
    Eigen::MatrixXd R = sigma_pix_sq * Eigen::MatrixXd::Identity(res.rows(), res.rows());

    // 尝试初始化，如果失败则删除新指针
    double chi2_multipler =
        ((int)feat.featid < state->_options.max_aruco_features) ? _options_aruco.chi2_multipler : _options_slam.chi2_multipler;
    if (StateHelper::initialize(state, landmark, Hx_order, H_x, H_f, R, res, chi2_multipler)) {
      state->_features_SLAM.insert({(*it2)->featid, landmark});
      (*it2)->to_delete = true;
      it2++;
    } else {
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
    }
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  if (!feature_vec.empty()) {
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds to triangulate\n", (rT2 - rT1).total_microseconds() * 1e-6);
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds initialize (%d features)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds total\n", (rT3 - rT1).total_microseconds() * 1e-6);
  }
}

void UpdaterSLAM::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // 如果没有特征则返回
  if (feature_vec.empty())
    return;

  // 开始计时
  boost::posix_time::ptime rT0, rT1, rT2, rT3;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. 获取所有克隆所在的时间戳（即有效测量时间）
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. 清理所有特征测量值，确保它们都有有效的克隆时间
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // 清理特征
    (*it0)->clean_old_measurements(clonetimes);

    // 计算测量数量
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // 获取地标及其表示
    // 对于单深度表示，我们至少需要两个测量值
    // 这是因为我们进行零空间投影
    std::shared_ptr<Landmark> landmark = state->_features_SLAM.at((*it0)->featid);
    int required_meas = (landmark->_feat_representation == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 2 : 1;

    // 如果测量值不足则移除
    if (ct_meas < 1) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else if (ct_meas < required_meas) {
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 计算最大可能的测量大小
  size_t max_meas_size = 0;
  for (size_t i = 0; i < feature_vec.size(); i++) {
    for (const auto &pair : feature_vec.at(i)->timestamps) {
      max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
    }
  }

  // 计算最大可能的状态大小（即协方差的大小）
  size_t max_hx_size = state->max_covariance_size();

  // 此更新中*所有*特征的大型雅可比矩阵、残差和测量噪声
  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
  Eigen::MatrixXd R_big = Eigen::MatrixXd::Identity(max_meas_size, max_meas_size);
  std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
  std::vector<std::shared_ptr<Type>> Hx_order_big;
  size_t ct_jacob = 0;
  size_t ct_meas = 0;

  // 4. 计算每个特征的线性系统，进行零空间投影，并拒绝
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // 确保我们拥有地标且它是相同的
    assert(state->_features_SLAM.find((*it2)->featid) != state->_features_SLAM.end());
    assert(state->_features_SLAM.at((*it2)->featid)->_featid == (*it2)->featid);

    // 从状态获取地标
    std::shared_ptr<Landmark> landmark = state->_features_SLAM.at((*it2)->featid);

    // 将状态地标转换为当前格式
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // 如果使用单逆深度，则等效于使用msckf逆深度
    feat.feat_representation = landmark->_feat_representation;
    if (landmark->_feat_representation == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    }

    // 保存位置及其fej值
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = landmark->_anchor_cam_id;
      feat.anchor_clone_timestamp = landmark->_anchor_clone_timestamp;
      feat.p_FinA = landmark->get_xyz(false);
      feat.p_FinA_fej = landmark->get_xyz(true);
    } else {
      feat.p_FinG = landmark->get_xyz(false);
      feat.p_FinG_fej = landmark->get_xyz(true);
    }

    // 返回值（特征雅可比矩阵、状态雅可比矩阵、残差和状态雅可比矩阵的顺序）
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // 获取此特征的雅可比矩阵
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    // 将雅可比矩阵放在一个大型雅可比矩阵中，因为地标已经在我们的状态向量中
    Eigen::MatrixXd H_xf = H_x;
    if (landmark->_feat_representation == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {

      // 添加相对于特征深度的雅可比矩阵
      H_xf.conservativeResize(H_x.rows(), H_x.cols() + 1);
      H_xf.block(0, H_x.cols(), H_x.rows(), 1) = H_f.block(0, H_f.cols() - 1, H_f.rows(), 1);
      H_f.conservativeResize(H_f.rows(), H_f.cols() - 1);

      // 对方向部分进行零空间投影
      // 这考虑了我们已经边缘化了方向
      // 因此这对于确保估计器一致性至关重要，因为我们不将方向视为真值
      UpdaterHelper::nullspace_project_inplace(H_f, H_xf, res);

    } else {

      // 否则我们在状态中拥有完整特征，因此只需添加它
      H_xf.conservativeResize(H_x.rows(), H_x.cols() + H_f.cols());
      H_xf.block(0, H_x.cols(), H_x.rows(), H_f.cols()) = H_f;
    }

    // 添加到雅可比顺序向量
    std::vector<std::shared_ptr<Type>> Hxf_order = Hx_order;
    Hxf_order.push_back(landmark);

    // Chi2距离检查
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hxf_order);
    Eigen::MatrixXd S = H_xf * P_marg * H_xf.transpose();
    double sigma_pix_sq =
        ((int)feat.featid < state->_options.max_aruco_features) ? _options_aruco.sigma_pix_sq : _options_slam.sigma_pix_sq;
    S.diagonal() += sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));

    // 获取阈值（我们预计算到500，但处理超过的情况）
    double chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
    }

    // 检查是否应该删除
    double chi2_multipler =
        ((int)feat.featid < state->_options.max_aruco_features) ? _options_aruco.chi2_multipler : _options_slam.chi2_multipler;
    if (chi2 > chi2_multipler * chi2_check) {
      if ((int)feat.featid < state->_options.max_aruco_features) {
        PRINT_WARNING(YELLOW "[SLAM-UP]: rejecting aruco tag %d for chi2 thresh (%.3f > %.3f)\n" RESET, (int)feat.featid, chi2,
                      chi2_multipler * chi2_check);
      } else {
        landmark->update_fail_count++;
      }
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      continue;
    }

    // 调试打印：当我们更新aruco标签时
    if ((int)feat.featid < state->_options.max_aruco_features) {
      PRINT_DEBUG("[SLAM-UP]: accepted aruco tag %d for chi2 thresh (%.3f < %.3f)\n", (int)feat.featid, chi2, chi2_multipler * chi2_check);
    }

    // 很好！添加到大型H向量
    size_t ct_hx = 0;
    for (const auto &var : Hxf_order) {

      // 确保此变量在我们的雅可比矩阵中
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->size();
      }

      // 添加到大型雅可比矩阵
      Hx_big.block(ct_meas, Hx_mapping[var], H_xf.rows(), var->size()) = H_xf.block(0, ct_hx, H_xf.rows(), var->size());
      ct_hx += var->size();
    }

    // 各向同性测量噪声
    R_big.block(ct_meas, ct_meas, res.rows(), res.rows()) *= sigma_pix_sq;

    // 添加残差并向前移动
    res_big.block(ct_meas, 0, res.rows(), 1) = res;
    ct_meas += res.rows();
    it2++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // 我们已经将所有特征添加到Hx_big、res_big
  // 删除它们以便不重用信息
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  // 如果没有任何内容则返回并调整矩阵大小
  if (ct_meas < 1) {
    return;
  }
  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);
  res_big.conservativeResize(ct_meas, 1);
  Hx_big.conservativeResize(ct_meas, ct_jacob);
  R_big.conservativeResize(ct_meas, ct_meas);

  // 5. 使用所有良好的SLAM特征更新状态
  StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  PRINT_ALL("[SLAM-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[SLAM-UP]: %.4f seconds creating linear system\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[SLAM-UP]: %.4f seconds to update (%d feats of %d size)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size(),
            (int)Hx_big.rows());
  PRINT_ALL("[SLAM-UP]: %.4f seconds total\n", (rT3 - rT1).total_microseconds() * 1e-6);
}

void UpdaterSLAM::change_anchors(std::shared_ptr<State> state) {

  // 如果没有足够的克隆则返回
  if ((int)state->_clones_IMU.size() <= state->_options.max_clone_size) {
    return;
  }

  // 获取边缘化时间步，并更改从此时间步看到的任何特征的锚点
  // 注意：目前我们将特征锚定在与之前相同的相机中
  // 注意：这也不会改变特征的表示
  double marg_timestep = state->margtimestep();
  for (auto &f : state->_features_SLAM) {
    // 跳过全局坐标系中的任何特征
    if (f.second->_feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D ||
        f.second->_feat_representation == LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH)
      continue;
    // 否则检查它是否锚定在将被边缘化的克隆中
    assert(marg_timestep <= f.second->_anchor_clone_timestamp);
    if (f.second->_anchor_clone_timestamp == marg_timestep) {
      perform_anchor_change(state, f.second, state->_timestamp, f.second->_anchor_cam_id);
    }
  }
}

void UpdaterSLAM::perform_anchor_change(std::shared_ptr<State> state, std::shared_ptr<Landmark> landmark, double new_anchor_timestamp,
                                        size_t new_cam_id) {

  // 断言这是锚定表示
  assert(LandmarkRepresentation::is_relative_representation(landmark->_feat_representation));
  assert(landmark->_anchor_cam_id != -1);

  // 创建当前特征表示
  UpdaterHelper::UpdaterHelperFeature old_feat;
  old_feat.featid = landmark->_featid;
  old_feat.feat_representation = landmark->_feat_representation;
  old_feat.anchor_cam_id = landmark->_anchor_cam_id;
  old_feat.anchor_clone_timestamp = landmark->_anchor_clone_timestamp;
  old_feat.p_FinA = landmark->get_xyz(false);
  old_feat.p_FinA_fej = landmark->get_xyz(true);

  // 获取p_FinG相对于旧表示的雅可比矩阵
  Eigen::MatrixXd H_f_old;
  std::vector<Eigen::MatrixXd> H_x_old;
  std::vector<std::shared_ptr<Type>> x_order_old;
  UpdaterHelper::get_feature_jacobian_representation(state, old_feat, H_f_old, H_x_old, x_order_old);

  // 创建未来特征表示
  UpdaterHelper::UpdaterHelperFeature new_feat;
  new_feat.featid = landmark->_featid;
  new_feat.feat_representation = landmark->_feat_representation;
  new_feat.anchor_cam_id = new_cam_id;
  new_feat.anchor_clone_timestamp = new_anchor_timestamp;

  //==========================================================================
  //==========================================================================

  // 旧：锚点相机位置和方向
  Eigen::Matrix<double, 3, 3> R_GtoIOLD = state->_clones_IMU.at(old_feat.anchor_clone_timestamp)->Rot();
  Eigen::Matrix<double, 3, 3> R_GtoOLD = state->_calib_IMUtoCAM.at(old_feat.anchor_cam_id)->Rot() * R_GtoIOLD;
  Eigen::Matrix<double, 3, 1> p_OLDinG = state->_clones_IMU.at(old_feat.anchor_clone_timestamp)->pos() -
                                         R_GtoOLD.transpose() * state->_calib_IMUtoCAM.at(old_feat.anchor_cam_id)->pos();

  // 新：锚点相机位置和方向
  Eigen::Matrix<double, 3, 3> R_GtoINEW = state->_clones_IMU.at(new_feat.anchor_clone_timestamp)->Rot();
  Eigen::Matrix<double, 3, 3> R_GtoNEW = state->_calib_IMUtoCAM.at(new_feat.anchor_cam_id)->Rot() * R_GtoINEW;
  Eigen::Matrix<double, 3, 1> p_NEWinG = state->_clones_IMU.at(new_feat.anchor_clone_timestamp)->pos() -
                                         R_GtoNEW.transpose() * state->_calib_IMUtoCAM.at(new_feat.anchor_cam_id)->pos();

  // 计算旧锚点和新锚点之间的变换
  Eigen::Matrix<double, 3, 3> R_OLDtoNEW = R_GtoNEW * R_GtoOLD.transpose();
  Eigen::Matrix<double, 3, 1> p_OLDinNEW = R_GtoNEW * (p_OLDinG - p_NEWinG);
  new_feat.p_FinA = R_OLDtoNEW * landmark->get_xyz(false) + p_OLDinNEW;

  //==========================================================================
  //==========================================================================

  // 旧：锚点相机位置和方向（FEJ）
  Eigen::Matrix<double, 3, 3> R_GtoIOLD_fej = state->_clones_IMU.at(old_feat.anchor_clone_timestamp)->Rot_fej();
  Eigen::Matrix<double, 3, 3> R_GtoOLD_fej = state->_calib_IMUtoCAM.at(old_feat.anchor_cam_id)->Rot() * R_GtoIOLD_fej;
  Eigen::Matrix<double, 3, 1> p_OLDinG_fej = state->_clones_IMU.at(old_feat.anchor_clone_timestamp)->pos_fej() -
                                             R_GtoOLD_fej.transpose() * state->_calib_IMUtoCAM.at(old_feat.anchor_cam_id)->pos();

  // 新：锚点相机位置和方向（FEJ）
  Eigen::Matrix<double, 3, 3> R_GtoINEW_fej = state->_clones_IMU.at(new_feat.anchor_clone_timestamp)->Rot_fej();
  Eigen::Matrix<double, 3, 3> R_GtoNEW_fej = state->_calib_IMUtoCAM.at(new_feat.anchor_cam_id)->Rot() * R_GtoINEW_fej;
  Eigen::Matrix<double, 3, 1> p_NEWinG_fej = state->_clones_IMU.at(new_feat.anchor_clone_timestamp)->pos_fej() -
                                             R_GtoNEW_fej.transpose() * state->_calib_IMUtoCAM.at(new_feat.anchor_cam_id)->pos();

  // 计算旧锚点和新锚点之间的变换（FEJ）
  Eigen::Matrix<double, 3, 3> R_OLDtoNEW_fej = R_GtoNEW_fej * R_GtoOLD_fej.transpose();
  Eigen::Matrix<double, 3, 1> p_OLDinNEW_fej = R_GtoNEW_fej * (p_OLDinG_fej - p_NEWinG_fej);
  new_feat.p_FinA_fej = R_OLDtoNEW_fej * landmark->get_xyz(true) + p_OLDinNEW_fej;

  // 获取p_FinG相对于新表示的雅可比矩阵
  Eigen::MatrixXd H_f_new;
  std::vector<Eigen::MatrixXd> H_x_new;
  std::vector<std::shared_ptr<Type>> x_order_new;
  UpdaterHelper::get_feature_jacobian_representation(state, new_feat, H_f_new, H_x_new, x_order_new);

  //==========================================================================
  //==========================================================================

  // 新的phi顺序只是地标
  std::vector<std::shared_ptr<Type>> phi_order_NEW;
  phi_order_NEW.push_back(landmark);

  // 遍历所有顺序并添加它们
  std::vector<std::shared_ptr<Type>> phi_order_OLD;
  int current_it = 0;
  std::map<std::shared_ptr<Type>, int> Phi_id_map;
  for (const auto &var : x_order_old) {
    if (Phi_id_map.find(var) == Phi_id_map.end()) {
      Phi_id_map.insert({var, current_it});
      phi_order_OLD.push_back(var);
      current_it += var->size();
    }
  }
  for (const auto &var : x_order_new) {
    if (Phi_id_map.find(var) == Phi_id_map.end()) {
      Phi_id_map.insert({var, current_it});
      phi_order_OLD.push_back(var);
      current_it += var->size();
    }
  }
  Phi_id_map.insert({landmark, current_it});
  phi_order_OLD.push_back(landmark);
  current_it += landmark->size();

  // 锚点更改雅可比矩阵
  int phisize = (new_feat.feat_representation != LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 3 : 1;
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(phisize, current_it);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(phisize, phisize);

  // 新表示的逆
  // pf_new_error = Hfnew^{-1}*(Hfold*pf_olderror+Hxold*x_olderror-Hxnew*x_newerror)
  Eigen::MatrixXd H_f_new_inv;
  if (phisize == 1) {
    H_f_new_inv = 1.0 / H_f_new.squaredNorm() * H_f_new.transpose();
  } else {
    H_f_new_inv = H_f_new.colPivHouseholderQr().solve(Eigen::Matrix<double, 3, 3>::Identity());
  }

  // 放置旧锚点的雅可比矩阵
  for (size_t i = 0; i < H_x_old.size(); i++) {
    Phi.block(0, Phi_id_map.at(x_order_old[i]), phisize, x_order_old[i]->size()).noalias() += H_f_new_inv * H_x_old[i];
  }

  // 放置旧特征的雅可比矩阵
  Phi.block(0, Phi_id_map.at(landmark), phisize, phisize) = H_f_new_inv * H_f_old;

  // 放置新锚点的雅可比矩阵
  for (size_t i = 0; i < H_x_new.size(); i++) {
    Phi.block(0, Phi_id_map.at(x_order_new[i]), phisize, x_order_new[i]->size()).noalias() -= H_f_new_inv * H_x_new[i];
  }

  // 执行协方差传播
  StateHelper::EKFPropagation(state, phi_order_NEW, phi_order_OLD, Phi, Q);

  // 从新特征设置状态
  landmark->_featid = new_feat.featid;
  landmark->_feat_representation = new_feat.feat_representation;
  landmark->_anchor_cam_id = new_feat.anchor_cam_id;
  landmark->_anchor_clone_timestamp = new_feat.anchor_clone_timestamp;
  landmark->set_from_xyz(new_feat.p_FinA, false);
  landmark->set_from_xyz(new_feat.p_FinA_fej, true);
  landmark->has_had_anchor_change = true;
}
