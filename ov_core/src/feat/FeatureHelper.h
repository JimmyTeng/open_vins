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

#ifndef OV_CORE_FEATURE_HELPER_H
#define OV_CORE_FEATURE_HELPER_H

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include "Feature.h"
#include "FeatureDatabase.h"
#include "cam/CamBase.h"
#include "utils/colors.h"
#include "utils/print.h"

#include <unordered_map>

namespace ov_core {

/**
 * @brief Contains some nice helper functions for features.
 *
 * These functions should only depend on feature and the feature database.
 */
class FeatureHelper {

public:
  /**
   * @brief This functions will compute the disparity between common features in the two frames.
   *
   * First we find all features in the first frame.
   * Then we loop through each and find the uv of it in the next requested frame.
   * Features are skipped if no tracked feature is found (it was lost).
   * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
   * NOTE: This computes the disparity over all cameras!
   *
   * @param db Feature database pointer
   * @param time0 First camera frame timestamp
   * @param time1 Second camera frame timestamp
   * @param disp_mean Average raw disparity
   * @param disp_var Variance of the disparities
   * @param total_feats Total number of common features
   */
  static void compute_disparity(std::shared_ptr<ov_core::FeatureDatabase> db, double time0, double time1, double &disp_mean,
                                double &disp_var, int &total_feats) {

    // Get features seen from the first image
    std::vector<std::shared_ptr<Feature>> feats0 = db->features_containing(time0, false, true);

    // Compute the disparity
    std::vector<double> disparities;
    for (auto &feat : feats0) {

      // Get the two uvs for both times
      for (auto &campairs : feat->timestamps) {

        // First find the two timestamps
        size_t camid = campairs.first;
        auto it0 = std::find(feat->timestamps.at(camid).begin(), feat->timestamps.at(camid).end(), time0);
        auto it1 = std::find(feat->timestamps.at(camid).begin(), feat->timestamps.at(camid).end(), time1);
        if (it0 == feat->timestamps.at(camid).end() || it1 == feat->timestamps.at(camid).end())
          continue;
        auto idx0 = std::distance(feat->timestamps.at(camid).begin(), it0);
        auto idx1 = std::distance(feat->timestamps.at(camid).begin(), it1);

        // Now lets calculate the disparity
        Eigen::Vector2f uv0 = feat->uvs.at(camid).at(idx0).block(0, 0, 2, 1);
        Eigen::Vector2f uv1 = feat->uvs.at(camid).at(idx1).block(0, 0, 2, 1);
        disparities.push_back((uv1 - uv0).norm());
      }
    }

    // If no disparities, just return
    if (disparities.size() < 2) {
      disp_mean = -1;
      disp_var = -1;
      total_feats = 0;
    }

    // Compute mean and standard deviation in respect to it
    disp_mean = 0;
    for (double disp_i : disparities) {
      disp_mean += disp_i;
    }
    disp_mean /= (double)disparities.size();
    disp_var = 0;
    for (double &disp_i : disparities) {
      disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
    total_feats = (int)disparities.size();
  }

  /**
   * @brief This functions will compute the disparity over all features we have
   *
   * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
   * NOTE: This computes the disparity over all cameras!
   *
   * @param db Feature database pointer
   * @param disp_mean Average raw disparity
   * @param disp_var Variance of the disparities
   * @param total_feats Total number of common features
   * @param newest_time Only compute disparity for ones older (-1 to disable)
   * @param oldest_time Only compute disparity for ones newer (-1 to disable)
   */
  static void compute_disparity(std::shared_ptr<ov_core::FeatureDatabase> db, double &disp_mean, double &disp_var, int &total_feats,
                                double newest_time = -1, double oldest_time = -1) {

    // Compute the disparity
    std::vector<double> disparities;
    int tracks_total = 0;
    int tracks_with_history = 0;
    int tracks_time_window_matched = 0;
    for (auto &feat : db->get_internal_data()) {
      for (auto &campairs : feat.second->timestamps) {
        tracks_total++;

        // Skip if only one observation
        if (campairs.second.size() < 2)
          continue;
        tracks_with_history++;

        // Now lets calculate the disparity (assumes time array is monotonic)
        size_t camid = campairs.first;
        bool found0 = false;
        bool found1 = false;
        Eigen::Vector2f uv0 = Eigen::Vector2f::Zero();
        Eigen::Vector2f uv1 = Eigen::Vector2f::Zero();
        for (size_t idx = 0; idx < feat.second->timestamps.at(camid).size(); idx++) {
          double time = feat.second->timestamps.at(camid).at(idx);
          if ((oldest_time == -1 || time > oldest_time) && !found0) {
            uv0 = feat.second->uvs.at(camid).at(idx).block(0, 0, 2, 1);
            found0 = true;
            continue;
          }
          if ((newest_time == -1 || time < newest_time) && found0) {
            uv1 = feat.second->uvs.at(camid).at(idx).block(0, 0, 2, 1);
            found1 = true;
            continue;
          }
        }

        // If we found both an old and a new time, then we are good!
        if (!found0 || !found1)
          continue;
        tracks_time_window_matched++;
        disparities.push_back((uv1 - uv0).norm());
      }
    }
    PRINT_INFO(CYAN
               "[FeatureHelper::compute_disparity] 过程: total_tracks=%d, with_history=%d, "
               "window_matched=%d, disparities=%zu, newest=%.6f, oldest=%.6f\n" RESET,
               tracks_total, tracks_with_history, tracks_time_window_matched,
               disparities.size(), newest_time, oldest_time);

    // If no disparities, just return
    if (disparities.size() < 2) {
      disp_mean = -1;
      disp_var = -1;
      total_feats = 0;
      PRINT_INFO(
          YELLOW
          "[FeatureHelper::compute_disparity] 结果: 样本不足（%zu<2）, mean=%.3f, var=%.3f, total=%d\n"
          RESET,
          disparities.size(), disp_mean, disp_var, total_feats);
      return;
    }

    // Compute mean and standard deviation in respect to it
    disp_mean = 0;
    for (double disp_i : disparities) {
      disp_mean += disp_i;
    }
    disp_mean /= (double)disparities.size();
    disp_var = 0;
    for (double &disp_i : disparities) {
      disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
    total_feats = (int)disparities.size();
    PRINT_INFO(CYAN
               "[FeatureHelper::compute_disparity] 结果: mean=%.6f px, std=%.6f px, total=%d\n"
               RESET,
               disp_mean, disp_var, total_feats);
  }

  /**
   * @brief 绕相机光轴（视轴）旋转：像平面用向量 r（相对主点）、f（位移）分析。
   *
   * - 垂直度：perp_score = 1 - |cos∠(r,f)|，f 应近似沿切向故与 r 垂直。
   * - 方向一致性：二维叉积 s = r×f = r.x*f.y - r.y*f.x（标量，与视轴角速度同号），
   *   刚体绕固定光轴转动时各点 s 应同号；输出 sign_consistency = 与多数符号一致的样本占比。
   *
   * 仅使用距主点径向距离 **大于** min_r_px 像素的点；|f| 过小跳过。
   *
   * @param mean_perp 垂直度均值 [0,1]；-1 无样本
   * @param sign_consistency [0,1] 旋转方向一致比例；-1 无有效叉积符号
   * @param n_used 样本数
   */
  static void compute_optical_axis_flow_confidence(
      std::shared_ptr<FeatureDatabase> db, double time0, double time1,
      const std::unordered_map<size_t, std::shared_ptr<CamBase>> &cams,
      double min_r_px, double min_flow_px, double &mean_perp,
      double &sign_consistency, int &n_used) {

    mean_perp = -1.0;
    sign_consistency = -1.0;
    n_used = 0;
    if (db == nullptr || cams.empty())
      return;

    std::vector<double> perp_scores;
    std::vector<float> cross_zs;
    std::vector<std::shared_ptr<Feature>> feats0 =
        db->features_containing(time0, false, true);

    for (auto &feat : feats0) {
      for (auto &campairs : feat->timestamps) {
        size_t camid = campairs.first;
        auto it_cam = cams.find(camid);
        if (it_cam == cams.end())
          continue;

        Eigen::MatrixXd calib = it_cam->second->get_value();
        double cx = calib(2, 0);
        double cy = calib(3, 0);

        auto it0 = std::find(feat->timestamps.at(camid).begin(),
                             feat->timestamps.at(camid).end(), time0);
        auto it1 = std::find(feat->timestamps.at(camid).begin(),
                             feat->timestamps.at(camid).end(), time1);
        if (it0 == feat->timestamps.at(camid).end() ||
            it1 == feat->timestamps.at(camid).end())
          continue;
        auto idx0 = std::distance(feat->timestamps.at(camid).begin(), it0);
        auto idx1 = std::distance(feat->timestamps.at(camid).begin(), it1);

        Eigen::Vector2f uv0 = feat->uvs.at(camid).at(idx0).block(0, 0, 2, 1);
        Eigen::Vector2f uv1 = feat->uvs.at(camid).at(idx1).block(0, 0, 2, 1);
        Eigen::Vector2f f = uv1 - uv0;
        Eigen::Vector2f mid = 0.5f * (uv0 + uv1);
        Eigen::Vector2f r(mid(0) - static_cast<float>(cx),
                          mid(1) - static_cast<float>(cy));

        float rnorm = r.norm();
        float fnorm = f.norm();
        if (rnorm <= static_cast<float>(min_r_px) ||
            fnorm < static_cast<float>(min_flow_px))
          continue;

        float c = std::abs(r.dot(f)) / (rnorm * fnorm + 1e-9f);
        c = std::min(1.0f, std::max(0.0f, c));
        float perp_score = 1.0f - c;
        // r × f（朝视轴正方向的 z 分量，与绕光轴角速度符号一致）
        float cross_z = r.x() * f.y() - r.y() * f.x();

        perp_scores.push_back((double)perp_score);
        cross_zs.push_back(cross_z);
      }
    }

    const size_t n = perp_scores.size();
    if (n < 1) {
      mean_perp = -1.0;
      sign_consistency = -1.0;
      n_used = 0;
      return;
    }

    n_used = (int)n;
    mean_perp = 0.0;
    for (double s : perp_scores)
      mean_perp += s;
    mean_perp /= (double)n;

    // 方向一致性：s = r×f 的符号应与同一刚体绕光轴转动一致；对明确非零叉积做多数决
    const float eps_cross =
        std::max(1e-9f, static_cast<float>(min_flow_px * 0.02));
    std::vector<int> signs;
    signs.reserve(cross_zs.size());
    for (float cz : cross_zs) {
      if (cz > eps_cross)
        signs.push_back(1);
      else if (cz < -eps_cross)
        signs.push_back(-1);
    }
    if (signs.empty()) {
      sign_consistency = 0.0;
      return;
    }
    int n_plus = 0, n_minus = 0;
    for (int s : signs) {
      if (s > 0)
        n_plus++;
      else
        n_minus++;
    }
    const int ref_sign = (n_plus >= n_minus) ? 1 : -1;
    int n_agree = 0;
    for (int s : signs) {
      if (s == ref_sign)
        n_agree++;
    }
    sign_consistency = (double)n_agree / (double)signs.size();
  }

  /**
   * @brief 在指定相机上收集同时见于 time_ref 与 time_cur 的特征，并输出去畸变归一化像面坐标（z=1）
   * @return 匹配对数
   */
  static int collect_normalized_correspondences(
      std::shared_ptr<FeatureDatabase> db, double time_ref, double time_cur,
      size_t cam_id,
      const std::unordered_map<size_t, std::shared_ptr<CamBase>> &cams,
      std::vector<Eigen::Vector2d> &pts_ref,
      std::vector<Eigen::Vector2d> &pts_cur) {

    pts_ref.clear();
    pts_cur.clear();
    if (db == nullptr || cams.empty())
      return 0;
    auto it_cam = cams.find(cam_id);
    if (it_cam == cams.end())
      return 0;

    std::vector<std::shared_ptr<Feature>> feats0 =
        db->features_containing(time_ref, false, true);

    for (auto &feat : feats0) {
      auto it_ts = feat->timestamps.find(cam_id);
      if (it_ts == feat->timestamps.end())
        continue;
      auto it0 = std::find(it_ts->second.begin(), it_ts->second.end(),
                           time_ref);
      auto it1 = std::find(it_ts->second.begin(), it_ts->second.end(),
                           time_cur);
      if (it0 == it_ts->second.end() || it1 == it_ts->second.end())
        continue;
      auto idx0 = std::distance(it_ts->second.begin(), it0);
      auto idx1 = std::distance(it_ts->second.begin(), it1);
      Eigen::Vector2f uv_ref =
          feat->uvs.at(cam_id).at(idx0).block(0, 0, 2, 1);
      Eigen::Vector2f uv_cur =
          feat->uvs.at(cam_id).at(idx1).block(0, 0, 2, 1);
      Eigen::Vector2f n_ref = it_cam->second->undistort_f(uv_ref);
      Eigen::Vector2f n_cur = it_cam->second->undistort_f(uv_cur);
      pts_ref.emplace_back(n_ref.cast<double>());
      pts_cur.emplace_back(n_cur.cast<double>());
    }
    return (int)pts_ref.size();
  }

private:
  // Cannot construct this class
  FeatureHelper() {}
};

} // namespace ov_core

#endif /* OV_CORE_FEATURE_HELPER_H */