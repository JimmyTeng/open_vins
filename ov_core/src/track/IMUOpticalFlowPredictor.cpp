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

#include "IMUOpticalFlowPredictor.h"
#include "utils/print.h"

using namespace ov_core;

IMUOpticalFlowPredictor::IMUOpticalFlowPredictor(const Eigen::Matrix3d &R_ItoC, std::shared_ptr<CamBase> camera)
    : R_ItoC(R_ItoC), camera(camera) {
  // Validate inputs
  if (camera == nullptr) {
    PRINT_ERROR("[IMU-PREDICTOR]: Camera pointer is null!\n");
    std::exit(EXIT_FAILURE);
  }
}

void IMUOpticalFlowPredictor::feed_imu(const ImuData &imu_data) {
  std::lock_guard<std::mutex> lck(imu_data_mtx);
  this->imu_data.push_back(imu_data);

  // Keep buffer sorted by timestamp
  std::sort(this->imu_data.begin(), this->imu_data.end());

  // Clean old measurements
  if (!this->imu_data.empty() && last_frame_timestamp > 0) {
    double oldest_time = last_frame_timestamp - max_imu_buffer_time;
    clean_old_imu_measurements(oldest_time);
  }
  
  // Debug: print buffer size occasionally
  static int feed_count = 0;
  feed_count++;
  if (feed_count % 200 == 0) {
    PRINT_DEBUG("[IMU-PREDICTOR] Buffer size: %zu, last_frame_ts: %.6f, imu_ts: %.6f\n",
                this->imu_data.size(), last_frame_timestamp, imu_data.timestamp);
  }
}

bool IMUOpticalFlowPredictor::predict_points(double timestamp_new, const std::vector<cv::Point2f> &pts_old,
                                              std::vector<cv::Point2f> &pts_predicted) {
  // Check if we have last frame timestamp
  if (last_frame_timestamp < 0) {
    // First frame, just copy old points
    PRINT_DEBUG("[IMU-PREDICTOR] First frame, no prediction (last_frame_ts=%.6f, new_ts=%.6f)\n",
                last_frame_timestamp, timestamp_new);
    pts_predicted = pts_old;
    return false;
  }

  // Get rotation between frames
  Eigen::Matrix3d R_CtoC;
  if (!get_rotation_between_frames(last_frame_timestamp, timestamp_new, R_CtoC)) {
    // Failed to get rotation, use old points
    PRINT_DEBUG("[IMU-PREDICTOR] Failed to get rotation (old_ts=%.6f, new_ts=%.6f, dt=%.6f)\n",
                last_frame_timestamp, timestamp_new, timestamp_new - last_frame_timestamp);
    pts_predicted = pts_old;
    return false;
  }

  // Predict using rotation
  bool success = predict_points_with_rotation(R_CtoC, pts_old, pts_predicted);
  if (success) {
    PRINT_DEBUG("[IMU-PREDICTOR] Prediction successful: %zu points, dt=%.6f\n",
                pts_predicted.size(), timestamp_new - last_frame_timestamp);
  } else {
    PRINT_DEBUG("[IMU-PREDICTOR] Prediction failed in predict_points_with_rotation\n");
  }
  return success;
}

bool IMUOpticalFlowPredictor::predict_points_with_rotation(const Eigen::Matrix3d &R_CtoC,
                                                            const std::vector<cv::Point2f> &pts_old,
                                                            std::vector<cv::Point2f> &pts_predicted) {
  pts_predicted.clear();
  pts_predicted.reserve(pts_old.size());

  for (const auto &pt_old : pts_old) {
    // Step 1: Undistort point to normalized coordinates
    Eigen::Vector2d pt_old_eigen(pt_old.x, pt_old.y);
    Eigen::Vector2d pt_norm_old = camera->undistort_d(pt_old_eigen);

    // Step 2: Convert to 3D ray (assuming unit depth)
    Eigen::Vector3d ray_old;
    ray_old << pt_norm_old(0), pt_norm_old(1), 1.0;
    ray_old.normalize();

    // Step 3: Apply rotation to get new ray direction
    Eigen::Vector3d ray_new = R_CtoC * ray_old;

    // Step 4: Project back to normalized coordinates
    if (std::abs(ray_new(2)) < 1e-6) {
      // Point is behind camera or at infinity, use old position
      pts_predicted.push_back(pt_old);
      continue;
    }

    Eigen::Vector2d pt_norm_new;
    pt_norm_new << ray_new(0) / ray_new(2), ray_new(1) / ray_new(2);

    // Step 5: Distort and project to pixel coordinates
    Eigen::Vector2f pt_norm_new_f = pt_norm_new.cast<float>();
    Eigen::Vector2f pt_dist_new = camera->distort_f(pt_norm_new_f);

    // Step 6: Check bounds
    if (pt_dist_new(0) < 0 || pt_dist_new(0) >= camera->w() || pt_dist_new(1) < 0 || pt_dist_new(1) >= camera->h()) {
      // Out of bounds, use old position
      pts_predicted.push_back(pt_old);
      continue;
    }

    pts_predicted.emplace_back(pt_dist_new(0), pt_dist_new(1));
  }

  return true;
}

bool IMUOpticalFlowPredictor::get_rotation_between_frames(double timestamp_old, double timestamp_new,
                                                          Eigen::Matrix3d &R_CtoC) {
  // Get IMU readings between frames
  std::vector<ImuData> imu_readings = select_imu_readings(timestamp_old, timestamp_new);

  if (imu_readings.size() < 2) {
    // Not enough IMU data
    PRINT_DEBUG("[IMU-PREDICTOR] Not enough IMU readings: %zu (need >=2), time range [%.6f, %.6f]\n",
                imu_readings.size(), timestamp_old, timestamp_new);
    return false;
  }

  // Integrate angular velocity to get rotation in IMU frame
  Eigen::Matrix3d R_ItoI;
  if (!integrate_angular_velocity(imu_readings, R_ItoI)) {
    PRINT_DEBUG("[IMU-PREDICTOR] Failed to integrate angular velocity\n");
    return false;
  }

  // Transform rotation from IMU frame to Camera frame
  // We want: R_Cold_to_Cnew (rotation from old camera frame to new camera frame)
  // 
  // Coordinate transformation chain:
  // Step 1: Old camera frame -> Old IMU frame: R_Cold_to_Iold = R_ItoC^T
  // Step 2: Old IMU frame -> New IMU frame: R_Iold_to_Inew = R_ItoI
  // Step 3: New IMU frame -> New camera frame: R_Inew_to_Cnew = R_ItoC
  //
  // Therefore: R_Cold_to_Cnew = R_ItoC * R_ItoI * R_ItoC^T
  // This transforms: C_old -> I_old -> I_new -> C_new
  Eigen::Matrix3d R_CtoI = R_ItoC.transpose();  // R_Cold_to_Iold = R_ItoC^T
  R_CtoC = R_ItoC * R_ItoI * R_CtoI;  // R_Cold_to_Cnew = R_ItoC * R_ItoI * R_ItoC^T

  PRINT_DEBUG("[IMU-PREDICTOR] Got rotation: %zu IMU readings, dt=%.6f\n",
              imu_readings.size(), timestamp_new - timestamp_old);
  return true;
}

bool IMUOpticalFlowPredictor::integrate_angular_velocity(const std::vector<ImuData> &imu_readings,
                                                          Eigen::Matrix3d &R_ItoI) {
  if (imu_readings.size() < 2) {
    return false;
  }

  // Initialize rotation as identity
  R_ItoI = Eigen::Matrix3d::Identity();

  // Integrate using zero-order hold (constant angular velocity between measurements)
  for (size_t i = 0; i < imu_readings.size() - 1; i++) {
    const auto &data_minus = imu_readings[i];
    const auto &data_plus = imu_readings[i + 1];

    double dt = data_plus.timestamp - data_minus.timestamp;
    if (dt <= 0) {
      continue;
    }

    // Average angular velocity
    Eigen::Vector3d w_avg = 0.5 * (data_minus.wm + data_plus.wm);

    // Compute rotation increment using exponential map
    // R = exp(w * dt) where w is angular velocity
    Eigen::Vector3d w_dt = w_avg * dt;
    Eigen::Matrix3d dR = exp_so3(w_dt);

    // Update rotation
    R_ItoI = dR * R_ItoI;
  }

  return true;
}

std::vector<ImuData> IMUOpticalFlowPredictor::select_imu_readings(double timestamp_start, double timestamp_end) {
  std::lock_guard<std::mutex> lck(imu_data_mtx);

  std::vector<ImuData> selected;
  for (const auto &data : imu_data) {
    if (data.timestamp >= timestamp_start && data.timestamp <= timestamp_end) {
      selected.push_back(data);
    }
  }

  // Also include one measurement before and after if available
  if (!imu_data.empty()) {
    // Find measurement just before start
    for (auto it = imu_data.rbegin(); it != imu_data.rend(); ++it) {
      if (it->timestamp < timestamp_start) {
        selected.insert(selected.begin(), *it);
        break;
      }
    }

    // Find measurement just after end
    for (const auto &data : imu_data) {
      if (data.timestamp > timestamp_end) {
        selected.push_back(data);
        break;
      }
    }
  }

  // Sort by timestamp
  std::sort(selected.begin(), selected.end());

  return selected;
}

void IMUOpticalFlowPredictor::clean_old_imu_measurements(double oldest_time) {
  std::lock_guard<std::mutex> lck(imu_data_mtx);

  // Remove measurements older than oldest_time
  while (!imu_data.empty() && imu_data.front().timestamp < oldest_time) {
    imu_data.pop_front();
  }
}
