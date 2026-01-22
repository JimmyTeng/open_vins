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

#ifndef OV_CORE_IMU_OPTICAL_FLOW_PREDICTOR_H
#define OV_CORE_IMU_OPTICAL_FLOW_PREDICTOR_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <vector>
#include <deque>
#include "utils/sensor_data.h"
#include "cam/CamBase.h"
#include "utils/quat_ops.h"

namespace ov_core {

/**
 * @brief IMU-based optical flow prediction for feature tracking
 *
 * This class uses IMU angular velocity measurements to predict feature point locations
 * in the next frame, improving optical flow tracking performance especially during
 * fast camera rotations.
 *
 * The prediction process:
 * 1. Integrate IMU angular velocity to get rotation between frames
 * 2. Transform rotation from IMU frame to camera frame
 * 3. Project feature points from previous frame to new frame using the rotation
 * 4. Return predicted pixel locations for optical flow initialization
 */
class IMUOpticalFlowPredictor {

public:
  /**
   * @brief Constructor
   * @param R_ItoC Rotation matrix from IMU to Camera frame (3x3)
   * @param camera Camera calibration object for projection
   */
  IMUOpticalFlowPredictor(const Eigen::Matrix3d &R_ItoC, std::shared_ptr<CamBase> camera);

  /**
   * @brief Feed IMU measurement
   * @param imu_data IMU measurement with timestamp, angular velocity, and acceleration
   */
  void feed_imu(const ImuData &imu_data);

  /**
   * @brief Predict feature point locations in new frame using IMU rotation
   * @param timestamp_new Timestamp of the new frame
   * @param pts_old Feature points in the previous frame (pixel coordinates)
   * @param pts_predicted Output: Predicted feature points in the new frame (pixel coordinates)
   * @return true if prediction successful, false otherwise
   *
   * This function:
   * 1. Finds IMU measurements between last frame and new frame
   * 2. Integrates angular velocity to compute rotation
   * 3. Projects points from old frame to new frame
   */
  bool predict_points(double timestamp_new, const std::vector<cv::Point2f> &pts_old,
                      std::vector<cv::Point2f> &pts_predicted);

  /**
   * @brief Predict feature point locations using given rotation
   * @param R_CtoC Rotation from old camera frame to new camera frame (3x3)
   * @param pts_old Feature points in the previous frame (pixel coordinates)
   * @param pts_predicted Output: Predicted feature points in the new frame (pixel coordinates)
   * @return true if prediction successful, false otherwise
   */
  bool predict_points_with_rotation(const Eigen::Matrix3d &R_CtoC, const std::vector<cv::Point2f> &pts_old,
                                     std::vector<cv::Point2f> &pts_predicted);

  /**
   * @brief Get the rotation between two timestamps
   * @param timestamp_old Timestamp of old frame
   * @param timestamp_new Timestamp of new frame
   * @param R_CtoC Output: Rotation from old camera frame to new camera frame
   * @return true if rotation computed successfully, false otherwise
   */
  bool get_rotation_between_frames(double timestamp_old, double timestamp_new, Eigen::Matrix3d &R_CtoC);

  /**
   * @brief Clean old IMU measurements
   * @param oldest_time Keep measurements after this time
   */
  void clean_old_imu_measurements(double oldest_time);

  /**
   * @brief Set the last frame timestamp (for tracking)
   * @param timestamp Timestamp of the last processed frame
   */
  void set_last_frame_timestamp(double timestamp) {
    std::lock_guard<std::mutex> lck(imu_data_mtx);
    last_frame_timestamp = timestamp;
  }

  /**
   * @brief Get the last frame timestamp
   * @return Last frame timestamp
   */
  double get_last_frame_timestamp() const {
    std::lock_guard<std::mutex> lck(imu_data_mtx);
    return last_frame_timestamp;
  }

private:
  /**
   * @brief Integrate angular velocity to get rotation
   * @param imu_readings Vector of IMU readings between two timestamps
   * @param R_ItoI Output: Rotation from initial IMU frame to final IMU frame
   * @return true if integration successful
   */
  bool integrate_angular_velocity(const std::vector<ImuData> &imu_readings, Eigen::Matrix3d &R_ItoI);

  /**
   * @brief Select IMU readings between two timestamps
   * @param timestamp_start Start timestamp
   * @param timestamp_end End timestamp
   * @return Vector of IMU readings in the time interval
   */
  std::vector<ImuData> select_imu_readings(double timestamp_start, double timestamp_end);

  // Rotation from IMU to Camera frame
  Eigen::Matrix3d R_ItoC;

  // Camera calibration for projection
  std::shared_ptr<CamBase> camera;

  // IMU data buffer
  std::deque<ImuData> imu_data;
  mutable std::mutex imu_data_mtx;  // mutable to allow locking in const methods

  // Last frame timestamp
  double last_frame_timestamp = -1.0;

  // Maximum time to keep IMU data (seconds)
  double max_imu_buffer_time = 1.0;
};

} // namespace ov_core

#endif /* OV_CORE_IMU_OPTICAL_FLOW_PREDICTOR_H */
