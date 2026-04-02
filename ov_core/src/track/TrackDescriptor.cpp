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

#include "TrackDescriptor.h"

#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

#include "Grider_FAST.h"
#include "cam/CamBase.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"

using namespace ov_core;

void TrackDescriptor::feed_new_camera(const CameraData &message) {
  // Error check that we have all the data
  if (message.sensor_ids.empty() ||
      message.sensor_ids.size() != message.images.size() ||
      message.images.size() != message.masks.size()) {
    PRINT_ERROR(RED
                "[ERROR]: MESSAGE DATA SIZES DO NOT MATCH OR EMPTY!!!\n" RESET);
    PRINT_ERROR(RED "[ERROR]:   - message.sensor_ids.size() = %zu\n" RESET,
                message.sensor_ids.size());
    PRINT_ERROR(RED "[ERROR]:   - message.images.size() = %zu\n" RESET,
                message.images.size());
    PRINT_ERROR(RED "[ERROR]:   - message.masks.size() = %zu\n" RESET,
                message.masks.size());
    std::exit(EXIT_FAILURE);
  }

  // Either call our stereo or monocular version
  // If we are doing binocular tracking, then we should parallize our tracking
  size_t num_images = message.images.size();
  if (num_images == 1) {
    feed_monocular(message, 0);
  } else if (num_images == 2 && use_stereo) {
    feed_stereo(message, 0, 1);
  } else if (!use_stereo) {
    parallel_for_(cv::Range(0, (int)num_images),
                  LambdaBody([&](const cv::Range &range) {
                    for (int i = range.start; i < range.end; i++) {
                      feed_monocular(message, i);
                    }
                  }));
  } else {
    PRINT_ERROR(RED
                "[ERROR]: invalid number of images passed %zu, we only support "
                "mono or stereo tracking",
                num_images);
    std::exit(EXIT_FAILURE);
  }
}

void TrackDescriptor::feed_monocular(const CameraData &message, size_t msg_id) {
  // Start timing
  rT1 = rtime_now();

  // Lock this data feed for this camera
  size_t cam_id = message.sensor_ids.at(msg_id);
  std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));

  // Histogram equalize
  cv::Mat img, mask;
  if (histogram_method == HistogramMethod::HISTOGRAM) {
    cv::equalizeHist(message.images.at(msg_id), img);
  } else if (histogram_method == HistogramMethod::CLAHE) {
    double eq_clip_limit = 10.0;
    cv::Size eq_win_size = cv::Size(8, 8);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
    clahe->apply(message.images.at(msg_id), img);
  } else {
    img = message.images.at(msg_id);
  }
  mask = message.masks.at(msg_id);

  // If we are the first frame (or have lost tracking), initialize our
  // descriptors
  if (pts_last.find(cam_id) == pts_last.end() || pts_last[cam_id].empty()) {
    std::vector<cv::KeyPoint> good_left;
    std::vector<size_t> good_ids_left;
    cv::Mat good_desc_left;
    perform_detection_monocular(img, mask, good_left, good_desc_left,
                                good_ids_left);
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id] = img;
    img_mask_last[cam_id] = mask;
    pts_last[cam_id] = good_left;
    ids_last[cam_id] = good_ids_left;
    desc_last[cam_id] = good_desc_left;
    return;
  }

  // Our new keypoints and descriptor for the new image
  std::vector<cv::KeyPoint> pts_new;
  cv::Mat desc_new;
  std::vector<size_t> ids_new;

  // First, extract new descriptors for this new image
  perform_detection_monocular(img, mask, pts_new, desc_new, ids_new);
  rT2 = rtime_now();

  // Our matches temporally
  std::vector<cv::DMatch> matches_ll;

  // Lets match temporally
  robust_match(pts_last[cam_id], pts_new, desc_last[cam_id], desc_new, cam_id,
               cam_id, matches_ll);
  rT3 = rtime_now();

  // Get our "good tracks"
  std::vector<cv::KeyPoint> good_left;
  std::vector<size_t> good_ids_left;
  cv::Mat good_desc_left;

  // Count how many we have tracked from the last time
  int num_tracklast = 0;

  // Loop through all current left to right points
  // We want to see if any of theses have matches to the previous frame
  // If we have a match new->old then we want to use that ID instead of the new
  // one
  for (size_t i = 0; i < pts_new.size(); i++) {
    // Loop through all left matches, and find the old "train" id
    int idll = -1;
    for (size_t j = 0; j < matches_ll.size(); j++) {
      if (matches_ll[j].trainIdx == (int)i) {
        idll = matches_ll[j].queryIdx;
      }
    }

    // Then lets replace the current ID with the old ID if found
    // Else just append the current feature and its unique ID
    good_left.push_back(pts_new[i]);
    good_desc_left.push_back(desc_new.row((int)i));
    if (idll != -1) {
      good_ids_left.push_back(ids_last[cam_id][idll]);
      num_tracklast++;
    } else {
      good_ids_left.push_back(ids_new[i]);
    }
  }
  rT4 = rtime_now();

  // Update our feature database, with theses new observations
  for (size_t i = 0; i < good_left.size(); i++) {
    cv::Point2f npt_l =
        camera_calib.at(cam_id)->undistort_cv(good_left.at(i).pt);
    database->update_feature(good_ids_left.at(i), message.timestamp, cam_id,
                             good_left.at(i).pt.x, good_left.at(i).pt.y,
                             npt_l.x, npt_l.y);
  }

  // Debug info
  // PRINT_DEBUG("LtoL = %d | good = %d | fromlast =
  // %d\n",(int)matches_ll.size(),(int)good_left.size(),num_tracklast);

  // Move forward in time
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id] = img;
    img_mask_last[cam_id] = mask;
    pts_last[cam_id] = good_left;
    ids_last[cam_id] = good_ids_left;
    desc_last[cam_id] = good_desc_left;
  }
  rT5 = rtime_now();

  // Our timing information
  PRINT_ALL("[TIME-DESC]: %.2f ms for detection\n", rtime_ms(rT1, rT2));
  PRINT_ALL("[TIME-DESC]: %.2f ms for matching\n", rtime_ms(rT2, rT3));
  PRINT_ALL("[TIME-DESC]: %.2f ms for merging\n", rtime_ms(rT3, rT4));
  PRINT_ALL("[TIME-DESC]: %.2f ms for feature DB update (%d features)\n",
            rtime_ms(rT4, rT5), (int)good_left.size());
  PRINT_ALL("[TIME-DESC]: %.2f ms for total\n", rtime_ms(rT1, rT5));
}

void TrackDescriptor::feed_stereo(const CameraData &message, size_t msg_id_left,
                                  size_t msg_id_right) {
  // Start timing
  rT1 = rtime_now();

  // Lock this data feed for this camera
  size_t cam_id_left = message.sensor_ids.at(msg_id_left);
  size_t cam_id_right = message.sensor_ids.at(msg_id_right);
  std::lock_guard<std::mutex> lck1(mtx_feeds.at(cam_id_left));
  std::lock_guard<std::mutex> lck2(mtx_feeds.at(cam_id_right));

  // Histogram equalize images
  cv::Mat img_left, img_right, mask_left, mask_right;
  if (histogram_method == HistogramMethod::HISTOGRAM) {
    cv::equalizeHist(message.images.at(msg_id_left), img_left);
    cv::equalizeHist(message.images.at(msg_id_right), img_right);
  } else if (histogram_method == HistogramMethod::CLAHE) {
    double eq_clip_limit = 10.0;
    cv::Size eq_win_size = cv::Size(8, 8);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
    clahe->apply(message.images.at(msg_id_left), img_left);
    clahe->apply(message.images.at(msg_id_right), img_right);
  } else {
    img_left = message.images.at(msg_id_left);
    img_right = message.images.at(msg_id_right);
  }
  mask_left = message.masks.at(msg_id_left);
  mask_right = message.masks.at(msg_id_right);

  // If we are the first frame (or have lost tracking), initialize our
  // descriptors
  if (pts_last[cam_id_left].empty() || pts_last[cam_id_right].empty()) {
    std::vector<cv::KeyPoint> good_left, good_right;
    std::vector<size_t> good_ids_left, good_ids_right;
    cv::Mat good_desc_left, good_desc_right;
    perform_detection_stereo(img_left, img_right, mask_left, mask_right,
                             good_left, good_right, good_desc_left,
                             good_desc_right, cam_id_left, cam_id_right,
                             good_ids_left, good_ids_right);
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    pts_last[cam_id_left] = good_left;
    pts_last[cam_id_right] = good_right;
    ids_last[cam_id_left] = good_ids_left;
    ids_last[cam_id_right] = good_ids_right;
    desc_last[cam_id_left] = good_desc_left;
    desc_last[cam_id_right] = good_desc_right;
    return;
  }

  // Our new keypoints and descriptor for the new image
  std::vector<cv::KeyPoint> pts_left_new, pts_right_new;
  cv::Mat desc_left_new, desc_right_new;
  std::vector<size_t> ids_left_new, ids_right_new;

  // First, extract new descriptors for this new image
  perform_detection_stereo(img_left, img_right, mask_left, mask_right,
                           pts_left_new, pts_right_new, desc_left_new,
                           desc_right_new, cam_id_left, cam_id_right,
                           ids_left_new, ids_right_new);
  rT2 = rtime_now();

  // Our matches temporally
  std::vector<cv::DMatch> matches_ll, matches_rr;
  parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    bool is_left = (i == 0);
                    robust_match(
                        pts_last[is_left ? cam_id_left : cam_id_right],
                        is_left ? pts_left_new : pts_right_new,
                        desc_last[is_left ? cam_id_left : cam_id_right],
                        is_left ? desc_left_new : desc_right_new,
                        is_left ? cam_id_left : cam_id_right,
                        is_left ? cam_id_left : cam_id_right,
                        is_left ? matches_ll : matches_rr);
                  }
                }));
  rT3 = rtime_now();

  // Get our "good tracks"
  std::vector<cv::KeyPoint> good_left, good_right;
  std::vector<size_t> good_ids_left, good_ids_right;
  cv::Mat good_desc_left, good_desc_right;

  // Points must be of equal size
  assert(pts_last[cam_id_left].size() == pts_last[cam_id_right].size());
  assert(pts_left_new.size() == pts_right_new.size());

  // Count how many we have tracked from the last time
  int num_tracklast = 0;

  // Loop through all current left to right points
  // We want to see if any of theses have matches to the previous frame
  // If we have a match new->old then we want to use that ID instead of the new
  // one
  for (size_t i = 0; i < pts_left_new.size(); i++) {
    // Loop through all left matches, and find the old "train" id
    int idll = -1;
    for (size_t j = 0; j < matches_ll.size(); j++) {
      if (matches_ll[j].trainIdx == (int)i) {
        idll = matches_ll[j].queryIdx;
      }
    }

    // Loop through all left matches, and find the old "train" id
    int idrr = -1;
    for (size_t j = 0; j < matches_rr.size(); j++) {
      if (matches_rr[j].trainIdx == (int)i) {
        idrr = matches_rr[j].queryIdx;
      }
    }

    // If we found a good stereo track from left to left, and right to right
    // Then lets replace the current ID with the old ID
    // We also check that we are linked to the same past ID value
    if (idll != -1 && idrr != -1 &&
        ids_last[cam_id_left][idll] == ids_last[cam_id_right][idrr]) {
      good_left.push_back(pts_left_new[i]);
      good_right.push_back(pts_right_new[i]);
      good_desc_left.push_back(desc_left_new.row((int)i));
      good_desc_right.push_back(desc_right_new.row((int)i));
      good_ids_left.push_back(ids_last[cam_id_left][idll]);
      good_ids_right.push_back(ids_last[cam_id_right][idrr]);
      num_tracklast++;
    } else {
      // Else just append the current feature and its unique ID
      good_left.push_back(pts_left_new[i]);
      good_right.push_back(pts_right_new[i]);
      good_desc_left.push_back(desc_left_new.row((int)i));
      good_desc_right.push_back(desc_right_new.row((int)i));
      good_ids_left.push_back(ids_left_new[i]);
      good_ids_right.push_back(ids_left_new[i]);
    }
  }
  rT4 = rtime_now();

  //===================================================================================
  //===================================================================================

  // Update our feature database, with theses new observations
  for (size_t i = 0; i < good_left.size(); i++) {
    // Assert that our IDs are the same
    assert(good_ids_left.at(i) == good_ids_right.at(i));
    // Try to undistort the point
    cv::Point2f npt_l =
        camera_calib.at(cam_id_left)->undistort_cv(good_left.at(i).pt);
    cv::Point2f npt_r =
        camera_calib.at(cam_id_right)->undistort_cv(good_right.at(i).pt);
    // Append to the database
    database->update_feature(good_ids_left.at(i), message.timestamp,
                             cam_id_left, good_left.at(i).pt.x,
                             good_left.at(i).pt.y, npt_l.x, npt_l.y);
    database->update_feature(good_ids_left.at(i), message.timestamp,
                             cam_id_right, good_right.at(i).pt.x,
                             good_right.at(i).pt.y, npt_r.x, npt_r.y);
  }

  // Debug info
  // PRINT_DEBUG("LtoL = %d | RtoR = %d | LtoR = %d | good = %d | fromlast =
  // %d\n", (int)matches_ll.size(),
  //       (int)matches_rr.size(),(int)ids_left_new.size(),(int)good_left.size(),num_tracklast);

  // Move forward in time
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    pts_last[cam_id_left] = good_left;
    pts_last[cam_id_right] = good_right;
    ids_last[cam_id_left] = good_ids_left;
    ids_last[cam_id_right] = good_ids_right;
    desc_last[cam_id_left] = good_desc_left;
    desc_last[cam_id_right] = good_desc_right;
  }
  rT5 = rtime_now();

  // Our timing information
  PRINT_ALL("[TIME-DESC]: %.2f ms for detection\n", rtime_ms(rT1, rT2));
  PRINT_ALL("[TIME-DESC]: %.2f ms for matching\n", rtime_ms(rT2, rT3));
  PRINT_ALL("[TIME-DESC]: %.2f ms for merging\n", rtime_ms(rT3, rT4));
  PRINT_ALL("[TIME-DESC]: %.2f ms for feature DB update (%d features)\n",
            rtime_ms(rT4, rT5), (int)good_left.size());
  PRINT_ALL("[TIME-DESC]: %.2f ms for total\n", rtime_ms(rT1, rT5));
}

void TrackDescriptor::perform_detection_monocular(
    const cv::Mat &img0, const cv::Mat &mask0, std::vector<cv::KeyPoint> &pts0,
    cv::Mat &desc0, std::vector<size_t> &ids0) {
  // Assert that we need features
  assert(pts0.empty());

  // Extract our features (use FAST with griding)
  std::vector<cv::KeyPoint> pts0_ext;
  Grider_FAST::perform_griding(img0, mask0, pts0_ext, num_features, grid_x,
                               grid_y, threshold, true);

  // For all new points, extract their descriptors
  cv::Mat desc0_ext;
  this->orb0->compute(img0, pts0_ext, desc0_ext);

  // Create a 2D occupancy grid for this current image
  // Note that we scale this down, so that each grid point is equal to a set of
  // pixels This means that we will reject points that less then grid_px_size
  // points away then existing features
  cv::Size size((int)((float)img0.cols / (float)min_px_dist),
                (int)((float)img0.rows / (float)min_px_dist));
  cv::Mat grid_2d = cv::Mat::zeros(size, CV_8UC1);

  // For all good matches, lets append to our returned vectors
  // NOTE: if we multi-thread this atomic can cause some randomness due to
  // multiple thread detecting features NOTE: this is due to the fact that we
  // select update features based on feat id NOTE: thus the order will matter
  // since we try to select oldest (smallest id) to update with NOTE: not sure
  // how to remove... maybe a better way?
  for (size_t i = 0; i < pts0_ext.size(); i++) {
    // Get current left keypoint, check that it is in bounds
    cv::KeyPoint kpt = pts0_ext.at(i);
    int x = (int)kpt.pt.x;
    int y = (int)kpt.pt.y;
    int x_grid = (int)(kpt.pt.x / (float)min_px_dist);
    int y_grid = (int)(kpt.pt.y / (float)min_px_dist);
    if (x_grid < 0 || x_grid >= size.width || y_grid < 0 ||
        y_grid >= size.height || x < 0 || x >= img0.cols || y < 0 ||
        y >= img0.rows) {
      continue;
    }
    // Check if this keypoint is near another point
    if (grid_2d.at<uint8_t>(y_grid, x_grid) > 127) continue;
    // Else we are good, append our keypoints and descriptors
    pts0.push_back(pts0_ext.at(i));
    desc0.push_back(desc0_ext.row((int)i));
    // Set our IDs to be unique IDs here, will later replace with corrected
    // ones, after temporal matching
    size_t temp = ++currid;
    ids0.push_back(temp);
    grid_2d.at<uint8_t>(y_grid, x_grid) = 255;
  }
}

void TrackDescriptor::perform_detection_stereo(
    const cv::Mat &img0, const cv::Mat &img1, const cv::Mat &mask0,
    const cv::Mat &mask1, std::vector<cv::KeyPoint> &pts0,
    std::vector<cv::KeyPoint> &pts1, cv::Mat &desc0, cv::Mat &desc1,
    size_t cam_id0, size_t cam_id1, std::vector<size_t> &ids0,
    std::vector<size_t> &ids1) {
  // Assert that we need features
  assert(pts0.empty());
  assert(pts1.empty());

  // Extract our features (use FAST with griding), and their descriptors
  std::vector<cv::KeyPoint> pts0_ext, pts1_ext;
  cv::Mat desc0_ext, desc1_ext;
  parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    bool is_left = (i == 0);
                    Grider_FAST::perform_griding(
                        is_left ? img0 : img1, is_left ? mask0 : mask1,
                        is_left ? pts0_ext : pts1_ext, num_features, grid_x,
                        grid_y, threshold, true);
                    (is_left ? orb0 : orb1)
                        ->compute(is_left ? img0 : img1,
                                  is_left ? pts0_ext : pts1_ext,
                                  is_left ? desc0_ext : desc1_ext);
                  }
                }));

  // Do matching from the left to the right image
  std::vector<cv::DMatch> matches;
  robust_match(pts0_ext, pts1_ext, desc0_ext, desc1_ext, cam_id0, cam_id1,
               matches);

  // Create a 2D occupancy grid for this current image
  // Note that we scale this down, so that each grid point is equal to a set of
  // pixels This means that we will reject points that less then grid_px_size
  // points away then existing features
  cv::Size size0((int)((float)img0.cols / (float)min_px_dist),
                 (int)((float)img0.rows / (float)min_px_dist));
  cv::Mat grid_2d_0 = cv::Mat::zeros(size0, CV_8UC1);
  cv::Size size1((int)((float)img1.cols / (float)min_px_dist),
                 (int)((float)img1.rows / (float)min_px_dist));
  cv::Mat grid_2d_1 = cv::Mat::zeros(size1, CV_8UC1);

  // For all good matches, lets append to our returned vectors
  for (size_t i = 0; i < matches.size(); i++) {
    // Get our ids
    int index_pt0 = matches.at(i).queryIdx;
    int index_pt1 = matches.at(i).trainIdx;

    // Get current left/right keypoint, check that it is in bounds
    cv::KeyPoint kpt0 = pts0_ext.at(index_pt0);
    cv::KeyPoint kpt1 = pts1_ext.at(index_pt1);
    int x0 = (int)kpt0.pt.x;
    int y0 = (int)kpt0.pt.y;
    int x0_grid = (int)(kpt0.pt.x / (float)min_px_dist);
    int y0_grid = (int)(kpt0.pt.y / (float)min_px_dist);
    if (x0_grid < 0 || x0_grid >= size0.width || y0_grid < 0 ||
        y0_grid >= size0.height || x0 < 0 || x0 >= img0.cols || y0 < 0 ||
        y0 >= img0.rows) {
      continue;
    }
    int x1 = (int)kpt1.pt.x;
    int y1 = (int)kpt1.pt.y;
    int x1_grid = (int)(kpt1.pt.x / (float)min_px_dist);
    int y1_grid = (int)(kpt1.pt.y / (float)min_px_dist);
    if (x1_grid < 0 || x1_grid >= size1.width || y1_grid < 0 ||
        y1_grid >= size1.height || x1 < 0 || x1 >= img0.cols || y1 < 0 ||
        y1 >= img0.rows) {
      continue;
    }

    // Check if this keypoint is near another point
    if (grid_2d_0.at<uint8_t>(y0_grid, x0_grid) > 127 ||
        grid_2d_1.at<uint8_t>(y1_grid, x1_grid) > 127)
      continue;

    // Append our keypoints and descriptors
    pts0.push_back(pts0_ext[index_pt0]);
    pts1.push_back(pts1_ext[index_pt1]);
    desc0.push_back(desc0_ext.row(index_pt0));
    desc1.push_back(desc1_ext.row(index_pt1));

    // Set our IDs to be unique IDs here, will later replace with corrected
    // ones, after temporal matching
    size_t temp = ++currid;
    ids0.push_back(temp);
    ids1.push_back(temp);
  }
}

void TrackDescriptor::robust_match(const std::vector<cv::KeyPoint> &pts0,
                                   const std::vector<cv::KeyPoint> &pts1,
                                   const cv::Mat &desc0, const cv::Mat &desc1,
                                   size_t id0, size_t id1,
                                   std::vector<cv::DMatch> &matches) {
  // Our 1to2 and 2to1 match vectors
  std::vector<std::vector<cv::DMatch>> matches0to1, matches1to0;

  // Match descriptors (return 2 nearest neighbours)
  matcher->knnMatch(desc0, desc1, matches0to1, 2);
  matcher->knnMatch(desc1, desc0, matches1to0, 2);

  // Do a ratio test for both matches
  robust_ratio_test(matches0to1);
  robust_ratio_test(matches1to0);

  // Finally do a symmetry test
  std::vector<cv::DMatch> matches_good;
  robust_symmetry_test(matches0to1, matches1to0, matches_good);

  // Convert points into points for RANSAC
  std::vector<cv::Point2f> pts0_rsc, pts1_rsc;
  for (size_t i = 0; i < matches_good.size(); i++) {
    // Get our ids
    int index_pt0 = matches_good.at(i).queryIdx;
    int index_pt1 = matches_good.at(i).trainIdx;
    // Push back just the 2d point
    pts0_rsc.push_back(pts0[index_pt0].pt);
    pts1_rsc.push_back(pts1[index_pt1].pt);
  }

  // If we don't have enough points for ransac just return empty
  if (pts0_rsc.size() < 10) return;

  // Normalize these points, so we can then do ransac
  // We don't want to do ransac on distorted image uvs since the mapping is
  // nonlinear
  std::vector<cv::Point2f> pts0_n, pts1_n;
  for (size_t i = 0; i < pts0_rsc.size(); i++) {
    pts0_n.push_back(camera_calib.at(id0)->undistort_cv(pts0_rsc.at(i)));
    pts1_n.push_back(camera_calib.at(id1)->undistort_cv(pts1_rsc.at(i)));
  }

  // Do RANSAC outlier rejection (note since we normalized the max pixel error
  // is now in the normalized cords)
  std::vector<uchar> mask_rsc;
  std::vector<uchar> mask_hmg;
  double max_focallength_img0 = std::max(camera_calib.at(id0)->get_K()(0, 0),
                                         camera_calib.at(id0)->get_K()(1, 1));
  double max_focallength_img1 = std::max(camera_calib.at(id1)->get_K()(0, 0),
                                         camera_calib.at(id1)->get_K()(1, 1));
  double max_focallength = std::max(max_focallength_img0, max_focallength_img1);
  const double ransac_thr = 1.0 / max_focallength;
  cv::Mat F = cv::findFundamentalMat(pts0_n, pts1_n, cv::FM_RANSAC, ransac_thr,
                                     0.999, mask_rsc);
  cv::Mat H =
      cv::findHomography(pts0_n, pts1_n, cv::RANSAC, ransac_thr, mask_hmg,
                         2000, 0.999);

  size_t f_inliers = 0;
  for (uchar m : mask_rsc) {
    f_inliers += (m == (uchar)1);
  }
  size_t h_inliers = 0;
  for (uchar m : mask_hmg) {
    h_inliers += (m == (uchar)1);
  }
  const size_t total = pts0_n.size();
  const double f_ratio =
      (total > 0) ? (100.0 * static_cast<double>(f_inliers) / total) : 0.0;
  const double h_ratio =
      (total > 0) ? (100.0 * static_cast<double>(h_inliers) / total) : 0.0;
  auto compute_f_residual = [](const cv::Mat &Fmat,
                               const std::vector<cv::Point2f> &p0,
                               const std::vector<cv::Point2f> &p1,
                               const std::vector<uchar> &mask) {
    if (Fmat.empty() || Fmat.rows != 3 || Fmat.cols != 3) {
      return std::numeric_limits<double>::infinity();
    }
    cv::Mat F64;
    Fmat.convertTo(F64, CV_64F);
    cv::Mat Ft = F64.t();
    double sum = 0.0;
    int cnt = 0;
    for (size_t i = 0; i < p0.size(); i++) {
      if (i >= mask.size() || mask[i] != (uchar)1) {
        continue;
      }
      const cv::Vec3d x0(p0[i].x, p0[i].y, 1.0);
      const cv::Vec3d x1(p1[i].x, p1[i].y, 1.0);
      const cv::Vec3d l1 = cv::Vec3d(
          Ft.at<double>(0, 0) * x1[0] + Ft.at<double>(0, 1) * x1[1] +
              Ft.at<double>(0, 2) * x1[2],
          Ft.at<double>(1, 0) * x1[0] + Ft.at<double>(1, 1) * x1[1] +
              Ft.at<double>(1, 2) * x1[2],
          Ft.at<double>(2, 0) * x1[0] + Ft.at<double>(2, 1) * x1[1] +
              Ft.at<double>(2, 2) * x1[2]);
      const cv::Vec3d l2 = cv::Vec3d(
          F64.at<double>(0, 0) * x0[0] + F64.at<double>(0, 1) * x0[1] +
              F64.at<double>(0, 2) * x0[2],
          F64.at<double>(1, 0) * x0[0] + F64.at<double>(1, 1) * x0[1] +
              F64.at<double>(1, 2) * x0[2],
          F64.at<double>(2, 0) * x0[0] + F64.at<double>(2, 1) * x0[1] +
              F64.at<double>(2, 2) * x0[2]);
      const double n1 = std::sqrt(l1[0] * l1[0] + l1[1] * l1[1]);
      const double n2 = std::sqrt(l2[0] * l2[0] + l2[1] * l2[1]);
      if (n1 < 1e-12 || n2 < 1e-12) {
        continue;
      }
      const double d1 = std::abs(l1[0] * x0[0] + l1[1] * x0[1] + l1[2]) / n1;
      const double d2 = std::abs(l2[0] * x1[0] + l2[1] * x1[1] + l2[2]) / n2;
      sum += 0.5 * (d1 + d2);
      cnt++;
    }
    return (cnt > 0) ? (sum / static_cast<double>(cnt))
                     : std::numeric_limits<double>::infinity();
  };
  auto compute_h_residual = [](const cv::Mat &Hmat,
                               const std::vector<cv::Point2f> &p0,
                               const std::vector<cv::Point2f> &p1,
                               const std::vector<uchar> &mask) {
    if (Hmat.empty() || Hmat.rows != 3 || Hmat.cols != 3) {
      return std::numeric_limits<double>::infinity();
    }
    cv::Mat H64;
    Hmat.convertTo(H64, CV_64F);
    cv::Mat Hinv;
    if (!cv::invert(H64, Hinv, cv::DECOMP_SVD)) {
      return std::numeric_limits<double>::infinity();
    }
    double sum = 0.0;
    int cnt = 0;
    for (size_t i = 0; i < p0.size(); i++) {
      if (i >= mask.size() || mask[i] != (uchar)1) {
        continue;
      }
      const cv::Vec3d x0(p0[i].x, p0[i].y, 1.0);
      const cv::Vec3d x1(p1[i].x, p1[i].y, 1.0);
      cv::Vec3d hx1(
          H64.at<double>(0, 0) * x0[0] + H64.at<double>(0, 1) * x0[1] +
              H64.at<double>(0, 2) * x0[2],
          H64.at<double>(1, 0) * x0[0] + H64.at<double>(1, 1) * x0[1] +
              H64.at<double>(1, 2) * x0[2],
          H64.at<double>(2, 0) * x0[0] + H64.at<double>(2, 1) * x0[1] +
              H64.at<double>(2, 2) * x0[2]);
      cv::Vec3d hix0(
          Hinv.at<double>(0, 0) * x1[0] + Hinv.at<double>(0, 1) * x1[1] +
              Hinv.at<double>(0, 2) * x1[2],
          Hinv.at<double>(1, 0) * x1[0] + Hinv.at<double>(1, 1) * x1[1] +
              Hinv.at<double>(1, 2) * x1[2],
          Hinv.at<double>(2, 0) * x1[0] + Hinv.at<double>(2, 1) * x1[1] +
              Hinv.at<double>(2, 2) * x1[2]);
      if (std::abs(hx1[2]) < 1e-12 || std::abs(hix0[2]) < 1e-12) {
        continue;
      }
      hx1 /= hx1[2];
      hix0 /= hix0[2];
      const double e1 = std::hypot(hx1[0] - x1[0], hx1[1] - x1[1]);
      const double e2 = std::hypot(hix0[0] - x0[0], hix0[1] - x0[1]);
      sum += 0.5 * (e1 + e2);
      cnt++;
    }
    return (cnt > 0) ? (sum / static_cast<double>(cnt))
                     : std::numeric_limits<double>::infinity();
  };
  auto compute_hull_area_and_shrunk =
      [](const std::vector<cv::Point2f> &pts, const std::vector<uchar> &mask,
         double shrink_px) {
    std::vector<cv::Point2f> inliers;
    inliers.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); i++) {
      if (i < mask.size() && mask[i] == (uchar)1) {
        inliers.push_back(pts[i]);
      }
    }
    if (inliers.size() < 3) {
      return std::make_pair(0.0, 0.0);
    }
    std::vector<cv::Point2f> hull;
    cv::convexHull(inliers, hull, false);
    if (hull.size() < 3) {
      return std::make_pair(0.0, 0.0);
    }
    const double area_raw = std::abs(cv::contourArea(hull));
    if (area_raw <= 1e-12 || shrink_px <= 0.0) {
      return std::make_pair(area_raw, area_raw);
    }

    // 凸多边形快速内缩：各边沿内法线平移 shrink_px，并取相邻平移线交点。
    const size_t n = hull.size();
    const double signed_area = cv::contourArea(hull, true);
    const bool ccw = (signed_area > 0.0);
    std::vector<cv::Vec3d> lines;
    lines.reserve(n);
    for (size_t i = 0; i < n; i++) {
      const cv::Point2f &p0 = hull[i];
      const cv::Point2f &p1 = hull[(i + 1) % n];
      const cv::Point2f e = p1 - p0;
      const double len = std::hypot(e.x, e.y);
      if (len < 1e-9) {
        return std::make_pair(area_raw, 0.0);
      }
      cv::Point2f n_in = ccw ? cv::Point2f(-e.y / len, e.x / len)
                             : cv::Point2f(e.y / len, -e.x / len);
      const double c = n_in.x * p0.x + n_in.y * p0.y + shrink_px;
      lines.emplace_back(n_in.x, n_in.y, c);
    }

    std::vector<cv::Point2f> hull_shrunk;
    hull_shrunk.reserve(n);
    for (size_t i = 0; i < n; i++) {
      const cv::Vec3d &l1 = lines[i];
      const cv::Vec3d &l2 = lines[(i + 1) % n];
      const double det = l1[0] * l2[1] - l1[1] * l2[0];
      if (std::abs(det) < 1e-12) {
        return std::make_pair(area_raw, 0.0);
      }
      const double x = (l1[2] * l2[1] - l2[2] * l1[1]) / det;
      const double y = (l1[0] * l2[2] - l2[0] * l1[2]) / det;
      hull_shrunk.emplace_back((float)x, (float)y);
    }
    if (hull_shrunk.size() < 3) {
      return std::make_pair(area_raw, 0.0);
    }
    const double area_shrunk = std::abs(cv::contourArea(hull_shrunk));
    return std::make_pair(area_raw, area_shrunk);
  };
  const double f_res = compute_f_residual(F, pts0_n, pts1_n, mask_rsc);
  const double h_res = compute_h_residual(H, pts0_n, pts1_n, mask_hmg);
  const double kShrinkPx = 12.0;
  const auto f_hull0 =
      compute_hull_area_and_shrunk(pts0_rsc, mask_rsc, kShrinkPx);
  const auto h_hull0 =
      compute_hull_area_and_shrunk(pts0_rsc, mask_hmg, kShrinkPx);
  const auto f_hull1 =
      compute_hull_area_and_shrunk(pts1_rsc, mask_rsc, kShrinkPx);
  const auto h_hull1 =
      compute_hull_area_and_shrunk(pts1_rsc, mask_hmg, kShrinkPx);
  const double f_hull_total = f_hull0.second + f_hull1.second;
  const double h_hull_total = h_hull0.second + h_hull1.second;
  const char *better_code =
      (f_res < h_res) ? "F" : ((h_res < f_res) ? "H" : "TIE");
  PRINT_INFO(
      CYAN
      "[DESC-GEOM] cam %zu->%zu\n"
      "  TOTAL | n=%-4zu | better=%-3s | shrink=%.0f px | env_shrink(H/F): total=%8.1f/%-8.1f\n"
      "  H     | inlier=%4zu(%6.2f%%) | res=%10.6f | env_shrink: total=%8.1f src=%8.1f dst=%8.1f\n"
      "  F     | inlier=%4zu(%6.2f%%) | res=%10.6f | env_shrink: total=%8.1f src=%8.1f dst=%8.1f\n"
      RESET,
      id0, id1, total, better_code, kShrinkPx, h_hull_total, f_hull_total,
      h_inliers, h_ratio, h_res, h_hull_total, h_hull0.second, h_hull1.second,
      f_inliers, f_ratio, f_res, f_hull_total, f_hull0.second, f_hull1.second);

  // Loop through all good matches, and only append ones that have passed RANSAC
  for (size_t i = 0; i < matches_good.size(); i++) {
    // Skip if bad ransac id
    if (mask_rsc[i] != 1) continue;
    // Else, lets append this match to the return array!
    matches.push_back(matches_good.at(i));
  }
}

void TrackDescriptor::robust_ratio_test(
    std::vector<std::vector<cv::DMatch>> &matches) {
  // Loop through all matches
  for (auto &match : matches) {
    // If 2 NN has been identified, else remove this feature
    if (match.size() > 1) {
      // check distance ratio, remove it if the ratio is larger
      if (match[0].distance / match[1].distance > knn_ratio) {
        match.clear();
      }
    } else {
      // does not have 2 neighbours, so remove it
      match.clear();
    }
  }
}

void TrackDescriptor::robust_symmetry_test(
    std::vector<std::vector<cv::DMatch>> &matches1,
    std::vector<std::vector<cv::DMatch>> &matches2,
    std::vector<cv::DMatch> &good_matches) {
  // for all matches image 1 -> image 2
  for (auto &match1 : matches1) {
    // ignore deleted matches
    if (match1.empty() || match1.size() < 2) continue;
    // for all matches image 2 -> image 1
    for (auto &match2 : matches2) {
      // ignore deleted matches
      if (match2.empty() || match2.size() < 2) continue;
      // Match symmetry test
      if (match1[0].queryIdx == match2[0].trainIdx &&
          match2[0].queryIdx == match1[0].trainIdx) {
        // add symmetrical match
        good_matches.emplace_back(cv::DMatch(
            match1[0].queryIdx, match1[0].trainIdx, match1[0].distance));
        // next match in image 1 -> image 2
        break;
      }
    }
  }
}
