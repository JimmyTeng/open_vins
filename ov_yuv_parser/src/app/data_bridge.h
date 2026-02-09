//
// Data Bridge: vio_interface.h 数据类型 <-> ov_core sensor_data.h 互转
//

#ifndef OV_YUV_PARSER_DATA_BRIDGE_H
#define OV_YUV_PARSER_DATA_BRIDGE_H

#include "vio_interface.h"
#include "utils/sensor_data.h"
#include <opencv2/opencv.hpp>

namespace ov_yuv_parser {

namespace {

// 纳秒转秒
constexpr double NS_TO_SEC = 1e-9;

} // namespace

/**
 * @brief 将 vio_imu_msg_t 转换为 ov_core::ImuData
 */
inline ov_core::ImuData ToImuData(const vio_imu_msg_t& imu) {
  ov_core::ImuData out;
  out.timestamp = static_cast<double>(imu.timestamp) * NS_TO_SEC;
  out.wm = Eigen::Vector3d(imu.gyro.data[0], imu.gyro.data[1], imu.gyro.data[2]);
  out.am = Eigen::Vector3d(imu.acc.data[0], imu.acc.data[1], imu.acc.data[2]);
  return out;
}

/**
 * @brief 将 vio_image_msg_t 的 buffer 转为 cv::Mat
 * @param img 图像消息
 * @param clone_data 若为 true 则克隆数据（安全）；若为 false 则包装外部 buffer（调用者需保证 buffer 生命周期）
 */
inline cv::Mat ToCvMat(const vio_image_msg_t& img, bool clone_data = true) {
  if (img.buffer == nullptr || img.width <= 0 || img.height <= 0) {
    return cv::Mat();
  }

  cv::Mat wrap;
  switch (img.format) {
    case VIO_PIXEL_FMT_GRAY8:
      wrap = cv::Mat(img.height, img.width, CV_8UC1,
                    const_cast<uint8_t*>(img.buffer),
                    img.stride > 0 ? static_cast<size_t>(img.stride) : cv::Mat::AUTO_STEP);
      break;
    case VIO_PIXEL_FMT_RGB888:
      wrap = cv::Mat(img.height, img.width, CV_8UC3,
                    const_cast<uint8_t*>(img.buffer),
                    img.stride > 0 ? static_cast<size_t>(img.stride) : cv::Mat::AUTO_STEP);
      break;
    case VIO_PIXEL_FMT_RGBA8888:
      wrap = cv::Mat(img.height, img.width, CV_8UC4,
                    const_cast<uint8_t*>(img.buffer),
                    img.stride > 0 ? static_cast<size_t>(img.stride) : cv::Mat::AUTO_STEP);
      break;
    default:
      return cv::Mat();
  }

  return clone_data ? wrap.clone() : wrap;
}

/**
 * @brief 将 vio_image_msg_t 转换为 ov_core::CameraData
 * OpenVINS 通常使用灰度图进行跟踪，RGB888/RGBA 会转为灰度
 *
 * @param img 图像消息
 * @param to_grayscale 若为 true 且格式为 RGB/RGBA，则转换为灰度图（推荐用于 OpenVINS）
 */
inline ov_core::CameraData ToCameraData(const vio_image_msg_t& img,
                                        bool to_grayscale = true) {
  ov_core::CameraData out;
  out.timestamp = static_cast<double>(img.timestamp) * NS_TO_SEC;
  out.sensor_ids.push_back(img.camera_id);

  cv::Mat mat = ToCvMat(img, true);
  if (mat.empty()) {
    return out;
  }

  if (to_grayscale && mat.channels() > 1) {
    cv::Mat gray;
    if (mat.channels() == 3) {
      cv::cvtColor(mat, gray, cv::COLOR_RGB2GRAY);
    } else {
      cv::cvtColor(mat, gray, cv::COLOR_RGBA2GRAY);
    }
    out.images.push_back(gray);
  } else {
    out.images.push_back(mat);
  }

  out.masks.push_back(cv::Mat());  // 空 mask，OpenVINS 可选
  return out;
}

/**
 * @brief 批量 IMU 消息转换（用于队列场景）
 */
inline std::vector<ov_core::ImuData> ToImuDataBatch(
    const vio_imu_msg_t* imus, size_t count) {
  std::vector<ov_core::ImuData> out;
  out.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    out.push_back(ToImuData(imus[i]));
  }
  return out;
}

} // namespace ov_yuv_parser

#endif // OV_YUV_PARSER_DATA_BRIDGE_H
