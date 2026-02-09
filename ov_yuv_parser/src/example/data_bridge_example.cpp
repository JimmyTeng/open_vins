/*
 * Data Bridge 使用示例
 *
 * 演示如何使用 data_bridge.h 将 vio_interface 数据类型转换为 ov_core 类型。
 * 数据路径: data/yuv_data/init/outdoor/grass/static
 *
 * 用法: data_bridge_example [数据目录]
 *  默认: data/yuv_data/init/outdoor/grass/static
 */

#include "yuv_parser.h"
#include "imu_parser.h"
#include "vio_interface.h"
#include "data_bridge.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

namespace ov_yuv_parser {

// 微秒转纳秒
vio_timestamp_t us_to_ns(long long ts_us) {
  if (ts_us < 1e12) {
    return static_cast<vio_timestamp_t>(ts_us) * 1000;
  }
  return static_cast<vio_timestamp_t>(ts_us);
}

}  // namespace ov_yuv_parser

int main(int argc, char* argv[]) {
  using namespace ov_yuv_parser;

  // 默认使用 init 下的室外草地静止场景
  std::string data_dir = "data/yuv_data/init/outdoor/grass/static";

  if (argc > 1) {
    data_dir = argv[1];
  }

  std::string dump_yuv_dir = data_dir;
  if (dump_yuv_dir.size() < 8 ||
      dump_yuv_dir.substr(dump_yuv_dir.size() - 8) != "dump_yuv") {
    if (!dump_yuv_dir.empty() && dump_yuv_dir.back() != '/' &&
        dump_yuv_dir.back() != '\\') {
      dump_yuv_dir += "/";
    }
    dump_yuv_dir += "dump_yuv";
  }

  std::string imu_file = data_dir;
  if (!imu_file.empty() && imu_file.back() != '/' && imu_file.back() != '\\') {
    imu_file += "/";
  }
  imu_file += "imu.txt";

  std::cout << "=== Data Bridge 示例 ===" << std::endl;
  std::cout << "数据目录: " << data_dir << std::endl;
  std::cout << "Dump YUV: " << dump_yuv_dir << std::endl;
  std::cout << "IMU 文件: " << imu_file << std::endl;
  std::cout << std::endl;

  YUVParser yuv_parser(640, 480);
  IMUParser imu_parser;

  // 解析帧数据
  std::vector<FrameData> frame_data = yuv_parser.parseAllFrames(dump_yuv_dir);
  std::cout << "解析到 " << frame_data.size() << " 帧" << std::endl;

  // 解析 IMU
  std::vector<IMUData> imu_data = imu_parser.parseIMUFile(imu_file);
  std::cout << "解析到 " << imu_data.size() << " 条 IMU" << std::endl;
  std::cout << std::endl;

  if (frame_data.empty() && imu_data.empty()) {
    std::cerr << "错误: 未找到数据，请检查路径是否正确" << std::endl;
    std::cerr << "示例: " << argv[0]
              << " data/yuv_data/init/outdoor/grass/static" << std::endl;
    return 1;
  }

  // ========================================================================
  // 示例 1: 单个 IMU 转换 ToImuData
  // ========================================================================
  if (!imu_data.empty()) {
    vio_imu_msg_t vio_imu;
    vio_imu.timestamp = us_to_ns(imu_data[0].timeStamp);
    vio_imu.acc.data[0] = imu_data[0].accX;
    vio_imu.acc.data[1] = imu_data[0].accY;
    vio_imu.acc.data[2] = imu_data[0].accZ;
    vio_imu.gyro.data[0] = imu_data[0].gyroX;
    vio_imu.gyro.data[1] = imu_data[0].gyroY;
    vio_imu.gyro.data[2] = imu_data[0].gyroZ;

    ov_core::ImuData ov_imu = ToImuData(vio_imu);

    std::cout << "--- 示例 1: ToImuData ---" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  timestamp (s): " << ov_imu.timestamp << std::endl;
    std::cout << "  gyro (rad/s): ["
              << ov_imu.wm(0) << ", " << ov_imu.wm(1) << ", " << ov_imu.wm(2)
              << "]" << std::endl;
    std::cout << "  acc (m/s^2):  ["
              << ov_imu.am(0) << ", " << ov_imu.am(1) << ", " << ov_imu.am(2)
              << "]" << std::endl;
    std::cout << std::endl;
  }

  // ========================================================================
  // 示例 2: 批量 IMU 转换 ToImuDataBatch
  // ========================================================================
  if (imu_data.size() >= 5) {
    std::vector<vio_imu_msg_t> vio_imus(5);
    for (size_t i = 0; i < 5; ++i) {
      vio_imus[i].timestamp = us_to_ns(imu_data[i].timeStamp);
      vio_imus[i].acc.data[0] = imu_data[i].accX;
      vio_imus[i].acc.data[1] = imu_data[i].accY;
      vio_imus[i].acc.data[2] = imu_data[i].accZ;
      vio_imus[i].gyro.data[0] = imu_data[i].gyroX;
      vio_imus[i].gyro.data[1] = imu_data[i].gyroY;
      vio_imus[i].gyro.data[2] = imu_data[i].gyroZ;
    }

    auto ov_imus = ToImuDataBatch(vio_imus.data(), vio_imus.size());

    std::cout << "--- 示例 2: ToImuDataBatch ---" << std::endl;
    std::cout << "  转换了 " << ov_imus.size() << " 条 IMU" << std::endl;
    std::cout << "  时间范围: " << ov_imus.front().timestamp << " ~ "
              << ov_imus.back().timestamp << " (s)" << std::endl;
    std::cout << std::endl;
  }

  // ========================================================================
  // 示例 3: 图像转 cv::Mat 与 ToCameraData
  // ========================================================================
  if (!frame_data.empty()) {
    const FrameData& frame = frame_data[0];
    cv::Mat gray = yuv_parser.frameToMat(frame.frame_data);

    if (!gray.empty()) {
      vio_image_msg_t vio_img;
      vio_img.timestamp = us_to_ns(frame.timestamp);
      vio_img.camera_id = 0;
      vio_img.width = gray.cols;
      vio_img.height = gray.rows;
      vio_img.stride = gray.step[0];
      vio_img.format = VIO_PIXEL_FMT_GRAY8;
      vio_img.buffer = gray.data;

      // 3a: ToCvMat
      cv::Mat mat_clone = ToCvMat(vio_img, true);
      std::cout << "--- 示例 3a: ToCvMat ---" << std::endl;
      std::cout << "  cv::Mat size: " << mat_clone.cols << "x" << mat_clone.rows
                << ", type: CV_8UC" << mat_clone.channels() << std::endl;
      std::cout << std::endl;

      // 3b: ToCameraData (OpenVINS 可直接使用)
      ov_core::CameraData cam_data = ToCameraData(vio_img, true);

      std::cout << "--- 示例 3b: ToCameraData ---" << std::endl;
      std::cout << std::fixed << std::setprecision(6);
      std::cout << "  timestamp (s): " << cam_data.timestamp << std::endl;
      std::cout << "  sensor_ids: [";
      for (size_t i = 0; i < cam_data.sensor_ids.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << cam_data.sensor_ids[i];
      }
      std::cout << "]" << std::endl;
      std::cout << "  images.size(): " << cam_data.images.size() << std::endl;
      if (!cam_data.images.empty()) {
        std::cout << "  图像尺寸: " << cam_data.images[0].cols << "x"
                  << cam_data.images[0].rows << std::endl;
      }
      std::cout << std::endl;
    }
  }

  std::cout << "=== 示例完成 ===" << std::endl;
  return 0;
}
