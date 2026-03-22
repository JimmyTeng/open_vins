/*
 * VIO PNG Runner — 两种数据源：
 * 1) VioDataRecorder 数据集目录：dataset_meta.txt + imu.csv + camera_index.csv + mosaics/
 *    （与 ov_core VioDataPlayer 一致，异步预加载 mosaic PNG）
 * 2) YUV 导出 PNG：dump_yuv/FrameInfo.txt + 并列 png/<timestamp>.png + ../imu.txt
 *    （PngStreamDataLoader reader 线程）
 *
 * 再次落盘：在 estimator_config.yaml 中配置 record_vio_dataset 等；由 VioInterface 加载。
 */

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "ext/ss_vio debug.h"
#include "ext/ss_vio.h"
#include "ext/ss_vio_err.h"
#include "imu_parser.h"
#include "png_stream_data_loader.h"
#include "utils/vio_data_record_play.h"
#include "yuv_parser.h"

namespace fs = std::filesystem;

static double timestamp_to_sec(long long ts) {
  if (ts >= 1e12) {
    return static_cast<double>(ts) * 1e-9;
  }
  return static_cast<double>(ts) * 1e-6;
}

static void print_build_info() {
  char buf[512];
  SS_VIO_FormatBuildInfo(buf, sizeof(buf));
  std::cout << buf << "\n";
}

static bool is_vio_dataset_directory(const std::string& dir) {
  const fs::path root(dir);
  return fs::exists(root / "dataset_meta.txt") &&
         (fs::exists(root / "imu.csv") || fs::exists(root / "camera_index.csv"));
}

static std::string ensure_dump_yuv(const std::string& input_dir) {
  std::string d = input_dir;
  if (d.size() < 8 || d.substr(d.size() - 8) != "dump_yuv") {
    if (!d.empty() && d.back() != '/' && d.back() != '\\') {
      d += "/";
    }
    d += "dump_yuv";
  }
  return d;
}

static std::string dump_yuv_to_png_dir(const std::string& dump_yuv_dir) {
  std::string p = dump_yuv_dir;
  size_t pos = p.find("dump_yuv");
  if (pos != std::string::npos) {
    p.replace(pos, 8, "png");
    return p;
  }
  if (!p.empty() && p.back() != '/' && p.back() != '\\') {
    p += "/";
  }
  return p + "png";
}

/// SS_VIO 使用 GRAY8；VioDataPlayer 可能从 PNG 读出多通道
static cv::Mat to_gray8_continuous(const cv::Mat& in) {
  if (in.empty()) return {};
  cv::Mat g;
  if (in.channels() == 1)
    g = in;
  else
    cv::cvtColor(in, g, cv::COLOR_BGR2GRAY);
  if (!g.isContinuous()) g = g.clone();
  return g;
}

static int run_vio_dataset_mode(const std::string& dataset_dir,
                                const std::string& config_file) {
  std::cout << "\n========== Run Config (VioDataRecorder dataset) ==========\n";
  std::cout << "  Dataset dir:   " << dataset_dir << "\n";
  std::cout << "  Config file:   "
            << (config_file.empty() ? "(default)" : config_file) << "\n";
  std::cout << "  (dataset_meta.txt + imu.csv + camera_index.csv + mosaics/)\n";
  std::cout << "============================================================\n\n";

  ov_core::VioDataPlayer player;
  if (!player.open(dataset_dir)) {
    std::cerr << "Error: VioDataPlayer::open failed (check paths and CSV)\n";
    return 1;
  }

  std::cout << "  grid " << player.grid_cols() << " x " << player.grid_rows()
            << ", tile_w " << player.tile_width()
            << ", tile_h(meta) " << player.tile_height() << "\n";

  OT_VIO_Param param = {};
  param.logFlag = false;
  param.enableLoopClosure = false;
  if (!config_file.empty()) {
    size_t len = std::min(config_file.size(),
                          static_cast<size_t>(OT_VIO_MAX_PATH_LEN - 1));
    std::memcpy(param.calibParamPath, config_file.c_str(), len);
    param.calibParamPath[len] = '\0';
  }
  if (!param.calibParamPath[0]) {
    std::cerr << "Error: Config file path is required for SS_VIO_Init\n";
    return 1;
  }

  int init_ret = SS_VIO_Init(&param);
  if (init_ret != 0) {
    std::cerr << "Error: SS_VIO_Init failed (code: " << init_ret << ")\n";
    return 1;
  }

  int frame_count = 0;
  int imu_count = 0;

  ov_core::VioDataPlayer::PlaybackEvent ev;
  while (player.next_event(ev)) {
    if (ev.type == ov_core::VioDataPlayer::PlaybackEvent::Type::Imu) {
      const ov_core::ImuData& imu = ev.imu;
      OT_VIO_ImuDataInfo imu_info = {};
      imu_info.num = 1;
      imu_info.imuDatas[0].timestamp = static_cast<double>(imu.timestamp);
      imu_info.imuDatas[0].accX = static_cast<float>(imu.am(0));
      imu_info.imuDatas[0].accY = static_cast<float>(imu.am(1));
      imu_info.imuDatas[0].accZ = static_cast<float>(imu.am(2));
      imu_info.imuDatas[0].gyroX = static_cast<float>(imu.wm(0));
      imu_info.imuDatas[0].gyroY = static_cast<float>(imu.wm(1));
      imu_info.imuDatas[0].gyroZ = static_cast<float>(imu.wm(2));
      if (SS_VIO_PushImuData(&imu_info) == 0)
        imu_count++;
    } else {
      const ov_core::CameraData& cam = ev.camera;
      for (size_t k = 0; k < cam.images.size(); ++k) {
        cv::Mat gray = to_gray8_continuous(cam.images[k]);
        if (gray.empty()) continue;
        cv::Mat gray_cont = gray.clone();
        OT_VIO_CameraData cam_data = {};
        cam_data.leftImage.timestamp = cam.timestamp;
        cam_data.leftImage.exposureDuration = 0.0;
        cam_data.leftImage.virtAddr = gray_cont.data;
        cam_data.leftImage.width = static_cast<unsigned int>(gray_cont.cols);
        cam_data.leftImage.height = static_cast<unsigned int>(gray_cont.rows);
        cam_data.leftImage.stride =
            static_cast<unsigned int>(gray_cont.step[0]);
        if (SS_VIO_PushImageData(&cam_data) == 0)
          frame_count++;
      }
    }
  }

  std::cout << "Complete: " << frame_count << " image push(es), " << imu_count
            << " IMU\n";
  SS_VIO_DeInit();
  return 0;
}

static int run_exported_png_mode(const std::string& input_dir,
                                 const std::string& config_file,
                                 size_t max_frame_cache, bool show_image) {
  std::string dump_yuv_dir = ensure_dump_yuv(input_dir);
  std::string png_dir = dump_yuv_to_png_dir(dump_yuv_dir);
  std::string imu_file = dump_yuv_dir + "/../imu.txt";

  std::cout << "\n========== Run Config (exported timestamp PNG) ==========\n";
  std::cout << "  Input dir:     " << input_dir << "\n";
  std::cout << "  Dump YUV dir:  " << dump_yuv_dir << " (FrameInfo.txt)\n";
  std::cout << "  PNG dir:       " << png_dir << "\n";
  std::cout << "  IMU file:      " << imu_file << "\n";
  std::cout << "  Config file:   "
            << (config_file.empty() ? "(default)" : config_file) << "\n";
  std::cout << "===========================================================\n\n";

  YUVParser yuv_parser(640, 480);
  PngStreamDataLoader loader(png_dir, dump_yuv_dir, imu_file, max_frame_cache);
  std::cout << "Found " << loader.scheduleSize()
            << " scheduled items (PNG reader, cache max " << max_frame_cache
            << " frames)" << std::endl;

  if (loader.empty()) {
    std::cerr << "Error: No data found" << std::endl;
    return 1;
  }

  loader.start();

  OT_VIO_Param param = {};
  param.logFlag = false;
  param.enableLoopClosure = false;
  if (!config_file.empty()) {
    size_t len = std::min(config_file.size(),
                          static_cast<size_t>(OT_VIO_MAX_PATH_LEN - 1));
    std::memcpy(param.calibParamPath, config_file.c_str(), len);
    param.calibParamPath[len] = '\0';
  }
  if (!param.calibParamPath[0]) {
    std::cerr << "Error: Config file path is required for SS_VIO_Init"
              << std::endl;
    loader.stop();
    return 1;
  }

  int init_ret = SS_VIO_Init(&param);
  if (init_ret != 0) {
    std::cerr << "Error: SS_VIO_Init failed (code: " << init_ret << ")"
              << std::endl;
    loader.stop();
    return 1;
  }

  std::cout << "Processing PNG data..." << std::endl;
  int frame_count = 0;
  int imu_count = 0;

  TimestampedData data;
  while (loader.getNext(data)) {
    if (data.type == TimestampedData::FRAME) {
      const FrameData& frame = data.frame_data;
      cv::Mat gray_image = yuv_parser.frameToMat(frame.frame_data);
      if (gray_image.empty()) continue;

      cv::Mat gray_image_cont = gray_image.clone();
      OT_VIO_CameraData cam_data = {};
      cam_data.leftImage.timestamp = timestamp_to_sec(frame.timestamp);
      cam_data.leftImage.exposureDuration = 0.0;
      cam_data.leftImage.virtAddr = gray_image_cont.data;
      cam_data.leftImage.width =
          static_cast<unsigned int>(gray_image_cont.cols);
      cam_data.leftImage.height =
          static_cast<unsigned int>(gray_image_cont.rows);
      cam_data.leftImage.stride =
          static_cast<unsigned int>(gray_image_cont.step[0]);

      int push_ret = SS_VIO_PushImageData(&cam_data);
      if (push_ret != 0) {
        std::cerr << "SS_VIO_PushImageData failed: " << push_ret << std::endl;
      } else {
        frame_count++;
      }
    } else if (data.type == TimestampedData::IMU) {
      const IMUData& imu = data.imu_data;
      OT_VIO_ImuDataInfo imu_info = {};
      imu_info.num = 1;
      imu_info.imuDatas[0].timestamp = timestamp_to_sec(imu.timeStamp);
      imu_info.imuDatas[0].accX = imu.accX;
      imu_info.imuDatas[0].accY = imu.accY;
      imu_info.imuDatas[0].accZ = imu.accZ;
      imu_info.imuDatas[0].gyroX = imu.gyroX;
      imu_info.imuDatas[0].gyroY = imu.gyroY;
      imu_info.imuDatas[0].gyroZ = imu.gyroZ;

      int push_ret = SS_VIO_PushImuData(&imu_info);
      if (push_ret != 0) {
        std::cerr << "SS_VIO_PushImuData failed: " << push_ret << std::endl;
      } else {
        imu_count++;
      }
    }
  }

  loader.stop();
  (void)show_image;
  std::cout << "Complete: " << frame_count << " frames, " << imu_count << " IMU"
            << std::endl;
  SS_VIO_DeInit();
  return 0;
}

int main(int argc, char* argv[]) {
  print_build_info();
  std::string input_dir = "./data/move";
  std::string config_file = "";
  bool show_image = true;
  size_t max_frame_cache = 4;

  std::vector<std::string> positional_args;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--no-display" || arg == "--no-imshow" || arg == "-n") {
      show_image = false;
    } else if (arg == "--cache-size" && i + 1 < argc) {
      max_frame_cache = static_cast<size_t>(std::max(1, std::atoi(argv[++i])));
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "VIO PNG Runner — ss_vio 回放 PNG / 数据集\n"
                << "用法: " << argv[0] << " [选项] [数据目录] [配置文件路径]\n\n"
                << "数据目录自动识别：\n"
                << "  • 若含 dataset_meta.txt 与 imu.csv 或 camera_index.csv：按 "
                   "VioDataRecorder 格式回放（mosaics/mosaic_*.png，见 "
                   "ov_core::VioDataPlayer）\n"
                << "  • 否则：按 yuv_to_png 导出目录（dump_yuv 旁 png/<时间戳>.png）\n\n"
                << "  --no-display, -n    保留选项（与 vio_yuv_runner 一致）\n"
                << "  --cache-size N      仅「导出 PNG」模式：reader 预读缓存帧数\n\n"
                << "再次落盘请在 YAML 中配置 record_vio_dataset、record_vio_dataset_root 等。\n";
      return 0;
    } else {
      positional_args.push_back(arg);
    }
  }

  if (positional_args.size() > 0) {
    input_dir = positional_args[0];
  }
  if (positional_args.size() > 1) {
    config_file = positional_args[1];
  } else if (config_file.empty()) {
    std::vector<std::string> possible_paths = {
        "config/openvins/estimator_config.yaml",
        "../config/openvins/estimator_config.yaml",
    };
    for (const auto& path : possible_paths) {
      std::ifstream test_file(path);
      if (test_file.good()) {
        config_file = path;
        break;
      }
    }
  }

  if (is_vio_dataset_directory(input_dir))
    return run_vio_dataset_mode(input_dir, config_file);
  return run_exported_png_mode(input_dir, config_file, max_frame_cache,
                               show_image);
}
