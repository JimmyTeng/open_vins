/*
 * VIO YUV Runner - 使用 ov_core/src/ext (ss_vio) 接口处理 YUV 数据
 * 流式读取：单独线程预读帧数据到有界缓存，控制最大内存用量
 */

#include "yuv_parser.h"
#include "imu_parser.h"
#include "stream_data_loader.h"
#include "ext/ss_vio.h"
#include "ext/ss_vio_err.h"
#include "build_info.h"  // 编译时生成：构建时间、Git 版本/分支/用户等
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cstring>
#include <cstdlib>

// 时间戳转秒（输入微秒或纳秒，统一输出秒）
static double timestamp_to_sec(long long ts) {
    if (ts >= 1e12) {
        return static_cast<double>(ts) * 1e-9;
    }
    return static_cast<double>(ts) * 1e-6;
}

// 格式化输出编译时生成的构建信息
static void print_build_info() {
    std::cout << "\n========== Build Info ==========\n";
    std::cout << "  Build Time:   " << BUILD_TIMESTAMP << " " << BUILD_TIMEZONE << "\n";
    std::cout << "  Git Commit:   " << BUILD_GIT_COMMIT << " (" << BUILD_GIT_HASH << ")\n";
    std::cout << "  Git Branch:   " << BUILD_GIT_BRANCH << "\n";
    std::cout << "  Git Tag:      " << BUILD_GIT_TAG << "\n";
    std::cout << "  Git Status:   " << BUILD_GIT_DIRTY << "\n";
    std::cout << "  Git User:     " << BUILD_GIT_USER << " <" << BUILD_GIT_EMAIL << ">\n";
    std::cout << "=================================\n\n";
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
            std::cout << "VIO YUV Runner - 使用 ss_vio (ext) 接口处理 YUV 数据（流式加载）" << std::endl;
            std::cout << "用法: " << argv[0] << " [选项] [数据目录] [配置文件路径]" << std::endl;
            std::cout << "  --no-display, -n    不显示图像窗口" << std::endl;
            std::cout << "  --cache-size N      帧缓存大小，控制最大内存（默认 4）" << std::endl;
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

    std::string dump_yuv_dir = input_dir;
    if (dump_yuv_dir.size() < 8 || dump_yuv_dir.substr(dump_yuv_dir.size() - 8) != "dump_yuv") {
        if (!dump_yuv_dir.empty() && dump_yuv_dir.back() != '/' && dump_yuv_dir.back() != '\\') {
            dump_yuv_dir += "/";
        }
        dump_yuv_dir += "dump_yuv";
    }

    std::string imu_file = dump_yuv_dir + "/../imu.txt";

    std::cout << "=== VIO YUV Runner (ss_vio ext) ===" << std::endl;
    std::cout << "Input directory: " << input_dir << std::endl;
    std::cout << "Dump YUV directory: " << dump_yuv_dir << std::endl;
    std::cout << "IMU file: " << imu_file << std::endl;
    if (!config_file.empty()) {
        std::cout << "Config file: " << config_file << std::endl;
    }
    std::cout << std::endl;

    YUVParser yuv_parser(640, 480);
    StreamDataLoader loader(&yuv_parser, dump_yuv_dir, imu_file, max_frame_cache);
    std::cout << "Found " << loader.scheduleSize() << " scheduled items (streaming mode, cache max "
              << max_frame_cache << " frames)" << std::endl;

    if (loader.empty()) {
        std::cerr << "Error: No data found" << std::endl;
        return 1;
    }

    loader.start();

    OT_VIO_Param param = {};
    param.logFlag = false;
    param.enableLoopClosure = false;
    if (!config_file.empty()) {
        size_t len = std::min(config_file.size(), static_cast<size_t>(OT_VIO_MAX_PATH_LEN - 1));
        std::memcpy(param.calibParamPath, config_file.c_str(), len);
        param.calibParamPath[len] = '\0';
    }
    if (!param.calibParamPath[0]) {
        std::cerr << "Error: Config file path is required for SS_VIO_Init" << std::endl;
        loader.stop();
        return 1;
    }

    int init_ret = SS_VIO_Init(&param);
    if (init_ret != 0) {
        std::cerr << "Error: SS_VIO_Init failed (code: " << init_ret << ")" << std::endl;
        loader.stop();
        return 1;
    }

    std::cout << "Processing data..." << std::endl;
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
            cam_data.leftImage.width = static_cast<unsigned int>(gray_image_cont.cols);
            cam_data.leftImage.height = static_cast<unsigned int>(gray_image_cont.rows);
            cam_data.leftImage.stride = static_cast<unsigned int>(gray_image_cont.step[0]);

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
    std::cout << "Complete: " << frame_count << " frames, " << imu_count << " IMU" << std::endl;
    SS_VIO_DeInit();
    return 0;
}
