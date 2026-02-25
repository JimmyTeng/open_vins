/*
 * VIO YUV Runner - 使用 vio_interface API 处理 YUV 数据
 * 读取 YUV 图像和 IMU 数据，按时间戳顺序推送到 VIO 系统
 */

#include "yuv_parser.h"
#include "imu_parser.h"
#include "vio_interface.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cstring>

// 统一的数据结构，用于按时间戳排序
struct TimestampedData {
    enum DataType {
        FRAME,
        IMU
    };
    
    DataType type;
    long long timestamp;
    
    // Frame data (if type == FRAME)
    FrameData frame_data;
    
    // IMU data (if type == IMU)
    IMUData imu_data;
    
    // 用于时间戳比较
    bool operator<(const TimestampedData& other) const {
        return timestamp < other.timestamp;
    }
};

// 回调函数用户数据结构
struct CallbackUserData {
    bool show_image;
    std::chrono::high_resolution_clock::time_point frame_start_time;  // 当前帧开始处理的时间
    bool is_initializing;  // 是否正在初始化
};

// 状态回调函数
void state_callback(const vio_state_t* state, void* user_data) {
    if (!state) return;
    
    // 获取用户数据
    CallbackUserData* cb_data = static_cast<CallbackUserData*>(user_data);
    bool show_image = cb_data ? cb_data->show_image : true;  // 默认为 true
    
    // 计算处理时间
    double processing_time_ms = 0.0;
    if (cb_data) {
        auto frame_end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            frame_end_time - cb_data->frame_start_time);
        processing_time_ms = duration.count() / 1000.0;  // 转换为毫秒
    }
    
    // 状态字符串
    const char* status_str[] = {
        "NOT_READY",
        "INITIALIZING",
        "TRACKING",
        "LOST"
    };
    
    // 更新初始化状态
    if (cb_data) {
        cb_data->is_initializing = (state->status == VIO_STATUS_INITIALIZING);
    }
    
    // 输出状态信息（单行格式）
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== VIO State === ";
    std::cout << "Timestamp: " << state->timestamp << " ns, ";
    std::cout << "Status: " << status_str[state->status] << ", ";
    std::cout << "Processing Time: " << std::setprecision(3) << processing_time_ms << " ms";
    
    if (state->status == VIO_STATUS_TRACKING) {
        std::cout << ", Position: [" 
                  << std::setprecision(3) << state->position.data[0] << ", "
                  << state->position.data[1] << ", "
                  << state->position.data[2] << "], ";
        
        std::cout << "Orientation: [" 
                  << state->orientation.data[0] << ", "
                  << state->orientation.data[1] << ", "
                  << state->orientation.data[2] << ", "
                  << state->orientation.data[3] << "], ";
        
        std::cout << "Velocity: [" 
                  << state->velocity.data[0] << ", "
                  << state->velocity.data[1] << ", "
                  << state->velocity.data[2] << "], ";
        
        std::cout << "Gyro Bias: [" 
                  << std::setprecision(4) << state->gyro_bias.data[0] << ", "
                  << state->gyro_bias.data[1] << ", "
                  << state->gyro_bias.data[2] << "], ";
        
        std::cout << "Acc Bias: [" 
                  << state->acc_bias.data[0] << ", "
                  << state->acc_bias.data[1] << ", "
                  << state->acc_bias.data[2] << "]";
    }
    std::cout << std::endl;
}

// 转换时间戳（假设输入是微秒，转换为纳秒）
vio_timestamp_t convert_timestamp(long long ts_us) {
    if (ts_us < 1e12) {
        return static_cast<vio_timestamp_t>(ts_us) * 1000;
    }
    return static_cast<vio_timestamp_t>(ts_us);
}

int main(int argc, char* argv[]) {
    // 默认路径
    std::cout << cv::getBuildInformation() << std::endl;
    std::cout << "Build Time: " <<__DATE__ <<" " << __TIME__ << std::endl;
    std::string input_dir = "./data/move";
    std::string config_file = "";
    bool show_image = true;
    
    std::vector<std::string> positional_args;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--no-display" || arg == "--no-imshow" || arg == "-n") {
            show_image = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "VIO YUV Runner - 使用 vio_interface API 处理 YUV 数据" << std::endl;
            std::cout << "用法: " << argv[0] << " [选项] [数据目录] [配置文件路径]" << std::endl;
            std::cout << "  --no-display, -n    不显示图像窗口" << std::endl;
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
    
    std::cout << "=== VIO YUV Runner ===" << std::endl;
    std::cout << "Input directory: " << input_dir << std::endl;
    std::cout << "Dump YUV directory: " << dump_yuv_dir << std::endl;
    std::cout << "IMU file: " << imu_file << std::endl;
    if (!config_file.empty()) {
        std::cout << "Config file: " << config_file << std::endl;
    }
    std::cout << std::endl;
    
    YUVParser yuv_parser(640, 480);
    IMUParser imu_parser;
    
    std::vector<FrameData> frame_data = yuv_parser.parseAllFrames(dump_yuv_dir);
    std::vector<IMUData> imu_data = imu_parser.parseIMUFile(imu_file);
    std::cout << "Found " << frame_data.size() << " frames, " << imu_data.size() << " IMU entries" << std::endl;
    
    if (frame_data.empty() && imu_data.empty()) {
        std::cerr << "Error: No data found" << std::endl;
        return 1;
    }
    
    std::vector<TimestampedData> merged_data;
    for (const auto& frame : frame_data) {
        TimestampedData data;
        data.type = TimestampedData::FRAME;
        data.timestamp = frame.timestamp;
        data.frame_data = frame;
        merged_data.push_back(data);
    }
    for (const auto& imu : imu_data) {
        TimestampedData data;
        data.type = TimestampedData::IMU;
        data.timestamp = imu.timeStamp;
        data.imu_data = imu;
        merged_data.push_back(data);
    }
    std::sort(merged_data.begin(), merged_data.end(),
              [](const TimestampedData& a, const TimestampedData& b) {
                  return a.timestamp < b.timestamp;
              });
    
    const char* yaml_path = config_file.empty() ? nullptr : config_file.c_str();
    vio_handle_t vio_handle = vio_create(yaml_path);
    if (!vio_handle) {
        std::cerr << "Error: Failed to create VIO instance" << std::endl;
        return 1;
    }
    
    int init_result = vio_init(vio_handle);
    if (init_result != 0) {
        std::cerr << "Error: Failed to initialize VIO (code: " << init_result << ")" << std::endl;
        vio_destroy(vio_handle);
        return 1;
    }
    
    CallbackUserData callback_data;
    callback_data.show_image = show_image;
    callback_data.is_initializing = false;
    callback_data.frame_start_time = std::chrono::high_resolution_clock::now();
    vio_register_state_callback(vio_handle, state_callback, &callback_data);
    
    std::cout << "Processing data..." << std::endl;
    int frame_count = 0;
    int imu_count = 0;
    
    for (const auto& data : merged_data) {
        if (data.type == TimestampedData::FRAME) {
            const FrameData& frame = data.frame_data;
            cv::Mat gray_image = yuv_parser.frameToMat(frame.frame_data);
            if (gray_image.empty()) continue;
            
            cv::Mat gray_image_cont = gray_image.clone();
            vio_image_msg_t img_msg;
            img_msg.timestamp = convert_timestamp(frame.timestamp);
            img_msg.camera_id = 0;
            img_msg.width = gray_image_cont.cols;
            img_msg.height = gray_image_cont.rows;
            img_msg.stride = gray_image_cont.step[0];
            img_msg.format = VIO_PIXEL_FMT_GRAY8;
            img_msg.buffer = gray_image_cont.data;
            
            callback_data.frame_start_time = std::chrono::high_resolution_clock::now();
            vio_push_image(vio_handle, &img_msg);
            frame_count++;
        } else if (data.type == TimestampedData::IMU) {
            const IMUData& imu = data.imu_data;
            vio_imu_msg_t imu_msg;
            imu_msg.timestamp = convert_timestamp(imu.timeStamp);
            imu_msg.acc.data[0] = imu.accX;
            imu_msg.acc.data[1] = imu.accY;
            imu_msg.acc.data[2] = imu.accZ;
            imu_msg.gyro.data[0] = imu.gyroX;
            imu_msg.gyro.data[1] = imu.gyroY;
            imu_msg.gyro.data[2] = imu.gyroZ;
            vio_push_imu(vio_handle, &imu_msg);
            imu_count++;
        }
    }
    
    std::cout << "Complete: " << frame_count << " frames, " << imu_count << " IMU" << std::endl;
    vio_destroy(vio_handle);
    return 0;
}
