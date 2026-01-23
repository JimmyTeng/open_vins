/*
 * VIO YUV Runner - 使用 vio_interface API 处理 YUV 数据
 * 读取 YUV 图像和 IMU 数据，按时间戳顺序推送到 VIO 系统
 */

#include "yuv_parser.h"
#include "imu_parser.h"
#include "api/vio_interface.h"
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
    
    // 显示跟踪图像（如果启用）
    if (show_image && state->tracked_image.available && 
        state->tracked_image.buffer != nullptr && 
        state->tracked_image.width > 0 && 
        state->tracked_image.height > 0 &&
        state->tracked_image.buffer_size > 0) {
        try {
            // 从缓冲区创建 OpenCV Mat（注意：数据是 BGR 格式）
            cv::Mat tracked_img(state->tracked_image.height, 
                               state->tracked_image.width, 
                               CV_8UC3, 
                               const_cast<uint8_t*>(state->tracked_image.buffer));
            
            // 验证 Mat 是否有效
            if (!tracked_img.empty() && tracked_img.data != nullptr) {
                // 使用 cv::imshow 显示图像
                cv::imshow("VIO Tracked Image", tracked_img);
                cv::waitKey(1);  // 非阻塞，允许 GUI 更新
            }
        } catch (const cv::Exception& e) {
            std::cerr << "Error displaying tracked image: " << e.what() << std::endl;
        }
    }
}

// 创建默认 VIO 配置
vio_config_t create_default_config(const char* config_file_path = nullptr) {
    vio_config_t config;
    
    // 如果提供了配置文件路径，使用它
    if (config_file_path && strlen(config_file_path) > 0) {
        config.config_file_path = config_file_path;
        // 其他参数会被忽略，因为会从配置文件加载
        return config;
    }
    
    // 否则使用默认值
    config.config_file_path = nullptr;
    
    // IMU 噪声参数（默认值，应该从配置文件读取）
    config.acc_noise_density = 1.6e-2;      // m/s^2/sqrt(Hz)
    config.acc_random_walk = 4.0e-4;         // m/s^3/sqrt(Hz)
    config.gyr_noise_density = 1.6e-4;      // rad/s/sqrt(Hz)
    config.gyr_random_walk = 1.9e-5;        // rad/s^2/sqrt(Hz)
    
    // 外参 T_bc (Body -> Camera)
    // 默认单位矩阵（需要根据实际标定设置）
    config.t_bc.data[0] = 0.0;
    config.t_bc.data[1] = 0.0;
    config.t_bc.data[2] = 0.0;
    
    config.r_bc.data[0] = 0.0;  // x
    config.r_bc.data[1] = 0.0;  // y
    config.r_bc.data[2] = 0.0;  // z
    config.r_bc.data[3] = 1.0;  // w
    
    // 初始化配置
    config.enable_online_calibration = false;
    config.init_window_size = 20;  // 初始化窗口大小（帧数）
    
    // 日志等级
    config.log_level = 1;  // 1 = Info
    
    return config;
}

// 转换时间戳（假设输入是微秒，转换为纳秒）
vio_timestamp_t convert_timestamp(long long ts_us) {
    // 如果时间戳看起来是微秒（通常很大），转换为纳秒
    // 如果已经是纳秒，直接返回
    if (ts_us < 1e12) {  // 小于 1e12 可能是微秒
        return static_cast<vio_timestamp_t>(ts_us) * 1000;
    }
    return static_cast<vio_timestamp_t>(ts_us);
}

int main(int argc, char* argv[]) {
    // 默认路径
    std::string input_dir = "data/yuv_data/2";
    std::string config_file = "";  // 默认不使用配置文件
    bool show_image = true;  // 默认显示图像
    
    // 解析命令行参数
    // 用法: vio_yuv_runner [数据目录] [配置文件路径(可选)] [--no-display]
    // 或者: vio_yuv_runner [--no-display] [数据目录] [配置文件路径(可选)]
    std::vector<std::string> positional_args;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--no-display" || arg == "--no-imshow" || arg == "-n") {
            show_image = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "VIO YUV Runner - 使用 vio_interface API 处理 YUV 数据" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "用法: " << argv[0] << " [选项] [数据目录] [配置文件路径]" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "选项:" << std::endl;
            std::cout << "  --no-display, --no-imshow, -n    不显示图像窗口" << std::endl;
            std::cout << "  --help, -h                        显示帮助信息" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "参数:" << std::endl;
            std::cout << "  数据目录          YUV 数据目录路径（默认: data/yuv_data/2）" << std::endl;
            std::cout << "  配置文件路径      OpenVINS 配置文件路径（可选）" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "示例:" << std::endl;
            std::cout << "  " << argv[0] << " data/yuv_data/2" << std::endl;
            std::cout << "  " << argv[0] << " --no-display data/yuv_data/2 config.yaml" << std::endl;
            return 0;
        } else {
            positional_args.push_back(arg);
        }
    }
    
    // 处理位置参数
    if (positional_args.size() > 0) {
        input_dir = positional_args[0];
    }
    if (positional_args.size() > 1) {
        config_file = positional_args[1];
    } else if (positional_args.size() == 0 || config_file.empty()) {
        // 如果没有提供配置文件，尝试使用默认路径
        // 尝试多个可能的路径
        std::vector<std::string> possible_paths = {
            "src/yuv_parser/config/openvins/estimator_config.yaml",
            "../src/yuv_parser/config/openvins/estimator_config.yaml",
            "../../src/yuv_parser/config/openvins/estimator_config.yaml",
            "/home/jimmy/project/msckf/src/yuv_parser/config/openvins/estimator_config.yaml"
        };
        
        // 检查哪个路径存在
        bool found = false;
        for (const auto& path : possible_paths) {
            std::ifstream test_file(path);
            if (test_file.good()) {
                config_file = path;
                found = true;
                break;
            }
        }
        
        if (!found) {
            std::cerr << "Warning: Could not find default config file, will use hardcoded defaults" << std::endl;
            config_file = "";
        }
    }
    
    // 确定 dump_yuv 目录
    std::string dump_yuv_dir = input_dir;
    if (dump_yuv_dir.size() < 8 || dump_yuv_dir.substr(dump_yuv_dir.size() - 8) != "dump_yuv") {
        if (!dump_yuv_dir.empty() && dump_yuv_dir.back() != '/' && dump_yuv_dir.back() != '\\') {
            dump_yuv_dir += "/";
        }
        dump_yuv_dir += "dump_yuv";
    }
    
    // 文件路径
    std::string info_file = dump_yuv_dir + "/FrameInfo.txt";
    std::string imu_file = dump_yuv_dir + "/../imu.txt";
    
    std::cout << "=== VIO YUV Runner ===" << std::endl;
    std::cout << "Input directory: " << input_dir << std::endl;
    std::cout << "Dump YUV directory: " << dump_yuv_dir << std::endl;
    std::cout << "FrameInfo file: " << info_file << std::endl;
    std::cout << "IMU file: " << imu_file << std::endl;
    if (!config_file.empty()) {
        std::cout << "Config file: " << config_file << std::endl;
    }
    std::cout << std::endl;
    
    // 创建解析器
    YUVParser yuv_parser(640, 480);
    IMUParser imu_parser;
    
    // 解析帧数据
    std::cout << "Parsing FrameInfo.txt and YUV files..." << std::endl;
    std::vector<FrameData> frame_data = yuv_parser.parseAllFrames(dump_yuv_dir);
    std::cout << "Found " << frame_data.size() << " frames" << std::endl;
    
    // 解析 IMU 数据
    std::cout << "Parsing imu.txt..." << std::endl;
    std::vector<IMUData> imu_data = imu_parser.parseIMUFile(imu_file);
    std::cout << "Found " << imu_data.size() << " IMU entries" << std::endl;
    
    if (frame_data.empty() && imu_data.empty()) {
        std::cerr << "Error: No data found" << std::endl;
        return 1;
    }
    
    // 合并并按时间戳排序
    std::cout << "Merging data by timestamp..." << std::endl;
    std::vector<TimestampedData> merged_data;
    
    // 添加帧数据
    for (const auto& frame : frame_data) {
        TimestampedData data;
        data.type = TimestampedData::FRAME;
        data.timestamp = frame.timestamp;
        data.frame_data = frame;
        merged_data.push_back(data);
    }
    
    // 添加 IMU 数据
    for (const auto& imu : imu_data) {
        TimestampedData data;
        data.type = TimestampedData::IMU;
        data.timestamp = imu.timeStamp;
        data.imu_data = imu;
        merged_data.push_back(data);
    }
    
    // 按时间戳排序
    std::sort(merged_data.begin(), merged_data.end(),
              [](const TimestampedData& a, const TimestampedData& b) {
                  return a.timestamp < b.timestamp;
              });
    
    std::cout << "Total merged entries: " << merged_data.size() << std::endl;
    if (!merged_data.empty()) {
        std::cout << "Time range: " << merged_data[0].timestamp 
                  << " to " << merged_data.back().timestamp << std::endl;
    }
    std::cout << std::endl;
    
    // 创建 VIO 实例
    std::cout << "Initializing VIO system..." << std::endl;
    vio_handle_t vio_handle = vio_create();
    if (!vio_handle) {
        std::cerr << "Error: Failed to create VIO instance" << std::endl;
        return 1;
    }
    
    // 创建配置（如果提供了配置文件路径，会从配置文件加载）
    vio_config_t config = create_default_config(config_file.empty() ? nullptr : config_file.c_str());
    
    // 初始化 VIO
    int init_result = vio_init(vio_handle, &config);
    if (init_result != 0) {
        std::cerr << "Error: Failed to initialize VIO system (code: " << init_result << ")" << std::endl;
        vio_destroy(vio_handle);
        return 1;
    }
    
    // 创建回调用户数据
    CallbackUserData callback_data;
    callback_data.show_image = show_image;
    callback_data.is_initializing = false;
    callback_data.frame_start_time = std::chrono::high_resolution_clock::now();
    
    // 注册状态回调
    vio_register_state_callback(vio_handle, state_callback, &callback_data);
    
    std::cout << "VIO system initialized successfully" << std::endl;
    if (!show_image) {
        std::cout << "Image display disabled (--no-display)" << std::endl;
    }
    std::cout << "Processing data..." << std::endl;
    std::cout << std::endl;
    
    // 处理数据
    int frame_count = 0;
    int imu_count = 0;
    int processed_count = 0;
    
    std::cout << "Starting to process " << merged_data.size() << " data entries..." << std::endl;
    
    for (const auto& data : merged_data) {
        if (processed_count % 1000 == 0 && processed_count > 0) {
            std::cout << "[Progress] Processed " << processed_count << "/" << merged_data.size() 
                      << " entries (Frames: " << frame_count << ", IMU: " << imu_count << ")" << std::endl;
        }
        
        if (data.type == TimestampedData::FRAME) {
            // 处理图像帧
            const FrameData& frame = data.frame_data;
            
            // 转换为灰度图像（YUV400 已经是灰度）
            cv::Mat gray_image = yuv_parser.frameToMat(frame.frame_data);
            if (gray_image.empty()) {
                std::cerr << "Warning: Failed to convert frame to Mat" << std::endl;
                continue;
            }
            
            // 确保图像数据是连续的（clone 确保数据在内存中连续）
            cv::Mat gray_image_cont = gray_image.clone();
            
            // 准备图像消息
            vio_image_msg_t img_msg;
            img_msg.timestamp = convert_timestamp(frame.timestamp);
            img_msg.camera_id = 0;
            img_msg.width = gray_image_cont.cols;
            img_msg.height = gray_image_cont.rows;
            img_msg.stride = gray_image_cont.step[0];
            img_msg.format = VIO_PIXEL_FMT_GRAY8;
            // 注意：这里需要确保 gray_image_cont 在 vio_push_image 调用期间有效
            // 由于我们立即调用，所以是安全的
            img_msg.buffer = gray_image_cont.data;
            
            // 记录帧处理开始时间
            callback_data.frame_start_time = std::chrono::high_resolution_clock::now();
            
            // 推送到 VIO
            // #region agent log
            {
                std::ofstream log_file("/home/jimmy/project/msckf/.cursor/debug.log", std::ios::app);
                log_file << "{\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\",\"location\":\"vio_yuv_runner.cpp:319\",\"message\":\"Before vio_push_image call\",\"data\":{\"frame_count\":" << frame_count << ",\"timestamp\":" << img_msg.timestamp << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
                log_file.close();
            }
            // #endregion
            
            if (frame_count % 10 == 0) {
                std::cout << "[Frame " << frame_count << "] Pushing image, timestamp: " 
                          << img_msg.timestamp << " ns, size: " << img_msg.width 
                          << "x" << img_msg.height << std::endl;
            }
            vio_push_image(vio_handle, &img_msg);
            
            // #region agent log
            {
                std::ofstream log_file("/home/jimmy/project/msckf/.cursor/debug.log", std::ios::app);
                log_file << "{\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\",\"location\":\"vio_yuv_runner.cpp:332\",\"message\":\"After vio_push_image call\",\"data\":{\"frame_count\":" << frame_count << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
                log_file.close();
            }
            // #endregion
            
            if (frame_count % 10 == 0) {
                std::cout << "[Frame " << frame_count << "] Image pushed successfully" << std::endl;
                std::cout.flush();
            }
            
            frame_count++;
            if (frame_count % 10 == 0) {
                std::cout << "[Status] Processed " << frame_count << " frames, " 
                          << imu_count << " IMU samples" << std::endl;
                std::cout.flush();
            }
        } else if (data.type == TimestampedData::IMU) {
            // 处理 IMU 数据
            const IMUData& imu = data.imu_data;
            
            // 准备 IMU 消息
            vio_imu_msg_t imu_msg;
            imu_msg.timestamp = convert_timestamp(imu.timeStamp);
            imu_msg.acc.data[0] = imu.accX;
            imu_msg.acc.data[1] = imu.accY;
            imu_msg.acc.data[2] = imu.accZ;
            imu_msg.gyro.data[0] = imu.gyroX;
            imu_msg.gyro.data[1] = imu.gyroY;
            imu_msg.gyro.data[2] = imu.gyroZ;
            
            // 推送到 VIO
            if (imu_count % 1000 == 0 && imu_count > 0) {
                std::cout << "[IMU " << imu_count << "] Pushing IMU, timestamp: " 
                          << imu_msg.timestamp << " ns" << std::endl;
                std::cout.flush();
            }
            vio_push_imu(vio_handle, &imu_msg);
            if (imu_count % 1000 == 0 && imu_count > 0) {
                std::cout << "[IMU " << imu_count << "] IMU pushed successfully" << std::endl;
                std::cout.flush();
            }
            
            imu_count++;
        }
        
        processed_count++;
    }
    
    std::cout << std::endl;
    std::cout << "=== Processing Complete ===" << std::endl;
    std::cout << "Total processed: " << processed_count << " entries" << std::endl;
    std::cout << "Frames: " << frame_count << std::endl;
    std::cout << "IMU samples: " << imu_count << std::endl;
    
    // 清理
    vio_destroy(vio_handle);
    
    return 0;
}

