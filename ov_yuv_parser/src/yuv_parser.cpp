#include "yuv_parser.h"
#include "imu_parser.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm>
#include <map>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/highgui/highgui.hpp>

YUVParser::YUVParser(int width, int height) 
    : width_(width), height_(height) {
    frame_size_ = width_ * height_;  // YUV400: only Y component, 1 byte per pixel
}

std::vector<FrameInfo> YUVParser::parseFrameInfo(const std::string& info_file) {
    std::vector<FrameInfo> frame_infos;
    std::ifstream file(info_file);
    
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << info_file << std::endl;
        return frame_infos;
    }
    
    std::string line;
    bool first_line = true;
    
    while (std::getline(file, line)) {
        // Skip header line
        if (first_line) {
            first_line = false;
            continue;
        }
        
        // Skip empty lines
        if (line.empty()) continue;
        
        // Remove BOM and other leading whitespace/special characters
        if (!line.empty() && static_cast<unsigned char>(line[0]) == 0xEF) {
            // UTF-8 BOM: EF BB BF
            if (line.size() >= 3 && 
                static_cast<unsigned char>(line[1]) == 0xBB && 
                static_cast<unsigned char>(line[2]) == 0xBF) {
                line = line.substr(3);
            }
        }
        // Remove null characters
        line.erase(std::remove(line.begin(), line.end(), '\0'), line.end());
        // Remove leading whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        FrameInfo info;
        
        iss >> info.filename 
            >> info.frame_count 
            >> info.vi_pts 
            >> info.exp_time 
            >> info.width 
            >> info.height;
        
        frame_infos.push_back(info);
    }
    
    file.close();
    return frame_infos;
}

bool YUVParser::readFrame(std::ifstream& file, std::vector<unsigned char>& frame_data) {
    frame_data.resize(frame_size_);
    file.read(reinterpret_cast<char*>(frame_data.data()), frame_size_);
    
    if (file.gcount() != frame_size_) {
        std::cerr << "Error: Failed to read complete frame. Read " 
                  << file.gcount() << " bytes, expected " << frame_size_ << std::endl;
        return false;
    }
    
    return true;
}

std::vector<std::vector<unsigned char>> YUVParser::parseYUVFile(
    const std::string& yuv_file, 
    int num_frames) {
    
    std::vector<std::vector<unsigned char>> frames;
    std::ifstream file(yuv_file, std::ios::binary);
    
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open YUV file " << yuv_file << std::endl;
        return frames;
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    size_t expected_size = frame_size_ * num_frames;
    if (file_size < expected_size) {
        int actual_frames = file_size / frame_size_;
        std::cerr << "Warning: YUV file \"" << yuv_file << "\" size (" << file_size 
                  << " bytes) is smaller than expected (" << expected_size 
                  << " bytes). Adjusting to read " << actual_frames 
                  << " frames instead of " << num_frames << std::endl;
        num_frames = actual_frames;
    }
    
    for (int i = 0; i < num_frames; ++i) {
        std::vector<unsigned char> frame_data;
        if (readFrame(file, frame_data)) {
            frames.push_back(frame_data);
        } else {
            std::cerr << "Error reading frame " << i << std::endl;
            break;
        }
    }
    
    file.close();
    return frames;
}

std::vector<FrameData> YUVParser::parseAllFrames(const std::string& dump_yuv_dir) {
    std::vector<FrameData> all_frames;
    std::string info_file = dump_yuv_dir + "/FrameInfo.txt";
    
    // Parse frame info
    std::vector<FrameInfo> frame_infos = parseFrameInfo(info_file);
    
    // Group by filename
    std::map<std::string, std::vector<FrameInfo>> file_groups;
    for (const auto& info : frame_infos) {
        file_groups[info.filename].push_back(info);
    }
    
    // Process each file
    for (const auto& file_group : file_groups) {
        const std::string& filename = file_group.first;
        const std::vector<FrameInfo>& infos = file_group.second;
        
        std::string yuv_path = dump_yuv_dir + "/" + filename;
        std::vector<std::vector<unsigned char>> frames = parseYUVFile(yuv_path, 100);
        
        // Create FrameData for each frame
        for (size_t i = 0; i < frames.size() && i < infos.size(); ++i) {
            FrameData frame_data;
            frame_data.info = infos[i];
            frame_data.frame_data = frames[i];
            frame_data.timestamp = infos[i].vi_pts;
            all_frames.push_back(frame_data);
        }
    }
    
    // Sort by timestamp
    std::sort(all_frames.begin(), all_frames.end());
    
    return all_frames;
}

cv::Mat YUVParser::frameToMat(const std::vector<unsigned char>& frame_data) {
    if (frame_data.size() != static_cast<size_t>(frame_size_)) {
        std::cerr << "Error: Frame data size mismatch" << std::endl;
        return cv::Mat();
    }
    
    // YUV400 is grayscale, directly create Mat from data
    cv::Mat frame(height_, width_, CV_8UC1);
    std::memcpy(frame.data, frame_data.data(), frame_size_);
    
    return frame;
}

bool YUVParser::saveFrameAsImage(const std::vector<unsigned char>& frame_data, 
                                 const std::string& output_path) {
    cv::Mat frame = frameToMat(frame_data);
    if (frame.empty()) {
        return false;
    }
    
    return cv::imwrite(output_path, frame);
}

void YUVParser::printFrameStats(const std::vector<unsigned char>& frame_data) {
    if (frame_data.empty()) {
        std::cerr << "Error: Empty frame data" << std::endl;
        return;
    }
    
    unsigned char min_val = 255, max_val = 0;
    long long sum = 0;
    
    for (unsigned char pixel : frame_data) {
        if (pixel < min_val) min_val = pixel;
        if (pixel > max_val) max_val = pixel;
        sum += pixel;
    }
    
    double mean = static_cast<double>(sum) / frame_data.size();
    
    std::cout << "Frame Statistics:" << std::endl;
    std::cout << "  Min: " << static_cast<int>(min_val) << std::endl;
    std::cout << "  Max: " << static_cast<int>(max_val) << std::endl;
    std::cout << "  Mean: " << std::fixed << std::setprecision(2) << mean << std::endl;
}

int YUVParser::exportFramesToPNG(const std::string& dump_yuv_dir) {
    // Check if dump_yuv directory exists
    struct stat info;
    if (stat(dump_yuv_dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        std::cerr << "Error: Directory does not exist: " << dump_yuv_dir << std::endl;
        return -1;
    }
    
    // Create output directory (parallel to dump_yuv, replace "dump_yuv" with "png")
    std::string output_dir = dump_yuv_dir;
    size_t pos = output_dir.find("dump_yuv");
    if (pos != std::string::npos) {
        output_dir.replace(pos, 8, "png");
    } else {
        // If "dump_yuv" not found, append "/png" to parent directory
        size_t last_slash = output_dir.find_last_of("/\\");
        if (last_slash != std::string::npos) {
            output_dir = output_dir.substr(0, last_slash + 1) + "png";
        } else {
            output_dir = "./png";
        }
    }
    
    // Create output directory (create parent directories if needed)
    // Simple recursive directory creation
    std::string path = output_dir;
    std::string current_path;
    
    // Handle absolute paths
    if (path[0] == '/') {
        current_path = "/";
    }
    
    // Split path and create directories
    size_t start = (path[0] == '/') ? 1 : 0;
    for (size_t i = start; i <= path.length(); ++i) {
        if (i == path.length() || path[i] == '/' || path[i] == '\\') {
            if (i > start) {
                if (!current_path.empty() && current_path.back() != '/') {
                    current_path += "/";
                }
                current_path += path.substr(start, i - start);
                
                struct stat dir_info;
                if (stat(current_path.c_str(), &dir_info) != 0) {
                    if (mkdir(current_path.c_str(), 0755) != 0) {
                        std::cerr << "Error: Failed to create directory: " << current_path << std::endl;
                        return -1;
                    }
                }
            }
            start = i + 1;
        }
    }
    
    std::cout << "Output directory: " << output_dir << std::endl;
    
    // Parse all frames
    std::cout << "Parsing frames from: " << dump_yuv_dir << std::endl;
    std::vector<FrameData> all_frames = parseAllFrames(dump_yuv_dir);
    
    if (all_frames.empty()) {
        std::cerr << "Error: No frames found in " << dump_yuv_dir << std::endl;
        return -1;
    }
    
    std::cout << "Found " << all_frames.size() << " frames. Exporting to PNG..." << std::endl;
    
    // Export each frame as PNG
    int success_count = 0;
    for (size_t i = 0; i < all_frames.size(); ++i) {
        const FrameData& frame_data = all_frames[i];
        
        // Create filename using timestamp
        std::string filename = std::to_string(frame_data.timestamp) + ".png";
        std::string output_path_str = output_dir + "/" + filename;
        
        // Save frame as PNG
        if (saveFrameAsImage(frame_data.frame_data, output_path_str)) {
            success_count++;
            if ((i + 1) % 100 == 0) {
                std::cout << "Exported " << (i + 1) << " / " << all_frames.size() << " frames..." << std::endl;
            }
        } else {
            std::cerr << "Warning: Failed to save frame " << i 
                      << " (timestamp: " << frame_data.timestamp << ")" << std::endl;
        }
    }
    
    std::cout << "Export complete! Successfully exported " << success_count 
              << " / " << all_frames.size() << " frames to " << output_dir << std::endl;
    
    return success_count;
}

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

int YUVParser::playFramesStepByStep(const std::string& dump_yuv_dir, const std::string& imu_file) {
    // 检查目录是否存在
    struct stat info;
    if (stat(dump_yuv_dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        std::cerr << "Error: Directory does not exist: " << dump_yuv_dir << std::endl;
        return -1;
    }
    
    std::cout << "=== 逐帧播放模式 ===" << std::endl;
    std::cout << "图像目录: " << dump_yuv_dir << std::endl;
    std::cout << "IMU文件: " << imu_file << std::endl;
    
    // 解析图像数据
    std::cout << "正在解析图像数据..." << std::endl;
    std::vector<FrameData> frame_data = parseAllFrames(dump_yuv_dir);
    std::cout << "找到 " << frame_data.size() << " 帧图像" << std::endl;
    
    // 解析IMU数据
    std::cout << "正在解析IMU数据..." << std::endl;
    IMUParser imu_parser;
    std::vector<IMUData> imu_data = imu_parser.parseIMUFile(imu_file);
    std::cout << "找到 " << imu_data.size() << " 条IMU数据" << std::endl;
    
    if (frame_data.empty() && imu_data.empty()) {
        std::cerr << "Error: 没有找到任何数据!" << std::endl;
        return -1;
    }
    
    // 合并数据并按时间戳排序
    std::cout << "正在合并数据并按时间戳排序..." << std::endl;
    std::vector<TimestampedData> merged_data;
    
    // 添加图像数据
    for (const auto& frame : frame_data) {
        TimestampedData data;
        data.type = TimestampedData::FRAME;
        data.timestamp = frame.timestamp;
        data.frame_data = frame;
        merged_data.push_back(data);
    }
    
    // 添加IMU数据
    for (const auto& imu : imu_data) {
        TimestampedData data;
        data.type = TimestampedData::IMU;
        data.timestamp = imu.timeStamp;
        data.imu_data = imu;
        merged_data.push_back(data);
    }
    
    // 按时间戳排序
    std::sort(merged_data.begin(), merged_data.end());
    
    std::cout << "总共 " << merged_data.size() << " 条数据" << std::endl;
    if (!merged_data.empty()) {
        std::cout << "时间范围: " << merged_data[0].timestamp 
                  << " 到 " << merged_data.back().timestamp << std::endl;
    }
    std::cout << "\n按任意键继续播放下一帧，按 'q' 键退出\n" << std::endl;
    
    // 统计信息
    int frame_count = 0;
    int imu_count = 0;
    
    // 逐帧播放
    for (size_t i = 0; i < merged_data.size(); ++i) {
        const TimestampedData& data = merged_data[i];
        
        if (data.type == TimestampedData::FRAME) {
            frame_count++;
            
            // 转换为OpenCV Mat并显示
            cv::Mat frame_mat = frameToMat(data.frame_data.frame_data);
            if (frame_mat.empty()) {
                std::cerr << "Warning: 无法转换第 " << frame_count << " 帧图像" << std::endl;
                continue;
            }
            
            // 在图像上添加信息文本
            cv::Mat display_frame = frame_mat.clone();
            std::stringstream ss;
            ss << "Frame #" << frame_count 
               << " | Timestamp: " << data.timestamp
               << " | Total: " << i + 1 << "/" << merged_data.size();
            
            // 先画黑色轮廓，再画白色文本，使文本更清晰
            cv::putText(display_frame, ss.str(), cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 3);
            cv::putText(display_frame, ss.str(), cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            
            // 显示图像
            cv::imshow("YUV Frame Player", display_frame);
            
            // 等待按键
            std::cout << "[" << i + 1 << "/" << merged_data.size() << "] "
                      << "图像帧 #" << frame_count 
                      << " | 时间戳: " << data.timestamp
                      << " | 按任意键继续 (q退出)..." << std::flush;
            
            int key = cv::waitKey(0) & 0xFF;
            std::cout << std::endl;
            
            // 如果按了 'q' 键，退出
            if (key == 'q' || key == 'Q' || key == 27) {  // 27 is ESC
                std::cout << "用户退出播放" << std::endl;
                break;
            }
            
        } else if (data.type == TimestampedData::IMU) {
            imu_count++;
            
            // 打印IMU信息（不暂停，只打印信息）
            std::cout << "[" << i + 1 << "/" << merged_data.size() << "] "
                      << "IMU数据 #" << imu_count 
                      << " | 时间戳: " << data.timestamp
                      << " | acc: [" << std::fixed << std::setprecision(3)
                      << data.imu_data.accX << ", " 
                      << data.imu_data.accY << ", "
                      << data.imu_data.accZ << "]"
                      << " | gyro: [" 
                      << data.imu_data.gyroX << ", "
                      << data.imu_data.gyroY << ", "
                      << data.imu_data.gyroZ << "]"
                      << std::endl;
        }
    }
    
    // 关闭窗口
    cv::destroyAllWindows();
    
    std::cout << "\n播放完成!" << std::endl;
    std::cout << "总共播放: " << frame_count << " 帧图像, " << imu_count << " 条IMU数据" << std::endl;
    
    return 0;
}
