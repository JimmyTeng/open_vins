#include "yuv_parser.h"
#include "imu_parser.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <queue>

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
        return timestamp > other.timestamp;  // 用于优先队列（最小堆）
    }
};

int main(int argc, char* argv[]) {
    std::cout<< "Build Time: " << __DATE__ << " " << __TIME__ << std::endl;
    // Default paths
    std::string input_dir = "data/250704/8";
    
    // Parse command line arguments
    if (argc > 1) {
        input_dir = argv[1];
    }
    
    // Determine dump_yuv directory
    std::string dump_yuv_dir = input_dir;
    if (dump_yuv_dir.size() < 8 || dump_yuv_dir.substr(dump_yuv_dir.size() - 8) != "dump_yuv") {
        if (!dump_yuv_dir.empty() && dump_yuv_dir.back() != '/' && dump_yuv_dir.back() != '\\') {
            dump_yuv_dir += "/";
        }
        dump_yuv_dir += "dump_yuv";
    }
    
    // Paths for files
    std::string info_file = dump_yuv_dir + "/FrameInfo.txt";
    std::string imu_file = dump_yuv_dir + "/imu.txt";
    
    std::cout << "=== YUV400 Parser with IMU (Time-synchronized) ===" << std::endl;
    std::cout << "Input directory: " << input_dir << std::endl;
    std::cout << "Dump YUV directory: " << dump_yuv_dir << std::endl;
    std::cout << "FrameInfo file: " << info_file << std::endl;
    std::cout << "IMU file: " << imu_file << std::endl;
    std::cout << std::endl;
    
    // Create parsers
    YUVParser yuv_parser(640, 480);
    IMUParser imu_parser;
    
    // Parse frame data
    std::cout << "Parsing FrameInfo.txt and YUV files..." << std::endl;
    std::vector<FrameData> frame_data = yuv_parser.parseAllFrames(dump_yuv_dir);
    std::cout << "Found " << frame_data.size() << " frames" << std::endl;
    
    // Parse IMU data
    std::cout << "Parsing imu.txt..." << std::endl;
    std::vector<IMUData> imu_data = imu_parser.parseIMUFile(imu_file);
    std::cout << "Found " << imu_data.size() << " IMU entries" << std::endl;
    imu_parser.printIMUStats(imu_data);
    std::cout << std::endl;
    
    if (frame_data.empty() && imu_data.empty()) {
        std::cerr << "Error: No data found" << std::endl;
        return 1;
    }
    
    // Merge and sort by timestamp
    std::cout << "Merging data by timestamp..." << std::endl;
    std::vector<TimestampedData> merged_data;
    
    // Add frame data
    for (const auto& frame : frame_data) {
        TimestampedData data;
        data.type = TimestampedData::FRAME;
        data.timestamp = frame.timestamp;
        data.frame_data = frame;
        merged_data.push_back(data);
    }
    
    // Add IMU data
    for (const auto& imu : imu_data) {
        TimestampedData data;
        data.type = TimestampedData::IMU;
        data.timestamp = imu.timeStamp;
        data.imu_data = imu;
        merged_data.push_back(data);
    }
    
    // Sort by timestamp
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
    
    // Display/print data in timestamp order
    std::cout << "=== Displaying data in timestamp order ===" << std::endl;
    std::cout << "Controls: 'p' to pause/resume, 'q' or ESC to quit" << std::endl;
    std::cout << std::endl;
    
    bool auto_play = true;
    bool paused = false;
    int frame_delay = 30;  // 30ms per frame
    int frame_count = 0;
    int imu_count = 0;
    
    for (size_t i = 0; i < merged_data.size(); ++i) {
        const TimestampedData& data = merged_data[i];
        
        if (data.type == TimestampedData::FRAME) {
            frame_count++;
            std::cout << "[" << std::setw(10) << data.timestamp << "] FRAME #" << frame_count 
                      << " from " << data.frame_data.info.filename 
                      << " (frame " << data.frame_data.info.frame_count << ")" << std::endl;
            
            // Display frame
            cv::Mat frame = yuv_parser.frameToMat(data.frame_data.frame_data);
            if (!frame.empty()) {
                cv::Mat frame_with_text = frame.clone();
                std::string timestamp_text = "TS: " + std::to_string(data.timestamp);
                std::string frame_text = "Frame #" + std::to_string(frame_count);
                std::string mode_text = paused ? " [PAUSED]" : " [AUTO]";
                
                cv::putText(frame_with_text, timestamp_text, cv::Point(10, 25), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
                cv::putText(frame_with_text, frame_text, cv::Point(10, 45), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 2);
                cv::putText(frame_with_text, mode_text, cv::Point(10, 70), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
                
                cv::imshow("YUV Frame", frame_with_text);
                
                // Wait for key press or auto-advance
                int delay = auto_play && !paused ? frame_delay : 0;
                int key = cv::waitKey(delay) & 0xFF;
                
                if (key == 'q' || key == 'Q' || key == 27) {
                    std::cout << "Quitting..." << std::endl;
                    return 0;
                } else if (key == 'p' || key == 'P') {
                    paused = !paused;
                    std::cout << (paused ? "Paused" : "Resumed") << std::endl;
                    if (paused) {
                        cv::waitKey(0);
                        paused = false;
                    }
                }
            }
        } else if (data.type == TimestampedData::IMU) {
            imu_count++;
            std::cout << "[" << std::setw(10) << data.timestamp << "] IMU #" << imu_count << ": ";
            imu_parser.printIMUData(data.imu_data);
        }
    }
    
    std::cout << std::endl;
    std::cout << "=== Complete ===" << std::endl;
    std::cout << "Total frames: " << frame_count << std::endl;
    std::cout << "Total IMU entries: " << imu_count << std::endl;
    std::cout << "Total entries: " << merged_data.size() << std::endl;
    
    return 0;
}
