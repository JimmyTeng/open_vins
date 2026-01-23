/*
 * ROS Node for publishing YUV frames and IMU data to OpenVINS topics
 * Publishes data in timestamp order
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include "yuv_parser.h"
#include "imu_parser.h"
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <chrono>
#include <iostream>
#include <string>

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

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "yuv_imu_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    // Get parameters
    std::string data_dir;
    nh_private.param<std::string>("data_dir", data_dir, "data/250704/8");
    
    std::string topic_imu;
    nh_private.param<std::string>("topic_imu", topic_imu, "/imu0");
    
    std::string topic_camera;
    nh_private.param<std::string>("topic_camera", topic_camera, "/cam0/image_raw");
    
    double publish_rate;
    nh_private.param<double>("publish_rate", publish_rate, 1.0);  // 1.0 means real-time, >1.0 means faster
    
    bool frame_by_frame;
    nh_private.param<bool>("frame_by_frame", frame_by_frame, false);  // 是否逐帧播放
    
    // Determine dump_yuv directory
    std::string dump_yuv_dir = data_dir;
    if (dump_yuv_dir.size() < 8 || dump_yuv_dir.substr(dump_yuv_dir.size() - 8) != "dump_yuv") {
        if (!dump_yuv_dir.empty() && dump_yuv_dir.back() != '/' && dump_yuv_dir.back() != '\\') {
            dump_yuv_dir += "/";
        }
        dump_yuv_dir += "dump_yuv";
    }
    
    std::string info_file = dump_yuv_dir + "/FrameInfo.txt";
    // IMU 文件通常与 dump_yuv 目录处于同一级目录，因此这里回到上一层
    // 形如: /path/to/data/xxx/dump_yuv/../imu.txt -> /path/to/data/xxx/imu.txt
    std::string imu_file = dump_yuv_dir + "/../imu.txt";
    
    ROS_INFO("=== YUV/IMU Publisher Node ===");
    ROS_INFO("Data directory: %s", data_dir.c_str());
    ROS_INFO("Dump YUV directory: %s", dump_yuv_dir.c_str());
    ROS_INFO("IMU topic: %s", topic_imu.c_str());
    ROS_INFO("Camera topic: %s", topic_camera.c_str());
    ROS_INFO("Publish rate: %.2fx", publish_rate);
    ROS_INFO("Frame-by-frame mode: %s", frame_by_frame ? "ENABLED" : "DISABLED");
    if (frame_by_frame) {
        ROS_INFO("  -> Press ENTER to continue to next frame");
    }
    
    // Create parsers
    YUVParser yuv_parser(640, 480);
    IMUParser imu_parser;
    
    // Parse frame data
    ROS_INFO("Parsing FrameInfo.txt and YUV files...");
    std::vector<FrameData> frame_data = yuv_parser.parseAllFrames(dump_yuv_dir);
    ROS_INFO("Found %zu frames", frame_data.size());
    
    // Parse IMU data
    ROS_INFO("Parsing imu.txt...");
    std::vector<IMUData> imu_data = imu_parser.parseIMUFile(imu_file);
    ROS_INFO("Found %zu IMU entries", imu_data.size());
    
    if (frame_data.empty() && imu_data.empty()) {
        ROS_ERROR("No data found!");
        return 1;
    }
    
    // Merge and sort by timestamp
    ROS_INFO("Merging data by timestamp...");
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
    std::sort(merged_data.begin(), merged_data.end());
    
    ROS_INFO("Total merged entries: %zu", merged_data.size());
    if (!merged_data.empty()) {
        ROS_INFO("Time range: %lld to %lld", merged_data[0].timestamp, merged_data.back().timestamp);
    }
    
    // Create publishers
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>(topic_imu, 1000);
    ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>(topic_camera, 10);
    
    // Wait for subscribers
    ROS_INFO("Waiting for subscribers...");
    int wait_count = 0;
    while (pub_imu.getNumSubscribers() == 0 && pub_image.getNumSubscribers() == 0 && ros::ok()) {
        ros::Duration(0.1).sleep();
        wait_count++;
    }
    ROS_INFO("Subscribers connected. Starting to publish...");
    
    // Calculate time offset (use first timestamp as reference)
    long long time_offset = 0;
    if (!merged_data.empty()) {
        time_offset = merged_data[0].timestamp;
    }
    
    // Convert PTS to ROS time
    // PTS appears to be in some time unit (possibly microseconds or clock ticks)
    // Adjust the conversion factor based on your actual timestamp format
    // For now, assuming PTS is in microseconds (1e6 per second)
    // If your PTS is in a different unit, adjust the divisor accordingly
    double pts_to_seconds_factor = 1e6;  // Change this if your PTS unit is different
    
    // Use fixed base time to ensure consistent timestamps that match TF tree
    ros::Time base_time = ros::Time::now();
    
    auto pts_to_ros_time = [&time_offset, pts_to_seconds_factor, base_time](long long pts) -> ros::Time {
        double time_sec = (pts - time_offset) / pts_to_seconds_factor;
        return base_time + ros::Duration(time_sec);
    };
    
    // Publish data in timestamp order
    // Use the same base_time for all messages to ensure timestamps are consistent with TF tree
    long long first_timestamp = merged_data.empty() ? 0 : merged_data[0].timestamp;
    
    int frame_count = 0;
    int imu_count = 0;
    long long last_frame_timestamp = 0;
    long long last_imu_timestamp = 0;
    
    ROS_INFO("=== Starting Data Publication ===");
    ROS_INFO("Base time: %.6f (ROS time)", base_time.toSec());
    ROS_INFO("Time offset (first timestamp): %lld", time_offset);
    ROS_INFO("PTS to seconds factor: %.0f", pts_to_seconds_factor);
    ROS_INFO("First timestamp: %lld", first_timestamp);
    ROS_INFO("Total data entries: %zu", merged_data.size());
    
    for (size_t i = 0; i < merged_data.size() && ros::ok(); ++i) {
        const TimestampedData& data = merged_data[i];
        
        // Calculate when this message should be published (for rate control)
        double time_since_start = (data.timestamp - first_timestamp) / pts_to_seconds_factor / publish_rate;
        ros::Time target_time = base_time + ros::Duration(time_since_start);
        
        // Wait until it's time to publish
        ros::Time current_time = ros::Time::now();
        if (target_time > current_time) {
            ros::Duration(target_time - current_time).sleep();
        }
        
        if (data.type == TimestampedData::FRAME) {
            frame_count++;
            
            // Calculate time interval since last frame
            double frame_interval = 0.0;
            if (last_frame_timestamp > 0) {
                frame_interval = (data.timestamp - last_frame_timestamp) / pts_to_seconds_factor;
            }
            last_frame_timestamp = data.timestamp;
            
            // Convert frame to OpenCV Mat
            cv::Mat frame = yuv_parser.frameToMat(data.frame_data.frame_data);
            if (!frame.empty()) {
                // Convert to ROS Image message
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(
                    std_msgs::Header(), "mono8", frame).toImageMsg();
                
                // Set timestamp (convert PTS to ROS time)
                ros::Time frame_stamp = pts_to_ros_time(data.timestamp);
                img_msg->header.stamp = frame_stamp;
                img_msg->header.frame_id = "cam0";
                img_msg->header.seq = frame_count;
                
                pub_image.publish(img_msg);
                
                // Print detailed information
                bool should_print = frame_by_frame || (frame_count <= 5 || frame_count % 50 == 0);
                if (should_print) {
                    ROS_INFO("=== FRAME #%d ===", frame_count);
                    ROS_INFO("  PTS timestamp: %lld (raw)", data.timestamp);
                    ROS_INFO("  PTS relative: %lld (offset from first: %lld)", 
                             data.timestamp - time_offset, data.timestamp - first_timestamp);
                    ROS_INFO("  ROS timestamp: %.6f (sec: %d, nsec: %d)", 
                             frame_stamp.toSec(), frame_stamp.sec, frame_stamp.nsec);
                    ROS_INFO("  Time since start: %.6f sec", time_since_start);
                    ROS_INFO("  Frame interval: %.6f sec (%.3f ms)", 
                             frame_interval, frame_interval * 1000.0);
                    ROS_INFO("  Frame size: %dx%d, encoding: %s, step: %u, data size: %zu bytes",
                             frame.cols, frame.rows, img_msg->encoding.c_str(), 
                             img_msg->step, img_msg->data.size());
                    ROS_INFO("  Subscribers: %d", pub_image.getNumSubscribers());
                }
                
                // 逐帧模式：等待用户输入
                if (frame_by_frame) {
                    ROS_INFO("  [逐帧模式] 按 ENTER 键继续下一帧...");
                    std::string line;
                    std::getline(std::cin, line);
                }
            }
        } else if (data.type == TimestampedData::IMU) {
            imu_count++;
            
            // Calculate time interval since last IMU
            double imu_interval = 0.0;
            if (last_imu_timestamp > 0) {
                imu_interval = (data.timestamp - last_imu_timestamp) / pts_to_seconds_factor;
            }
            last_imu_timestamp = data.timestamp;
            
            // Create IMU message
            sensor_msgs::Imu imu_msg;
            ros::Time imu_stamp = pts_to_ros_time(data.timestamp);
            imu_msg.header.stamp = imu_stamp;
            imu_msg.header.frame_id = "imu0";
            imu_msg.header.seq = imu_count;
            
            // Set linear acceleration (m/s^2)
            imu_msg.linear_acceleration.x = data.imu_data.accX;
            imu_msg.linear_acceleration.y = data.imu_data.accY;
            imu_msg.linear_acceleration.z = data.imu_data.accZ;
            
            // Set angular velocity (rad/s)
            imu_msg.angular_velocity.x = data.imu_data.gyroX;
            imu_msg.angular_velocity.y = data.imu_data.gyroY;
            imu_msg.angular_velocity.z = data.imu_data.gyroZ;
            
            // Set orientation covariance (unknown)
            imu_msg.orientation_covariance[0] = -1;
            
            // Set linear acceleration covariance (unknown)
            imu_msg.linear_acceleration_covariance[0] = -1;
            
            // Set angular velocity covariance (unknown)
            imu_msg.angular_velocity_covariance[0] = -1;
            
            pub_imu.publish(imu_msg);
            
            // Print detailed information
            if (imu_count <= 10 || imu_count % 500 == 0) {
                ROS_INFO("=== IMU #%d ===", imu_count);
                ROS_INFO("  PTS timestamp: %lld (raw)", data.timestamp);
                ROS_INFO("  PTS relative: %lld (offset from first: %lld)", 
                         data.timestamp - time_offset, data.timestamp - first_timestamp);
                ROS_INFO("  ROS timestamp: %.6f (sec: %d, nsec: %d)", 
                         imu_stamp.toSec(), imu_stamp.sec, imu_stamp.nsec);
                ROS_INFO("  Time since start: %.6f sec", time_since_start);
                ROS_INFO("  IMU interval: %.6f sec (%.3f ms)", 
                         imu_interval, imu_interval * 1000.0);
                ROS_INFO("  Linear acceleration: [%.6f, %.6f, %.6f] m/s^2",
                         data.imu_data.accX, data.imu_data.accY, data.imu_data.accZ);
                ROS_INFO("  Angular velocity: [%.6f, %.6f, %.6f] rad/s",
                         data.imu_data.gyroX, data.imu_data.gyroY, data.imu_data.gyroZ);
                ROS_INFO("  Subscribers: %d", pub_imu.getNumSubscribers());
            }
        }
        
        ros::spinOnce();
    }
    
    ROS_INFO("=== Publishing Complete ===");
    ROS_INFO("Total frames published: %d", frame_count);
    ROS_INFO("Total IMU entries published: %d", imu_count);
    
    return 0;
}
