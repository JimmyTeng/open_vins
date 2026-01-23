#ifndef YUV_PARSER_H
#define YUV_PARSER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

struct FrameInfo {
    std::string filename;
    int frame_count;
    long long vi_pts;      // Video PTS (timestamp)
    int exp_time;          // Exposure time
    int width;
    int height;
    
    // 用于时间戳比较
    bool operator<(const FrameInfo& other) const {
        return vi_pts < other.vi_pts;
    }
};

struct FrameData {
    FrameInfo info;
    std::vector<unsigned char> frame_data;
    long long timestamp;  // 使用 vi_pts 作为时间戳
    
    // 用于时间戳比较
    bool operator<(const FrameData& other) const {
        return timestamp < other.timestamp;
    }
};

class YUVParser {
private:
    int width_;
    int height_;
    int frame_size_;  // Size of one frame in bytes (YUV400: width * height)
    
public:
    YUVParser(int width = 640, int height = 480);
    
    /**
     * @brief Parse FrameInfo.txt file
     * @param info_file Path to FrameInfo.txt
     * @return Vector of frame information
     */
    std::vector<FrameInfo> parseFrameInfo(const std::string& info_file);
    
    /**
     * @brief Read a single frame from YUV file
     * @param file Input file stream (already opened and positioned)
     * @param frame_data Output buffer for frame data
     * @return true if successful, false otherwise
     */
    bool readFrame(std::ifstream& file, std::vector<unsigned char>& frame_data);
    
    /**
     * @brief Parse all frames from a YUV file
     * @param yuv_file Path to YUV file
     * @param num_frames Number of frames to read (default: 100)
     * @return Vector of frames, each frame is a vector of unsigned char
     */
    std::vector<std::vector<unsigned char>> parseYUVFile(
        const std::string& yuv_file, 
        int num_frames = 100);
    
    /**
     * @brief Parse all frame data with timestamps
     * @param dump_yuv_dir Directory containing YUV files and FrameInfo.txt
     * @return Vector of FrameData sorted by timestamp
     */
    std::vector<FrameData> parseAllFrames(const std::string& dump_yuv_dir);
    
    /**
     * @brief Convert YUV400 frame to OpenCV Mat (grayscale)
     * @param frame_data YUV400 frame data
     * @return OpenCV Mat (CV_8UC1)
     */
    cv::Mat frameToMat(const std::vector<unsigned char>& frame_data);
    
    /**
     * @brief Save frame as image
     * @param frame_data Frame data
     * @param output_path Output image path
     * @return true if successful
     */
    bool saveFrameAsImage(const std::vector<unsigned char>& frame_data, 
                         const std::string& output_path);
    
    /**
     * @brief Get frame statistics
     * @param frame_data Frame data
     */
    void printFrameStats(const std::vector<unsigned char>& frame_data);
    
    /**
     * @brief Export all frames from dump_yuv directory to PNG files
     * @param dump_yuv_dir Directory containing YUV files and FrameInfo.txt
     * @return Number of frames exported, or -1 on error
     */
    int exportFramesToPNG(const std::string& dump_yuv_dir);
    
    /**
     * @brief Play frames one by one with IMU data, sorted by timestamp
     * @param dump_yuv_dir Directory containing YUV files and FrameInfo.txt
     * @param imu_file Path to imu.txt file
     * @return 0 on success, -1 on error
     */
    int playFramesStepByStep(const std::string& dump_yuv_dir, const std::string& imu_file);
    
    // Getters
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    int getFrameSize() const { return frame_size_; }
};

#endif // YUV_PARSER_H
