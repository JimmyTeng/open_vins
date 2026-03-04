#ifndef STREAM_DATA_LOADER_H
#define STREAM_DATA_LOADER_H

#include "yuv_parser.h"
#include "imu_parser.h"
#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// 统一的数据结构，用于按时间戳排序
struct TimestampedData {
    enum DataType { FRAME, IMU };
    DataType type;
    long long timestamp;
    FrameData frame_data;
    IMUData imu_data;
    bool operator<(const TimestampedData& other) const { return timestamp < other.timestamp; }
};

// 流式数据加载器：单独线程预读帧到有界缓存，控制内存
class StreamDataLoader {
public:
    StreamDataLoader(YUVParser* yuv_parser, const std::string& dump_yuv_dir,
                     const std::string& imu_file, size_t max_frame_cache = 4);
    ~StreamDataLoader();

    bool empty() const;
    size_t scheduleSize() const;
    void start();
    bool getNext(TimestampedData& out);
    void stop();

private:
    struct FrameRef {
        std::string yuv_path;
        int frame_index;
        FrameInfo info;
        long long timestamp;
    };
    struct ScheduleItem {
        enum Type { FRAME, IMU };
        Type type;
        long long timestamp;
        size_t frame_ref_idx;
        size_t imu_idx;
    };
    void readerLoop();

    YUVParser* yuv_parser_;
    size_t max_frame_cache_;
    std::vector<FrameRef> frame_refs_;
    std::vector<IMUData> imu_data_;
    std::vector<ScheduleItem> schedule_;
    std::atomic<bool> done_;
    size_t schedule_index_;
    std::queue<FrameData> frame_cache_;
    std::mutex cache_mutex_;
    std::condition_variable cache_cv_;
    std::thread reader_thread_;
};

#endif
