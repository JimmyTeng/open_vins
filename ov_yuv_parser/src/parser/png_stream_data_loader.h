#ifndef PNG_STREAM_DATA_LOADER_H
#define PNG_STREAM_DATA_LOADER_H

#include "imu_parser.h"
#include "yuv_parser.h"
#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "stream_data_loader.h"  // TimestampedData

/**
 * @brief 与 StreamDataLoader 相同的时间线调度，但帧在 reader 线程内用 cv::imread 从 png 目录加载。
 *
 * 约定：png 目录下文件名为 \<timestamp\>.png（vi_pts - exp/2，与 exportFramesToPNG 一致），
 * FrameInfo 仍从 dump_yuv 目录的 FrameInfo.txt 读取（与 YUV 流共用同一时间轴）。
 */
class PngStreamDataLoader {
public:
  PngStreamDataLoader(const std::string& png_dir,
                      const std::string& dump_yuv_dir_for_frameinfo,
                      const std::string& imu_file, size_t max_frame_cache = 4);
  ~PngStreamDataLoader();

  bool empty() const;
  size_t scheduleSize() const;
  void start();
  bool getNext(TimestampedData& out);
  void stop();

private:
  struct FrameRef {
    long long timestamp;
    FrameInfo info;
  };
  struct ScheduleItem {
    enum Type { FRAME, IMU };
    Type type;
    long long timestamp;
    size_t frame_ref_idx;
    size_t imu_idx;
  };
  void readerLoop();

  std::string png_dir_;
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
