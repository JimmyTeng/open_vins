#include "stream_data_loader.h"

#include <algorithm>

StreamDataLoader::StreamDataLoader(YUVParser* yuv_parser,
                                   const std::string& dump_yuv_dir,
                                   const std::string& imu_file,
                                   size_t max_frame_cache)
    : yuv_parser_(yuv_parser),
      max_frame_cache_(max_frame_cache),
      done_(false),
      schedule_index_(0) {
  std::string info_file = dump_yuv_dir + "/FrameInfo.txt";
  std::vector<FrameInfo> frame_infos = yuv_parser_->parseFrameInfo(info_file);
  std::map<std::string, std::vector<FrameInfo>> file_groups;
  for (const auto& info : frame_infos) {
    file_groups[info.filename].push_back(info);
  }
  for (const auto& fg : file_groups) {
    const std::string& filename = fg.first;
    const std::vector<FrameInfo>& infos = fg.second;
    std::string yuv_path = dump_yuv_dir + "/" + filename;
    for (size_t i = 0; i < infos.size(); ++i) {
      FrameRef ref;
      ref.yuv_path = yuv_path;
      ref.frame_index = static_cast<int>(i);
      ref.info = infos[i];
      ref.timestamp = infos[i].vi_pts;
      frame_refs_.push_back(ref);
    }
  }
  imu_data_ = IMUParser().parseIMUFile(imu_file);
  for (size_t i = 0; i < frame_refs_.size(); ++i) {
    ScheduleItem item;
    item.type = ScheduleItem::FRAME;
    item.timestamp = frame_refs_[i].timestamp;
    item.frame_ref_idx = i;
    item.imu_idx = 0;
    schedule_.push_back(item);
  }
  for (size_t i = 0; i < imu_data_.size(); ++i) {
    ScheduleItem item;
    item.type = ScheduleItem::IMU;
    item.timestamp = imu_data_[i].timeStamp;
    item.frame_ref_idx = 0;
    item.imu_idx = i;
    schedule_.push_back(item);
  }
  std::sort(schedule_.begin(), schedule_.end(),
            [](const ScheduleItem& a, const ScheduleItem& b) {
              return a.timestamp < b.timestamp;
            });
}

StreamDataLoader::~StreamDataLoader() { stop(); }

bool StreamDataLoader::empty() const { return schedule_.empty(); }
size_t StreamDataLoader::scheduleSize() const { return schedule_.size(); }

void StreamDataLoader::start() {
  reader_thread_ = std::thread(&StreamDataLoader::readerLoop, this);
}

bool StreamDataLoader::getNext(TimestampedData& out) {
  while (schedule_index_ < schedule_.size()) {
    const ScheduleItem& item = schedule_[schedule_index_];
    schedule_index_++;
    if (item.type == ScheduleItem::IMU) {
      out.type = TimestampedData::IMU;
      out.timestamp = item.timestamp;
      out.imu_data = imu_data_[item.imu_idx];
      return true;
    }
    std::unique_lock<std::mutex> lock(cache_mutex_);
    cache_cv_.wait(lock, [this] { return !frame_cache_.empty() || done_; });
    if (!frame_cache_.empty()) {
      out.type = TimestampedData::FRAME;
      out.timestamp = item.timestamp;
      out.frame_data = std::move(frame_cache_.front());
      frame_cache_.pop();
      cache_cv_.notify_one();
      return true;
    }
    if (done_) break;
  }
  return false;
}

void StreamDataLoader::stop() {
  done_ = true;
  cache_cv_.notify_all();
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

void StreamDataLoader::readerLoop() {
  for (size_t i = 0; i < schedule_.size() && !done_; ++i) {
    if (schedule_[i].type != ScheduleItem::FRAME) continue;
    const FrameRef& ref = frame_refs_[schedule_[i].frame_ref_idx];
    std::vector<unsigned char> frame_data;
    if (!yuv_parser_->readFrameAt(ref.yuv_path, ref.frame_index, frame_data))
      continue;
    FrameData fd;
    fd.info = ref.info;
    fd.timestamp = ref.timestamp;
    fd.frame_data = std::move(frame_data);
    {
      std::unique_lock<std::mutex> lock(cache_mutex_);
      cache_cv_.wait(lock, [this] {
        return frame_cache_.size() < max_frame_cache_ || done_;
      });
      if (done_) break;
      frame_cache_.push(std::move(fd));
      cache_cv_.notify_one();
    }
  }
  done_ = true;
  cache_cv_.notify_all();
}
