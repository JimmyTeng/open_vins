/*
 * VIO 数据集记录 / 回放（与 ov_core::ImuData、ov_core::CameraData 对齐，供 VioManager::feed_measurement_* 使用）
 */

#ifndef OV_CORE_VIO_DATA_RECORD_PLAY_H
#define OV_CORE_VIO_DATA_RECORD_PLAY_H

#include "system/ov_core_export.h"
#include "utils/sensor_data.h"

#include <condition_variable>
#include <deque>
#include <fstream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ov_core {

/**
 * @brief 将 IMU 与相机数据落盘：imu.csv + camera_index.csv + mosaics/mosaic_XXXXXX.png
 *
 * 默认布局为 **1 列 × 15 行**（一包 15 张）：小图自上而下纵向拼接成一条图带。
 * **宽度**固定为 tile_w（须与每帧图像 cols 一致）；**高度**按每帧实际 rows 可变，使用 vconcat 合成 PNG，无需补 0。
 * camera_index.csv 含列 row_h 记录该行高度；dataset_meta 中 tile_height=0 表示行高见 CSV。
 */
class VioDataRecorder {
public:
  /// @param grid_cols 水平格数（默认 1，竖条）
  /// @param grid_rows 竖直格数（默认 15，即一包 15 张图）
  /// @param tile_w 固定图像宽度；tile_h 仅写入 meta 占位时可忽略，行高以实际图像为准
  VioDataRecorder(const std::string &output_dir, int grid_cols = 1, int grid_rows = 15, int tile_w = 640, int tile_h = 480);

  VioDataRecorder(const VioDataRecorder &) = delete;
  VioDataRecorder &operator=(const VioDataRecorder &) = delete;

  ~VioDataRecorder();

  void record_imu(const ImuData &imu);

  /// 与 VioManager::feed_measurement_camera 入参一致；masks 不落盘，回放时 masks 为空
  void record_camera(const CameraData &cam);

  /// 将当前未满一包（grid_rows 张）的条带直接 vconcat 写出（不补行）
  void flush();

  int grid_cols() const { return grid_cols_; }
  int grid_rows() const { return grid_rows_; }
  int tile_width() const { return tile_w_; }
  int tile_height() const { return tile_h_; }

private:
  int tile_count() const { return grid_cols_ * grid_rows_; }
  void finalize_current_mosaic();

  std::string output_dir_;
  int grid_cols_;
  int grid_rows_;
  int tile_w_;
  int tile_h_;

  std::ofstream f_imu_;
  std::ofstream f_cam_;

  int mosaic_index_ = 0;
  int canvas_type_ = -1;
  std::vector<cv::Mat> mosaic_rows_;
};

/**
 * @brief 封装 VioDataRecorder：后台线程异步写盘（push 时深拷贝图像，避免与前端线程竞争）
 */
class VioDatasetWriter {
public:
  explicit VioDatasetWriter(std::unique_ptr<VioDataRecorder> recorder);
  VioDatasetWriter(const VioDatasetWriter &) = delete;
  VioDatasetWriter &operator=(const VioDatasetWriter &) = delete;
  ~VioDatasetWriter();

  void push_imu(const ImuData &imu);
  void push_camera(const CameraData &cam);

private:
  struct Job {
    enum class Kind { Imu, Camera } kind = Kind::Imu;
    ImuData imu;
    CameraData camera;
  };

  static CameraData clone_camera_payload(const CameraData &in);
  void worker_loop();

  std::unique_ptr<VioDataRecorder> recorder_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<Job> queue_;
  bool stop_ = false;
  std::thread worker_;
};

/**
 * @brief 回放 VioDataRecorder 生成的数据集，按时间戳合并 IMU 与相机事件
 *
 * mosaic PNG 在后台线程异步加载；内存中最多保留 **2** 个完整 mosaic 文件（LRU），与「一包 15 张」对应约 2×15 张小图缓存。
 *
 * OV_CORE_API：供 vio_png_runner 等可执行文件链接 libov_core（visibility=hidden 时需显式导出）。
 */
class OV_CORE_API VioDataPlayer {
public:
  struct PlaybackEvent {
    enum class Type { Imu, Camera } type = Type::Imu;
    ImuData imu;
    CameraData camera;
  };

  VioDataPlayer() = default;
  explicit VioDataPlayer(const std::string &dataset_dir) { open(dataset_dir); }
  ~VioDataPlayer();

  VioDataPlayer(const VioDataPlayer &) = delete;
  VioDataPlayer &operator=(const VioDataPlayer &) = delete;

  /// @return 目录下至少存在 imu.csv 或 camera_index 之一且可读时返回 true（可仅含表头）
  bool open(const std::string &dataset_dir);

  void reset();

  bool has_next() const;

  /// 时间戳非递减顺序；相机事件已将同一 timestamp 的多目合并为一条 CameraData
  bool next_event(PlaybackEvent &out);

  int grid_cols() const { return grid_cols_; }
  int grid_rows() const { return grid_rows_; }
  int tile_width() const { return tile_w_; }
  int tile_height() const { return tile_h_; }

  const std::vector<ImuData> &imu_buffer() const { return imus_; }

private:
  struct CamCellRef {
    double timestamp = 0.0;
    int cam_id = 0;
    std::string mosaic_path;
    int cell_idx = 0;
    /// 该行小图高度；0 表示旧数据集（统一用 meta 的 tile_height）
    int row_h = 0;
  };

  static bool parse_meta_file(const std::string &path, int &gc, int &gr, int &tw, int &th);
  static std::vector<ImuData> load_imu_csv(const std::string &path);
  static std::vector<CamCellRef> load_camera_csv(const std::string &path);
  cv::Mat get_tile(const CamCellRef &ref);

  void build_mosaic_playback_order();
  void prefetch_following_unlocked(const std::string &loaded_rel);
  void insert_mosaic_lru_unlocked(const std::string &key, cv::Mat &&mat);
  void touch_mosaic_lru_unlocked(const std::string &key);
  void loader_loop();
  void stop_loader_thread();

  static constexpr size_t kMosaicCacheCapacity = 2;

  std::string dataset_dir_;
  int grid_cols_ = 1;
  int grid_rows_ = 15;
  int tile_w_ = 640;
  int tile_h_ = 0;

  std::vector<ImuData> imus_;
  size_t imu_i_ = 0;

  std::vector<CamCellRef> cam_cells_;
  size_t cam_i_ = 0;

  /// 按时间顺序首次出现的 mosaic 文件名（用于预取下一包）
  std::vector<std::string> mosaic_playback_order_;

  std::unordered_map<std::string, cv::Mat> mosaic_cache_;
  std::list<std::string> mosaic_lru_;

  std::thread loader_thread_;
  std::mutex load_mtx_;
  std::condition_variable load_cv_;
  std::condition_variable loaded_cv_;
  std::deque<std::string> load_queue_;
  std::unordered_set<std::string> loading_;
  std::unordered_set<std::string> load_failed_;
  bool loader_stop_ = false;

  /// mosaic 文件名 -> 各 cell_idx 行高（回放可变行高时用）
  std::unordered_map<std::string, std::vector<int>> mosaic_heights_;
};

} // namespace ov_core

#endif // OV_CORE_VIO_DATA_RECORD_PLAY_H
