#include "utils/vio_data_record_play.h"
#include "utils/fs_compat.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ov_core {

namespace {

std::string trim(std::string s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r'))
    s.erase(s.begin());
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
    s.pop_back();
  return s;
}

std::vector<std::string> split_csv_line(const std::string &line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(trim(cur));
      cur.clear();
    } else
      cur += c;
  }
  out.push_back(trim(cur));
  return out;
}

} // namespace

// ---------------------------------------------------------------------------
// VioDataRecorder
// ---------------------------------------------------------------------------

VioDataRecorder::VioDataRecorder(const std::string &output_dir, int grid_cols, int grid_rows, int tile_w, int tile_h)
    : output_dir_(output_dir), grid_cols_(grid_cols), grid_rows_(grid_rows), tile_w_(tile_w), tile_h_(tile_h) {

  if (grid_cols_ < 1 || grid_rows_ < 1 || tile_w_ < 1)
    throw std::invalid_argument("VioDataRecorder: grid and tile width must be positive");

  std::filesystem::path root(output_dir_);
  std::filesystem::create_directories(root);
  std::filesystem::create_directories(root / "mosaics");

  {
    std::ofstream meta(root / "dataset_meta.txt");
    if (!meta.is_open())
      throw std::runtime_error("VioDataRecorder: cannot write dataset_meta.txt");
    meta << "tile_width " << tile_w_ << "\n";
    meta << "tile_height 0\n";
    meta << "grid_cols " << grid_cols_ << "\n";
    meta << "grid_rows " << grid_rows_ << "\n";
  }

  f_imu_.open(root / "imu.csv");
  f_cam_.open(root / "camera_index.csv");
  if (!f_imu_.is_open() || !f_cam_.is_open())
    throw std::runtime_error("VioDataRecorder: cannot open imu.csv or camera_index.csv");

  f_imu_ << "timestamp_s,wx,wy,wz,ax,ay,az\n";
  f_cam_ << "timestamp_s,cam_id,mosaic_png,cell_idx,row_h\n";
  f_imu_ << std::setprecision(17);
  f_cam_ << std::setprecision(17);
}

VioDataRecorder::~VioDataRecorder() {
  try {
    flush();
  } catch (...) {
  }
}

void VioDataRecorder::record_imu(const ImuData &imu) {
  f_imu_ << imu.timestamp << ',' << imu.wm(0) << ',' << imu.wm(1) << ',' << imu.wm(2) << ',' << imu.am(0) << ','
         << imu.am(1) << ',' << imu.am(2) << '\n';
}

void VioDataRecorder::finalize_current_mosaic() {
  if (mosaic_rows_.empty())
    return;
  cv::Mat stacked;
  cv::vconcat(mosaic_rows_, stacked);
  std::ostringstream stem;
  stem << "mosaic_" << std::setfill('0') << std::setw(6) << mosaic_index_ << ".png";
  const std::filesystem::path abs_path = std::filesystem::path(output_dir_) / "mosaics" / stem.str();
  if (!cv::imwrite(abs_path.string(), stacked))
    throw std::runtime_error("VioDataRecorder: cv::imwrite failed for " + abs_path.string());
  mosaic_rows_.clear();
  canvas_type_ = -1;
  mosaic_index_++;
}

void VioDataRecorder::record_camera(const CameraData &cam) {
  if (cam.images.empty())
    return;
  if (cam.sensor_ids.size() != cam.images.size())
    throw std::invalid_argument("VioDataRecorder: sensor_ids.size() must match images.size()");

  const size_t n = cam.images.size();
  if (static_cast<int>(n) > tile_count())
    throw std::invalid_argument("VioDataRecorder: more images in one CameraData than mosaic capacity");

  for (size_t i = 0; i < n; ++i) {
    const cv::Mat &im = cam.images[i];
    if (im.empty())
      throw std::invalid_argument("VioDataRecorder: empty image");
    if (im.cols != tile_w_)
      throw std::invalid_argument("VioDataRecorder: image width must be " + std::to_string(tile_w_));
    if (im.rows < 1)
      throw std::invalid_argument("VioDataRecorder: image height must be positive");

    if (canvas_type_ < 0)
      canvas_type_ = im.type();
    else if (im.type() != canvas_type_)
      throw std::invalid_argument("VioDataRecorder: all images in one dataset must have same OpenCV type as first tile");

    mosaic_rows_.push_back(im);
    const int idx = static_cast<int>(mosaic_rows_.size()) - 1;

    std::ostringstream rel;
    rel << "mosaics/mosaic_" << std::setfill('0') << std::setw(6) << mosaic_index_ << ".png";
    f_cam_ << cam.timestamp << ',' << cam.sensor_ids[i] << ',' << rel.str() << ',' << idx << ',' << im.rows << '\n';

    if (static_cast<int>(mosaic_rows_.size()) == tile_count())
      finalize_current_mosaic();
  }
}

void VioDataRecorder::flush() {
  finalize_current_mosaic();
  if (f_imu_.is_open())
    f_imu_.flush();
  if (f_cam_.is_open())
    f_cam_.flush();
}

// ---------------------------------------------------------------------------
// VioDatasetWriter
// ---------------------------------------------------------------------------

CameraData VioDatasetWriter::clone_camera_payload(const CameraData &in) {
  CameraData o;
  o.timestamp = in.timestamp;
  o.sensor_ids = in.sensor_ids;
  for (const auto &im : in.images)
    o.images.push_back(im.clone());
  return o;
}

VioDatasetWriter::VioDatasetWriter(std::unique_ptr<VioDataRecorder> recorder)
    : recorder_(std::move(recorder)) {
  if (!recorder_)
    throw std::invalid_argument("VioDatasetWriter: null recorder");
  worker_ = std::thread([this] { worker_loop(); });
}

VioDatasetWriter::~VioDatasetWriter() {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable())
    worker_.join();
  try {
    recorder_->flush();
  } catch (...) {
  }
}

void VioDatasetWriter::push_imu(const ImuData &imu) {
  Job j;
  j.kind = Job::Kind::Imu;
  j.imu = imu;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    queue_.push_back(std::move(j));
  }
  cv_.notify_one();
}

void VioDatasetWriter::push_camera(const CameraData &cam) {
  Job j;
  j.kind = Job::Kind::Camera;
  j.camera = clone_camera_payload(cam);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    queue_.push_back(std::move(j));
  }
  cv_.notify_one();
}

void VioDatasetWriter::worker_loop() {
  while (true) {
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait(lk, [this] { return stop_ || !queue_.empty(); });
    while (!queue_.empty()) {
      Job job = std::move(queue_.front());
      queue_.pop_front();
      lk.unlock();
      if (job.kind == Job::Kind::Imu)
        recorder_->record_imu(job.imu);
      else
        recorder_->record_camera(job.camera);
      lk.lock();
    }
    if (stop_)
      break;
  }
}

// ---------------------------------------------------------------------------
// VioDataPlayer
// ---------------------------------------------------------------------------

VioDataPlayer::~VioDataPlayer() { stop_loader_thread(); }

void VioDataPlayer::stop_loader_thread() {
  {
    std::lock_guard<std::mutex> lk(load_mtx_);
    loader_stop_ = true;
  }
  load_cv_.notify_all();
  if (loader_thread_.joinable())
    loader_thread_.join();
  std::lock_guard<std::mutex> lk(load_mtx_);
  loader_stop_ = false;
  load_queue_.clear();
  loading_.clear();
  load_failed_.clear();
}

void VioDataPlayer::build_mosaic_playback_order() {
  mosaic_playback_order_.clear();
  std::string last;
  for (const auto &c : cam_cells_) {
    if (c.mosaic_path != last) {
      mosaic_playback_order_.push_back(c.mosaic_path);
      last = c.mosaic_path;
    }
  }
}

void VioDataPlayer::insert_mosaic_lru_unlocked(const std::string &key, cv::Mat &&mat) {
  if (mosaic_cache_.size() >= kMosaicCacheCapacity && !mosaic_cache_.count(key)) {
    while (mosaic_cache_.size() >= kMosaicCacheCapacity) {
      const std::string ev = mosaic_lru_.front();
      mosaic_lru_.pop_front();
      mosaic_cache_.erase(ev);
    }
  }
  mosaic_cache_[key] = std::move(mat);
  mosaic_lru_.remove(key);
  mosaic_lru_.push_back(key);
}

void VioDataPlayer::touch_mosaic_lru_unlocked(const std::string &key) {
  mosaic_lru_.remove(key);
  mosaic_lru_.push_back(key);
}

void VioDataPlayer::prefetch_following_unlocked(const std::string &loaded_rel) {
  auto it = std::find(mosaic_playback_order_.begin(), mosaic_playback_order_.end(), loaded_rel);
  if (it == mosaic_playback_order_.end())
    return;
  for (++it; it != mosaic_playback_order_.end(); ++it) {
    const std::string &next = *it;
    if (mosaic_cache_.count(next) || loading_.count(next) || load_failed_.count(next))
      continue;
    load_queue_.push_back(next);
    loading_.insert(next);
    load_cv_.notify_one();
    break;
  }
}

void VioDataPlayer::loader_loop() {
  while (true) {
    std::string rel;
    {
      std::unique_lock<std::mutex> lk(load_mtx_);
      load_cv_.wait(lk, [this] { return loader_stop_ || !load_queue_.empty(); });
      if (loader_stop_)
        break;
      rel = load_queue_.front();
      load_queue_.pop_front();
    }

    std::filesystem::path p = std::filesystem::path(dataset_dir_) / rel;
    cv::Mat full = cv::imread(p.string(), cv::IMREAD_UNCHANGED);

    {
      std::lock_guard<std::mutex> lk(load_mtx_);
      loading_.erase(rel);
      if (full.empty()) {
        load_failed_.insert(rel);
      } else {
        insert_mosaic_lru_unlocked(rel, std::move(full));
        prefetch_following_unlocked(rel);
      }
    }
    loaded_cv_.notify_all();
  }
}

bool VioDataPlayer::parse_meta_file(const std::string &path, int &gc, int &gr, int &tw, int &th) {
  std::ifstream f(path);
  if (!f.is_open())
    return false;
  gc = gr = tw = th = -1;
  std::string line;
  while (std::getline(f, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream iss(line);
    std::string key;
    int val = 0;
    if (!(iss >> key >> val))
      continue;
    if (key == "tile_width")
      tw = val;
    else if (key == "tile_height")
      th = val;
    else if (key == "grid_cols")
      gc = val;
    else if (key == "grid_rows")
      gr = val;
  }
  return tw > 0 && th >= 0 && gc > 0 && gr > 0;
}

std::vector<ImuData> VioDataPlayer::load_imu_csv(const std::string &path) {
  std::vector<ImuData> out;
  std::ifstream f(path);
  if (!f.is_open())
    return out;
  std::string line;
  if (!std::getline(f, line))
    return out;
  while (std::getline(f, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    auto p = split_csv_line(line);
    if (p.size() < 7)
      continue;
    ImuData d;
    d.timestamp = std::stod(p[0]);
    d.wm << std::stod(p[1]), std::stod(p[2]), std::stod(p[3]);
    d.am << std::stod(p[4]), std::stod(p[5]), std::stod(p[6]);
    out.push_back(d);
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<VioDataPlayer::CamCellRef> VioDataPlayer::load_camera_csv(const std::string &path) {
  std::vector<CamCellRef> out;
  std::ifstream f(path);
  if (!f.is_open())
    return out;
  std::string line;
  if (!std::getline(f, line))
    return out;
  while (std::getline(f, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    auto p = split_csv_line(line);
    if (p.size() < 4)
      continue;
    CamCellRef r;
    r.timestamp = std::stod(p[0]);
    r.cam_id = std::stoi(p[1]);
    r.mosaic_path = p[2];
    r.cell_idx = std::stoi(p[3]);
    r.row_h = (p.size() >= 5) ? std::stoi(p[4]) : 0;
    out.push_back(r);
  }
  std::sort(out.begin(), out.end(), [](const CamCellRef &a, const CamCellRef &b) {
    if (a.timestamp != b.timestamp)
      return a.timestamp < b.timestamp;
    if (a.cam_id != b.cam_id)
      return a.cam_id < b.cam_id;
    if (a.mosaic_path != b.mosaic_path)
      return a.mosaic_path < b.mosaic_path;
    return a.cell_idx < b.cell_idx;
  });
  return out;
}

bool VioDataPlayer::open(const std::string &dataset_dir) {
  stop_loader_thread();

  dataset_dir_ = dataset_dir;
  mosaic_cache_.clear();
  mosaic_lru_.clear();
  mosaic_heights_.clear();
  mosaic_playback_order_.clear();
  imus_.clear();
  cam_cells_.clear();
  imu_i_ = cam_i_ = 0;

  const std::filesystem::path root(dataset_dir_);
  int gc, gr, tw, th;
  if (!parse_meta_file((root / "dataset_meta.txt").string(), gc, gr, tw, th)) {
    gc = 1;
    gr = 15;
    tw = 640;
    th = 480;
  }
  grid_cols_ = gc;
  grid_rows_ = gr;
  tile_w_ = tw;
  tile_h_ = th;

  const bool have_imu_f = std::filesystem::exists(root / "imu.csv");
  const bool have_cam_f = std::filesystem::exists(root / "camera_index.csv");
  if (!have_imu_f && !have_cam_f)
    return false;

  if (have_imu_f)
    imus_ = load_imu_csv((root / "imu.csv").string());
  if (have_cam_f)
    cam_cells_ = load_camera_csv((root / "camera_index.csv").string());

  for (const auto &c : cam_cells_) {
    auto &v = mosaic_heights_[c.mosaic_path];
    if (c.cell_idx + 1 > (int)v.size())
      v.resize(c.cell_idx + 1, 0);
    int h = (c.row_h > 0) ? c.row_h : tile_h_;
    if (h <= 0)
      throw std::runtime_error("VioDataPlayer: camera_index needs row_h when tile_height is 0, or valid tile_height in meta");
    v[c.cell_idx] = h;
  }

  build_mosaic_playback_order();
  loader_stop_ = false;
  loader_thread_ = std::thread([this] { loader_loop(); });

  return true;
}

void VioDataPlayer::reset() {
  imu_i_ = 0;
  cam_i_ = 0;
}

bool VioDataPlayer::has_next() const { return imu_i_ < imus_.size() || cam_i_ < cam_cells_.size(); }

cv::Mat VioDataPlayer::get_tile(const CamCellRef &ref) {
  if (ref.cell_idx < 0)
    throw std::out_of_range("VioDataPlayer: cell_idx out of range");
  const std::string &rel = ref.mosaic_path;
  if (rel.empty())
    throw std::runtime_error("VioDataPlayer: empty mosaic path");

  std::unique_lock<std::mutex> lk(load_mtx_);
  while (!mosaic_cache_.count(rel) && !load_failed_.count(rel)) {
    if (!loading_.count(rel)) {
      load_queue_.push_back(rel);
      loading_.insert(rel);
      load_cv_.notify_one();
    }
    loaded_cv_.wait(lk, [this, &rel] { return mosaic_cache_.count(rel) || load_failed_.count(rel); });
  }
  if (load_failed_.count(rel))
    throw std::runtime_error("VioDataPlayer: mosaic load failed " + rel);

  touch_mosaic_lru_unlocked(rel);
  const cv::Mat &full = mosaic_cache_.at(rel);

  cv::Rect roi;
  if (tile_h_ > 0) {
    const int r = ref.cell_idx / grid_cols_;
    const int c = ref.cell_idx % grid_cols_;
    roi = cv::Rect(c * tile_w_, r * tile_h_, tile_w_, tile_h_);
  } else {
    auto ith = mosaic_heights_.find(ref.mosaic_path);
    if (ith == mosaic_heights_.end())
      throw std::runtime_error("VioDataPlayer: mosaic height table missing");
    const std::vector<int> &hv = ith->second;
    if (ref.cell_idx >= (int)hv.size() || hv[ref.cell_idx] <= 0)
      throw std::runtime_error("VioDataPlayer: invalid row_h for cell");
    int y0 = 0;
    for (int i = 0; i < ref.cell_idx; ++i) {
      if (i >= (int)hv.size() || hv[i] <= 0)
        throw std::runtime_error("VioDataPlayer: incomplete mosaic height table");
      y0 += hv[i];
    }
    const int rh = hv[ref.cell_idx];
    const int rw = std::min(tile_w_, full.cols);
    roi = cv::Rect(0, y0, rw, rh);
  }

  if (roi.x + roi.width > full.cols || roi.y + roi.height > full.rows)
    throw std::runtime_error("VioDataPlayer: mosaic ROI out of bounds");
  return full(roi).clone();
}

bool VioDataPlayer::next_event(PlaybackEvent &out) {
  const bool have_imu = imu_i_ < imus_.size();
  const bool have_cam = cam_i_ < cam_cells_.size();
  if (!have_imu && !have_cam)
    return false;

  double next_imu_t = have_imu ? imus_[imu_i_].timestamp : std::numeric_limits<double>::infinity();
  double next_cam_t = have_cam ? cam_cells_[cam_i_].timestamp : std::numeric_limits<double>::infinity();

  if (next_imu_t <= next_cam_t) {
    out.type = PlaybackEvent::Type::Imu;
    out.imu = imus_[imu_i_++];
    return true;
  }

  out.type = PlaybackEvent::Type::Camera;
  out.camera = CameraData();
  const double t = cam_cells_[cam_i_].timestamp;
  constexpr double teps = 1e-9;
  while (cam_i_ < cam_cells_.size() && std::fabs(cam_cells_[cam_i_].timestamp - t) <= teps) {
    const CamCellRef &ref = cam_cells_[cam_i_];
    out.camera.timestamp = ref.timestamp;
    out.camera.sensor_ids.push_back(ref.cam_id);
    out.camera.images.push_back(get_tile(ref));
    out.camera.masks.push_back(cv::Mat());
    cam_i_++;
  }
  return true;
}

} // namespace ov_core
