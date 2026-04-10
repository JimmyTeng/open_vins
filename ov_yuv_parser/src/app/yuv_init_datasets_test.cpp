/**
 * 扫描 data/yuv_data/init（或命令行指定根目录）下所有 dump_yuv/FrameInfo.txt，
 * 校验：半曝光时间戳、按时间排序后的单调性、相邻间隔统计；默认还检查各分片 YUV 尺寸。
 * 不整文件解码 YUV。可选 --skip-yuv：YUV 缺/截断时仍可测完全部 FrameInfo 时间轴。
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "utils/fs_compat.h"
namespace stdfs = std::filesystem;

#include "yuv_parser.h"

static std::vector<stdfs::path> find_dump_yuv_dirs(const stdfs::path& init_root) {
  std::vector<stdfs::path> out;
  if (!stdfs::exists(init_root)) {
    return out;
  }
  std::error_code ec;
  for (auto it = stdfs::recursive_directory_iterator(init_root, ec);
       it != stdfs::recursive_directory_iterator(); ++it) {
    if (ec) break;
    if (!it->is_regular_file()) continue;
    const stdfs::path& p = it->path();
    if (p.filename() == "FrameInfo.txt" &&
        p.parent_path().filename() == "dump_yuv") {
      out.push_back(stdfs::weakly_canonical(p.parent_path(), ec));
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

static std::string rel_display(const stdfs::path& root, const stdfs::path& abs_path) {
  std::error_code ec;
  stdfs::path r = stdfs::weakly_canonical(root, ec);
  stdfs::path a = stdfs::weakly_canonical(abs_path, ec);
  if (!ec) {
    auto s = stdfs::relative(a, r, ec);
    if (!ec) return s.string();
  }
  return abs_path.string();
}

struct DatasetReport {
  std::string name;
  bool ok = true;
  size_t n_rows = 0;
  long long min_dt = 0;
  long long max_dt = 0;
  double mean_dt = 0.0;
  double std_dt = 0.0;
  size_t non_mono = 0;
  size_t duplicates = 0;
  std::string error;
};

static bool check_yuv_sizes(const stdfs::path& dump_dir,
                            const std::vector<FrameInfo>& infos,
                            std::string& err) {
  std::map<std::string, std::vector<const FrameInfo*>> by_file;
  for (const auto& fi : infos) {
    by_file[fi.filename].push_back(&fi);
  }
  for (const auto& kv : by_file) {
    const std::string& fname = kv.first;
    const auto& vec = kv.second;
    int min_fc = vec[0]->frame_count;
    int max_fc = vec[0]->frame_count;
    int w = vec[0]->width;
    int h = vec[0]->height;
    for (const FrameInfo* p : vec) {
      min_fc = std::min(min_fc, p->frame_count);
      max_fc = std::max(max_fc, p->frame_count);
      if (p->width != w || p->height != h) {
        err = "FrameInfo 中 Width/Height 不一致: " + fname;
        return false;
      }
    }
    const int n_in_span = max_fc - min_fc + 1;
    if (n_in_span <= 0) {
      err = "无效的 FramCnt 范围: " + fname;
      return false;
    }
    const auto yuv_path = dump_dir / fname;
    std::error_code ec;
    if (!stdfs::exists(yuv_path)) {
      err = "缺少 YUV 文件: " + yuv_path.string();
      return false;
    }
    const std::uintmax_t expected =
        static_cast<std::uintmax_t>(n_in_span) *
        static_cast<std::uintmax_t>(w) * static_cast<std::uintmax_t>(h);
    const std::uintmax_t actual = stdfs::file_size(yuv_path, ec);
    if (ec) {
      err = "无法读取文件大小: " + yuv_path.string();
      return false;
    }
    if (actual < expected) {
      err = "YUV 过小 " + yuv_path.string() + " 实际=" + std::to_string(actual) +
            " 期望≥" + std::to_string(expected) + " (帧数=" +
            std::to_string(n_in_span) + ")";
      return false;
    }
  }
  return true;
}

static DatasetReport validate_one_dump(const stdfs::path& init_root,
                                       const stdfs::path& dump_dir,
                                       bool verify_yuv_files) {
  DatasetReport rep;
  rep.name = rel_display(init_root, dump_dir);

  YUVParser parser(1, 1);
  const std::string info_path = (dump_dir / "FrameInfo.txt").string();
  std::vector<FrameInfo> infos = parser.parseFrameInfo(info_path);
  if (infos.empty()) {
    rep.ok = false;
    rep.error = "FrameInfo 为空或无法解析";
    return rep;
  }
  rep.n_rows = infos.size();

  if (verify_yuv_files) {
    std::string yuv_err;
    if (!check_yuv_sizes(dump_dir, infos, yuv_err)) {
      rep.ok = false;
      rep.error = yuv_err;
      return rep;
    }
  }

  std::vector<long long> ts;
  ts.reserve(infos.size());
  for (const auto& fi : infos) {
    ts.push_back(YUVParser::compensatedImageTimestamp(fi.vi_pts, fi.exp_time));
  }
  std::sort(ts.begin(), ts.end());

  for (size_t i = 1; i < ts.size(); ++i) {
    if (ts[i] < ts[i - 1]) rep.non_mono++;
    if (ts[i] == ts[i - 1]) rep.duplicates++;
  }
  if (rep.non_mono > 0) {
    rep.ok = false;
    rep.error = "修正后时间戳未严格递增 (逆序对数=" +
                std::to_string(rep.non_mono) + ")";
    return rep;
  }

  if (ts.size() < 2) {
    rep.min_dt = rep.max_dt = 0;
    rep.mean_dt = 0.0;
    rep.std_dt = 0.0;
    return rep;
  }

  std::vector<long long> dts;
  dts.reserve(ts.size() - 1);
  for (size_t i = 1; i < ts.size(); ++i) {
    dts.push_back(ts[i] - ts[i - 1]);
  }
  rep.min_dt = *std::min_element(dts.begin(), dts.end());
  rep.max_dt = *std::max_element(dts.begin(), dts.end());
  long double sum = 0;
  for (long long d : dts) sum += static_cast<long double>(d);
  rep.mean_dt = static_cast<double>(sum / static_cast<long double>(dts.size()));
  long double var = 0;
  for (long long d : dts) {
    long double x = static_cast<long double>(d) - rep.mean_dt;
    var += x * x;
  }
  rep.std_dt = static_cast<double>(
      std::sqrt(static_cast<double>(var / static_cast<long double>(dts.size()))));

  return rep;
}

int main(int argc, char** argv) {
  bool verify_yuv_files = true;
  const char* root_arg = nullptr;
  for (int i = 1; i < argc; ++i) {
    const std::string a(argv[i]);
    if (a == "--skip-yuv") {
      verify_yuv_files = false;
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "未知选项: " << a << "\n";
      std::cerr << "用法: " << (argc > 0 ? argv[0] : "yuv_init_datasets_test")
                << " [--skip-yuv] [init根目录]\n"
                << "  --skip-yuv  仅校验 FrameInfo 时间戳/间隔，不检查 YUV 文件是否存在或大小\n"
                << "  init 默认: <当前工作目录>/data/yuv_data/init\n";
      return 2;
    } else {
      root_arg = argv[i];
    }
  }

  stdfs::path init_root = root_arg ? stdfs::path(root_arg)
                                   : (stdfs::current_path() / "data/yuv_data/init");
  std::error_code ec;
  init_root = stdfs::weakly_canonical(init_root, ec);
  if (!stdfs::exists(init_root)) {
    std::cerr << "目录不存在: " << init_root.string() << "\n";
    std::cerr << "用法: " << (argc > 0 ? argv[0] : "yuv_init_datasets_test")
              << " [--skip-yuv] [init根目录，默认 <cwd>/data/yuv_data/init]\n";
    return 2;
  }

  std::vector<stdfs::path> dumps = find_dump_yuv_dirs(init_root);
  if (dumps.empty()) {
    std::cerr << "未找到任何 dump_yuv/FrameInfo.txt 于: " << init_root.string()
              << std::endl;
    return 3;
  }

  std::cout << "根目录: " << init_root.string() << "\n共 " << dumps.size()
            << " 个数据集 (dump_yuv)";
  if (!verify_yuv_files) {
    std::cout << "  [已 --skip-yuv，不检查 YUV 文件]";
  }
  std::cout << "\n\n";

  int failed = 0;
  std::cout << std::fixed << std::setprecision(2);
  for (const auto& d : dumps) {
    DatasetReport r = validate_one_dump(init_root, d, verify_yuv_files);
    if (!r.ok) {
      ++failed;
      std::cout << "[FAIL] " << r.name << "\n  " << r.error << "\n\n";
    } else {
      std::cout << "[ OK ] " << r.name << "  行数=" << r.n_rows
                << "  Δt[min,mean±std,max]=[" << r.min_dt << ", " << r.mean_dt
                << "±" << r.std_dt << ", " << r.max_dt << "]";
      if (r.duplicates > 0) {
        std::cout << "  重复时间戳相邻对=" << r.duplicates;
      }
      std::cout << "\n\n";
    }
  }

  std::cout << "----\n总计 " << dumps.size() << "，失败 " << failed << "\n";
  return failed > 0 ? 1 : 0;
}
