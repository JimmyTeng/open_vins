/**
 * 校验：FrameInfo 经 compensatedImageTimestamp 后，相邻帧时间间隔与原始 vi_pts 间隔一致
 * （曝光时间恒定时，减半曝光为整体平移）；并校验每帧 timestamp = vi_pts - exp/2。
 */

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "yuv_parser.h"

namespace fs = std::filesystem;

// parseAllFrames 内对单文件调用 parseYUVFile(..., 100)，YUV 至少写满 100 帧可避免告警
static constexpr int kYuvPrefetchFrames = 100;

static bool write_dump(const fs::path& dump_yuv, int w, int h, int n_frames,
                       long long base_pts, long long period_us, int exp_us) {
  fs::create_directories(dump_yuv);
  const int frame_size = w * h;
  std::ofstream info(dump_yuv / "FrameInfo.txt");
  if (!info) return false;
  info << "filename frame_count vi_pts exp_time width height\n";
  for (int i = 0; i < n_frames; ++i) {
    long long pts = base_pts + i * period_us;
    info << "cam0.yuv " << i << " " << pts << " " << exp_us << " " << w << " "
         << h << "\n";
  }
  info.close();

  std::ofstream yuv(dump_yuv / "cam0.yuv", std::ios::binary);
  if (!yuv) return false;
  std::vector<unsigned char> buf(static_cast<size_t>(frame_size), 0);
  const int yuv_frames = std::max(n_frames, kYuvPrefetchFrames);
  for (int i = 0; i < yuv_frames; ++i) {
    yuv.write(reinterpret_cast<const char*>(buf.data()), frame_size);
  }
  return true;
}

static int fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

int main() {
  if (YUVParser::compensatedImageTimestamp(1'000'000, 1000) != 999'500)
    return fail("compensatedImageTimestamp(1e6, 1000)");
  if (YUVParser::compensatedImageTimestamp(100, 0) != 100)
    return fail("compensatedImageTimestamp(100, 0)");
  if (YUVParser::compensatedImageTimestamp(101, 3) != 100)  // 3/2 = 1
    return fail("compensatedImageTimestamp(101, 3)");

  const fs::path tmp =
      fs::temp_directory_path() /
      ("ov_yuv_ts_interval_" + std::to_string(std::rand()));
  const fs::path dump = tmp / "dump_yuv";
  constexpr int kW = 8;
  constexpr int kH = 8;
  constexpr int kN = 5;
  constexpr long long kBase = 1'000'000;
  constexpr long long kPeriod = 33'333;
  constexpr int kExp = 2'000;

  if (!write_dump(dump, kW, kH, kN, kBase, kPeriod, kExp))
    return fail("write_dump");

  YUVParser parser(kW, kH);
  std::vector<FrameData> frames = parser.parseAllFrames(dump.string());
  if (static_cast<int>(frames.size()) != kN)
    return fail("parseAllFrames count");

  for (int i = 0; i < kN; ++i) {
    const FrameInfo& fi = frames[static_cast<size_t>(i)].info;
    long long expect_ts =
        YUVParser::compensatedImageTimestamp(fi.vi_pts, fi.exp_time);
    if (frames[static_cast<size_t>(i)].timestamp != expect_ts)
      return fail("timestamp != vi_pts - exp/2");
    if (fi.vi_pts != kBase + i * kPeriod)
      return fail("vi_pts mismatch");
  }

  for (int i = 1; i < kN; ++i) {
    long long dt = frames[static_cast<size_t>(i)].timestamp -
                   frames[static_cast<size_t>(i - 1)].timestamp;
    if (dt != kPeriod)
      return fail("adjacent interval != period (constant exposure case)");
  }

  std::error_code ec;
  fs::remove_all(tmp, ec);

  std::cout << "OK: 半曝光修正后共 " << kN << " 帧，相邻间隔均为 " << kPeriod
            << "（与 vi_pts 间隔一致）\n";
  return 0;
}
