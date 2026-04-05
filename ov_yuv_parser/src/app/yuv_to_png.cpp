#include <iostream>
#include <string>
#include <vector>

#include "yuv_parser.h"

void printUsage(const char* program_name) {
  std::cout << "Usage: " << program_name
            << " <dump_yuv_dir> [width] [height] [options]\n"
            << std::endl;
  std::cout << "Arguments:" << std::endl;
  std::cout << "  dump_yuv_dir  : Path to directory containing YUV files and "
               "FrameInfo.txt"
            << std::endl;
  std::cout << "  width         : Image width (default: 640)" << std::endl;
  std::cout << "  height        : Image height (default: 480)" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --show, -s          边导出边 imshow 预览（按 q / Esc 提前结束）"
            << std::endl;
  std::cout << "  --show-delay N, -d N  每帧 waitKey(N)；N=0 表示逐帧按键步进"
            << std::endl;
  std::cout << std::endl;
  std::cout << "Example:" << std::endl;
  std::cout << "  " << program_name
            << " data/yuv_data/20260115_1034/indoor_rich_texture/"
               "horizontal_move_indoor_rich_texture/dump_yuv"
            << std::endl;
  std::cout << "  " << program_name
            << " .../dump_yuv 640 480 --show --show-delay 1" << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Error: Missing required argument: dump_yuv_dir" << std::endl;
    std::cerr << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  std::vector<std::string> pos;
  bool show = false;
  int delay_ms = 1;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--show" || a == "-s") {
      show = true;
      continue;
    }
    if (a == "--show-delay" || a == "-d") {
      if (i + 1 >= argc) {
        std::cerr << "Error: " << a << " 需要整数参数" << std::endl;
        return 1;
      }
      delay_ms = std::stoi(argv[++i]);
      continue;
    }
    if (!a.empty() && a[0] == '-') {
      std::cerr << "Error: 未知选项: " << a << std::endl;
      printUsage(argv[0]);
      return 1;
    }
    pos.push_back(a);
  }

  if (pos.empty()) {
    std::cerr << "Error: Missing dump_yuv_dir" << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  std::string dump_yuv_dir = pos[0];
  int width = 640;
  int height = 480;

  if (pos.size() >= 2) {
    width = std::stoi(pos[1]);
    if (width <= 0) {
      std::cerr << "Error: Invalid width: " << pos[1] << std::endl;
      return 1;
    }
  }

  if (pos.size() >= 3) {
    height = std::stoi(pos[2]);
    if (height <= 0) {
      std::cerr << "Error: Invalid height: " << pos[2] << std::endl;
      return 1;
    }
  }

#if !OPENVINS_HAVE_OPENCV_HIGHGUI
  if (show) {
    std::cerr << "Warning: 当前目标未启用 OpenCV highgui，忽略 --show\n";
    show = false;
  }
#endif

  std::cout << "=== YUV to PNG Converter ===" << std::endl;
  std::cout << "Input directory: " << dump_yuv_dir << std::endl;
  std::cout << "Image size: " << width << "x" << height << std::endl;
  if (show) {
    std::cout << "Preview: imshow ON, waitKey delay = " << delay_ms
              << " ms (0 = step)\n";
  }
  std::cout << std::endl;

  YUVParser yuv_parser(width, height);

  int exported_count = yuv_parser.exportFramesToPNG(dump_yuv_dir, show, delay_ms);

  if (exported_count < 0) {
    std::cerr << "Error: Failed to export frames" << std::endl;
    return 1;
  }

  std::cout << std::endl;
  std::cout << "=== Conversion Complete ===" << std::endl;
  std::cout << "Successfully exported " << exported_count << " frames"
            << std::endl;

  return 0;
}
