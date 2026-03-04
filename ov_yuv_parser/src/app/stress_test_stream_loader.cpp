/*
 * 流式数据加载器压力测试
 * 生成合成数据，多轮运行验证内存稳定性与正确性
 */

#include "yuv_parser.h"
#include "stream_data_loader.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// Linux: 读取当前进程 RSS (KB)
static long getRssKb() {
#ifdef __linux__
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) {
            long kb = 0;
            sscanf(line.c_str() + 6, "%ld", &kb);
            return kb;
        }
    }
#endif
    return -1;
}

// 生成合成测试数据
static bool generateSyntheticData(const std::string& base_dir, int num_frames, int width, int height) {
    fs::create_directories(base_dir);
    std::string dump_yuv_dir = base_dir + "/dump_yuv";
    fs::create_directories(dump_yuv_dir);

    const int frame_size = width * height;
    const long long base_ts = 1000000;
    const int imu_per_frame = 5;

    // FrameInfo.txt
    std::ofstream info_file(dump_yuv_dir + "/FrameInfo.txt");
    if (!info_file) {
        std::cerr << "Failed to create FrameInfo.txt" << std::endl;
        return false;
    }
    info_file << "filename frame_count vi_pts exp_time width height\n";
    for (int i = 0; i < num_frames; ++i) {
        info_file << "cam0.yuv " << i << " " << (base_ts + i * 33333) << " 1000 " << width << " " << height << "\n";
    }
    info_file.close();

    // cam0.yuv
    std::ofstream yuv_file(dump_yuv_dir + "/cam0.yuv", std::ios::binary);
    if (!yuv_file) {
        std::cerr << "Failed to create YUV file" << std::endl;
        return false;
    }
    std::vector<unsigned char> frame(frame_size);
    std::mt19937 rng(42);
    for (int i = 0; i < num_frames; ++i) {
        for (int j = 0; j < frame_size; ++j) {
            frame[j] = static_cast<unsigned char>(rng() % 256);
        }
        yuv_file.write(reinterpret_cast<const char*>(frame.data()), frame_size);
    }
    yuv_file.close();

    // imu.txt
    std::ofstream imu_file(base_dir + "/imu.txt");
    if (!imu_file) {
        std::cerr << "Failed to create imu.txt" << std::endl;
        return false;
    }
    std::uniform_real_distribution<double> acc_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> gyro_dist(-0.1, 0.1);
    for (int i = 0; i < num_frames * imu_per_frame; ++i) {
        long long ts = base_ts + i * 6666;
        imu_file << "index:" << i << ", curPts:" << ts << ", timeStamp:" << ts
                 << ", accX:" << acc_dist(rng) << ", accY:" << acc_dist(rng)
                 << ", accZ:" << acc_dist(rng) << ", gyroX:" << gyro_dist(rng)
                 << ", gyroY:" << gyro_dist(rng) << ", gyroZ:" << gyro_dist(rng)
                 << ", accPts:" << ts << ", gyroTemp:25, accTemp:25\n";
    }
    imu_file.close();

    return true;
}

int main(int argc, char* argv[]) {
    int num_frames = 500;
    int num_iterations = 20;
    size_t cache_size = 4;
    int width = 640;
    int height = 480;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--frames" && i + 1 < argc) {
            num_frames = std::max(1, std::atoi(argv[++i]));
        } else if (arg == "--iterations" && i + 1 < argc) {
            num_iterations = std::max(1, std::atoi(argv[++i]));
        } else if (arg == "--cache-size" && i + 1 < argc) {
            cache_size = std::max(size_t(1), static_cast<size_t>(std::atoi(argv[++i])));
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "流式数据加载器压力测试\n"
                         "用法: " << argv[0] << " [选项]\n"
                         "  --frames N       合成帧数 (默认 500)\n"
                         "  --iterations M   运行轮数 (默认 20)\n"
                         "  --cache-size C   帧缓存大小 (默认 4)\n";
            return 0;
        }
    }

    std::string tmp_base = "/tmp/stress_test_stream_loader_" + std::to_string(getpid());
    std::string dump_yuv_dir = tmp_base + "/dump_yuv";
    std::string imu_file = tmp_base + "/imu.txt";

    std::cout << "=== 流式加载器压力测试 ===" << std::endl;
    std::cout << "帧数: " << num_frames << ", 轮数: " << num_iterations
              << ", 缓存: " << cache_size << " 帧" << std::endl;
    std::cout << "数据目录: " << tmp_base << std::endl;

    if (!generateSyntheticData(tmp_base, num_frames, width, height)) {
        std::cerr << "生成合成数据失败" << std::endl;
        return 1;
    }
    std::cout << "合成数据生成完成" << std::endl;

    YUVParser parser(width, height);
    long rss_before = getRssKb();
    if (rss_before > 0) {
        std::cout << "初始 RSS: " << (rss_before / 1024.0) << " MB" << std::endl;
    }

    std::vector<double> iter_times_ms;
    int total_frame_count = 0;
    int total_imu_count = 0;
    bool all_ok = true;

    for (int iter = 0; iter < num_iterations; ++iter) {
        StreamDataLoader loader(&parser, dump_yuv_dir, imu_file, cache_size);
        if (loader.empty()) {
            std::cerr << "迭代 " << iter << ": 数据为空" << std::endl;
            all_ok = false;
            continue;
        }

        auto t0 = std::chrono::high_resolution_clock::now();
        loader.start();

        int frame_count = 0;
        int imu_count = 0;
        TimestampedData data;
        while (loader.getNext(data)) {
            if (data.type == TimestampedData::FRAME) {
                frame_count++;
                if (data.frame_data.frame_data.size() != static_cast<size_t>(width * height)) {
                    std::cerr << "迭代 " << iter << ": 帧大小异常" << std::endl;
                    all_ok = false;
                }
            } else {
                imu_count++;
            }
        }
        loader.stop();

        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        iter_times_ms.push_back(ms);
        total_frame_count += frame_count;
        total_imu_count += imu_count;

        int expected_frames = num_frames;
        int expected_imu = num_frames * 5;  // imu_per_frame
        if (frame_count != expected_frames || imu_count != expected_imu) {
            std::cerr << "迭代 " << iter << ": 数量异常 frame=" << frame_count << "/" << expected_frames
                      << " imu=" << imu_count << "/" << expected_imu << std::endl;
            all_ok = false;
        }

        if ((iter + 1) % 5 == 0 || iter == 0) {
            std::cout << "  迭代 " << (iter + 1) << "/" << num_iterations
                      << ": " << frame_count << " 帧, " << imu_count << " IMU, "
                      << std::fixed << std::setprecision(1) << ms << " ms" << std::endl;
        }
    }

    long rss_after = getRssKb();
    if (rss_after > 0) {
        std::cout << "结束 RSS: " << (rss_after / 1024.0) << " MB" << std::endl;
        long delta = rss_after - rss_before;
        if (rss_before > 0 && delta > 1024) {
            std::cout << "警告: 内存增长 " << (delta / 1024.0) << " MB (可能存在泄漏)" << std::endl;
            all_ok = false;
        }
    }

    if (!iter_times_ms.empty()) {
        double sum = 0;
        for (double t : iter_times_ms) sum += t;
        double mean = sum / iter_times_ms.size();
        double var = 0;
        for (double t : iter_times_ms) var += (t - mean) * (t - mean);
        double std_dev = std::sqrt(var / (iter_times_ms.size() - 1));
        std::cout << "耗时: " << std::fixed << std::setprecision(1) << mean
                  << " ± " << std_dev << " ms/轮" << std::endl;
    }

    std::cout << "总计: " << total_frame_count << " 帧, " << total_imu_count << " IMU" << std::endl;

    // 清理
    try {
        fs::remove_all(tmp_base);
    } catch (...) {}

    std::cout << (all_ok ? "=== 压力测试通过 ===" : "=== 压力测试失败 ===") << std::endl;
    return all_ok ? 0 : 1;
}
