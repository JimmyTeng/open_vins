/**
 * @file fs_compat.h
 * @brief filesystem 兼容层：GCC 7 用 experimental/filesystem，GCC 8+ 用标准 <filesystem>
 *
 * Ubuntu 18.04 (GCC 7) 无 <filesystem>，需用 <experimental/filesystem> 并链接 -lstdc++fs。
 */
#pragma once

#if __has_include(<filesystem>)
#include <filesystem>
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace std {
namespace filesystem = experimental::filesystem;
}
#else
#error "No filesystem support. Need GCC 7+ with experimental/filesystem or GCC 8+ with <filesystem>."
#endif
