/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_CORE_TIMING_H
#define OV_CORE_TIMING_H

#include <chrono>

namespace ov_core {

/**
 * @brief 高精度计时工具，用于替代 Boost posix_time
 *
 * 使用 C++11 std::chrono 实现，避免 Boost 依赖。
 */
using rtime_t = std::chrono::high_resolution_clock::time_point;

/// 获取当前时间点（微秒级精度）
inline rtime_t rtime_now() { return std::chrono::high_resolution_clock::now(); }

/// 计算两时间点之间的秒数
template <typename T1, typename T2>
inline double rtime_sec(T1 t1, T2 t2) {
  return std::chrono::duration<double>(t2 - t1).count();
}

/// 计算两时间点之间的毫秒数
template <typename T1, typename T2>
inline double rtime_ms(T1 t1, T2 t2) {
  return std::chrono::duration<double, std::milli>(t2 - t1).count();
}

} // namespace ov_core

#endif /* OV_CORE_TIMING_H */
