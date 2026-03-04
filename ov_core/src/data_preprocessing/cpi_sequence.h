// Copyright 2025 the project authors.
// 在 CpiModel 基础上提供「序列 + 任意两时间点预积分」：先按时间顺序填入 IMU，
// 再对任意 [t_start, t_end] 段计算预积分结果。与 cpi_model.h 同命名空间、同量纲约定。

#ifndef MYVIO_DATA_PREPROCESSING_CPI_SEQUENCE_H_
#define MYVIO_DATA_PREPROCESSING_CPI_SEQUENCE_H_

#include "data_preprocessing/cpi_model.h"

#include <vector>

namespace data_preprocessing {

/// 一段预积分的只读结果，与 CpiModel 的对外量一致，便于比对与下游使用。
struct CpiResult {
  double DT{0};
  CpiModel::Vector3 alpha_tau{CpiModel::Vector3::Zero()};
  CpiModel::Vector3 beta_tau{CpiModel::Vector3::Zero()};
  CpiModel::Vector4 q_k2tau;
  CpiModel::Matrix3 R_k2tau{CpiModel::Matrix3::Identity()};
  CpiModel::Matrix3 J_q{CpiModel::Matrix3::Zero()};
  CpiModel::Matrix3 J_a{CpiModel::Matrix3::Zero()};
  CpiModel::Matrix3 J_b{CpiModel::Matrix3::Zero()};
  CpiModel::Matrix3 H_a{CpiModel::Matrix3::Zero()};
  CpiModel::Matrix3 H_b{CpiModel::Matrix3::Zero()};
  CpiModel::Matrix15 P_meas{CpiModel::Matrix15::Zero()};
};

/**
 * IMU 序列 + 任意两时间点预积分。
 *
 * 预积分在 AddImu 时计算（非 GetPreintegration 时），符合流式添加场景。
 *
 * 使用方式：(1) 构造时传入与 CpiModel 相同的噪声/是否平均参数；
 * (2) SetBias(b_w, b_a) 设定本序列使用的 bias（在添加/查询前调用）；
 * (3) 按时间升序 AddImu(t, w, a) 填入测量——每添加一对即做预积分；
 * (4) GetPreintegration(t_start, t_end, &out) 组合 [t_start, t_end] 内已计算的
 *     delta 结果，无额外积分计算。
 *
 * 量纲与 cpi_model.h 一致：t 为秒，w 为 rad/s，a 为 m/s^2。
 */
class CpiSequence {
 public:
  using Vector3 = CpiModel::Vector3;
  using Vector4 = CpiModel::Vector4;
  using Matrix3 = CpiModel::Matrix3;
  using Matrix15 = CpiModel::Matrix15;

  CpiSequence(double sigma_w, double sigma_wb, double sigma_a, double sigma_ab,
              bool imu_avg = false);

  void SetBias(const Vector3& b_w, const Vector3& b_a);

  /// 按时间顺序追加一条 IMU。若有前一点则立即做该区间的预积分并缓存。
  void AddImu(double t, const Vector3& w, const Vector3& a);

  /**
   * 获取 [t_start, t_end] 的预积分，由 AddImu 时缓存的 delta 组合而成，无额外积分。
   * @return 成功返回 true；区间内不足 2 个点或 t_start > t_end 返回 false，result 不改动。
   */
  bool GetPreintegration(double t_start, double t_end, CpiResult* result) const;

  /// 直接返回已缓存的预积分（[t_first_, t_last_] 全段），O(1)。无缓存时返回 false。
  bool GetCachedResult(CpiResult* result) const;

  /// 当前已添加的 IMU 数量。
  size_t Size() const { return imu_.size(); }

  /// 已缓存预积分的时间范围 [t_first, t_last]。用于确保 GetPreintegration 命中缓存。
  bool GetCachedBounds(double* t_first, double* t_last) const {
    if (!has_cached_ || !t_first || !t_last) return false;
    *t_first = t_first_;
    *t_last = t_last_;
    return true;
  }

  bool imu_avg() const { return imu_avg_; }

 private:
  struct ImuPoint {
    double t{0};
    Vector3 w{Vector3::Zero()};
    Vector3 a{Vector3::Zero()};
  };

  bool imu_avg_{false};
  Vector3 b_w_{Vector3::Zero()};
  Vector3 b_a_{Vector3::Zero()};
  std::vector<ImuPoint> imu_;
  CpiModel running_;           // AddImu 时累积的预积分
  CpiResult cached_full_;       // [t_first, t_last] 的缓存结果
  double t_first_{0};           // 已添加的第一个时间
  double t_last_{0};            // 已添加的最后一个时间
  bool has_cached_{false};
  double sigma_w_{0}, sigma_wb_{0}, sigma_a_{0}, sigma_ab_{0};
};

}  // namespace data_preprocessing

#endif  // MYVIO_DATA_PREPROCESSING_CPI_SEQUENCE_H_
