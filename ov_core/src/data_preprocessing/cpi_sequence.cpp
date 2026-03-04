// Copyright 2025 the project authors.

#include "data_preprocessing/cpi_sequence.h"
#include "utils/quat_ops.h"

#include <algorithm>
#include <cmath>

namespace data_preprocessing {

CpiSequence::CpiSequence(double sigma_w, double sigma_wb, double sigma_a,
                         double sigma_ab, bool imu_avg)
    : imu_avg_(imu_avg),
      running_(sigma_w, sigma_wb, sigma_a, sigma_ab, imu_avg),
      sigma_w_(sigma_w),
      sigma_wb_(sigma_wb),
      sigma_a_(sigma_a),
      sigma_ab_(sigma_ab) {}

void CpiSequence::SetBias(const Vector3& b_w, const Vector3& b_a) {
  b_w_ = b_w;
  b_a_ = b_a;
  running_.InitBias(b_w_, b_a_);
}

void CpiSequence::AddImu(double t, const Vector3& w, const Vector3& a) {
  const size_t n = imu_.size();
  imu_.push_back({t, w, a});

  if (n >= 1u) {
    const ImuPoint& p0 = imu_[n - 1];
    const ImuPoint& p1 = imu_[n];
    if (std::abs(p1.t - p0.t) >= 1e-15) {
      running_.FeedImu(p0.t, p1.t, p0.w, p0.a, p1.w, p1.a);
      if (n == 1u) {
        t_first_ = p0.t;
      }
      t_last_ = p1.t;
      has_cached_ = true;
      cached_full_.DT = running_.DT;
      cached_full_.alpha_tau = running_.alpha_tau;
      cached_full_.beta_tau = running_.beta_tau;
      cached_full_.q_k2tau = running_.q_k2tau;
      cached_full_.R_k2tau = running_.R_k2tau;
      cached_full_.J_q = running_.J_q;
      cached_full_.J_a = running_.J_a;
      cached_full_.J_b = running_.J_b;
      cached_full_.H_a = running_.H_a;
      cached_full_.H_b = running_.H_b;
      cached_full_.P_meas = running_.P_meas();
    }
  } else {
    t_first_ = t;
    t_last_ = t;
  }
}

bool CpiSequence::GetPreintegration(double t_start, double t_end,
                                    CpiResult* result) const {
  if (!result || t_start > t_end) return false;

  constexpr double kTol = 1e-6;
  const bool hit = has_cached_ && std::abs(t_start - t_first_) <= kTol &&
                   std::abs(t_end - t_last_) <= kTol;
  if (hit) {
    *result = cached_full_;
    return true;
  }
  std::vector<ImuPoint> seg;
  for (const auto& p : imu_) {
    if (p.t >= t_start - 1e-12 && p.t <= t_end + 1e-12) {
      seg.push_back(p);
    }
  }
  if (seg.size() < 2u) return false;

  std::sort(seg.begin(), seg.end(),
            [](const ImuPoint& a, const ImuPoint& b) { return a.t < b.t; });

  CpiModel m(sigma_w_, sigma_wb_, sigma_a_, sigma_ab_, imu_avg_);
  m.InitBias(b_w_, b_a_);

  for (size_t i = 0; i + 1 < seg.size(); ++i) {
    const double t0 = seg[i].t;
    const double t1 = seg[i + 1].t;
    if (std::abs(t1 - t0) < 1e-15) continue;
    if (imu_avg_) {
      m.FeedImu(t0, t1, seg[i].w, seg[i].a, seg[i + 1].w, seg[i + 1].a);
    } else {
      m.FeedImu(t0, t1, seg[i].w, seg[i].a, Vector3::Zero(), Vector3::Zero());
    }
  }

  result->DT = m.DT;
  result->alpha_tau = m.alpha_tau;
  result->beta_tau = m.beta_tau;
  result->q_k2tau = m.q_k2tau;
  result->R_k2tau = m.R_k2tau;
  result->J_q = m.J_q;
  result->J_a = m.J_a;
  result->J_b = m.J_b;
  result->H_a = m.H_a;
  result->H_b = m.H_b;
  result->P_meas = m.P_meas();
  return true;
}

bool CpiSequence::GetCachedResult(CpiResult* result) const {
  if (!result || !has_cached_) return false;
  *result = cached_full_;
  return true;
}

}  // namespace data_preprocessing
