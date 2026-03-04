// Copyright 2025 the project authors.

#include "data_preprocessing/cpi_sequence.h"

#include <algorithm>
#include <cmath>

namespace data_preprocessing {

CpiSequence::CpiSequence(double sigma_w, double sigma_wb, double sigma_a,
                         double sigma_ab, bool imu_avg)
    : imu_avg_(imu_avg),
      sigma_w_(sigma_w),
      sigma_wb_(sigma_wb),
      sigma_a_(sigma_a),
      sigma_ab_(sigma_ab) {}

void CpiSequence::SetBias(const Vector3& b_w, const Vector3& b_a) {
  b_w_ = b_w;
  b_a_ = b_a;
}

void CpiSequence::AddImu(double t, const Vector3& w, const Vector3& a) {
  imu_.push_back({t, w, a});
}

bool CpiSequence::GetPreintegration(double t_start, double t_end,
                                    CpiResult* result) const {
  if (!result || t_start > t_end) return false;

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

}  // namespace data_preprocessing
