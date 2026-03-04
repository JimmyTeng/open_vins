/*
 * 测试 CpiModel (data_preprocessing) 与 CpiV1 (ov_core) 的一致性。
 * 用相同的 IMU 序列和参数分别积分，比较 DT、alpha_tau、beta_tau、
 * R_k2tau、雅可比、协方差的差异。
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <Eigen/Dense>

#include "cpi/CpiV1.h"
#include "data_preprocessing/cpi_model.h"
#include "data_preprocessing/cpi_sequence.h"

using namespace ov_core;
using namespace data_preprocessing;

namespace {

constexpr double kTol = 1e-10;  // 相对/绝对容差

bool is_close(double a, double b, double rel_tol = kTol, double abs_tol = kTol) {
  return std::abs(a - b) <= abs_tol || std::abs(a - b) <= rel_tol * std::max(std::abs(a), std::abs(b));
}

bool matrix_close(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                  double rel_tol = kTol, double abs_tol = kTol) {
  if (A.rows() != B.rows() || A.cols() != B.cols()) return false;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      if (!is_close(A(i, j), B(i, j), rel_tol, abs_tol)) return false;
    }
  }
  return true;
}

Eigen::Matrix<double, 3, 1> vec3(double x, double y, double z) {
  Eigen::Matrix<double, 3, 1> v;
  v << x, y, z;
  return v;
}

}  // namespace

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  // 参数（与 CpiV1 典型用法一致）
  const double sigma_w = 1e-3;
  const double sigma_wb = 1e-4;
  const double sigma_a = 1e-2;
  const double sigma_ab = 1e-3;

  // Bias 线性化点
  const Eigen::Vector3d b_w = vec3(0.01, -0.02, 0.005);
  const Eigen::Vector3d b_a = vec3(0.1, -0.05, 0.0);

  int failed = 0;

  for (bool imu_avg : {false, true}) {
    printf("\n=== imu_avg=%s ===\n", imu_avg ? "true" : "false");

    // 构造 CpiV1 与 CpiModel
    auto cpi_v1 = std::make_shared<CpiV1>(sigma_w, sigma_wb, sigma_a, sigma_ab, imu_avg);
    CpiModel cpi_model(sigma_w, sigma_wb, sigma_a, sigma_ab, imu_avg);

    // 设置相同线性化点
    cpi_v1->setLinearizationPoints(b_w, b_a);
    cpi_model.InitBias(b_w, b_a);

    // 固定 IMU 序列（多段），边界处 pair[i].w1==pair[i+1].w0，保证 CpiSequence 单点序列与 CpiModel 一致
    struct ImuPair {
      double t0, t1;
      Eigen::Vector3d w0, a0, w1, a1;
    };
    Eigen::Vector3d w1 = vec3(0.1, 0.0, 0.0), a1 = vec3(0, 0, -9.81);
    Eigen::Vector3d w2 = vec3(0.2, 0.1, -0.05), a2 = vec3(0.1, 0.2, -9.8);
    Eigen::Vector3d w3 = vec3(0.5, -0.3, 0.2), a3 = vec3(-0.1, 0.1, -9.81);
    std::vector<ImuPair> pairs = {
        {0.0, 0.01, w1, a1, w2, a2},
        {0.01, 0.02, w2, a2, w3, a3},
        {0.02, 0.05, w3, a3, w3, a3},
    };

    for (const auto& p : pairs) {
      Eigen::Matrix<double, 3, 1> w0, a0, w1, a1;
      w0 << p.w0(0), p.w0(1), p.w0(2);
      a0 << p.a0(0), p.a0(1), p.a0(2);
      w1 << p.w1(0), p.w1(1), p.w1(2);
      a1 << p.a1(0), p.a1(1), p.a1(2);

      cpi_v1->feed_IMU(p.t0, p.t1, w0, a0, w1, a1);
      cpi_model.FeedImu(p.t0, p.t1, p.w0, p.a0, p.w1, p.a1);
    }

    // 比较 DT
    if (!is_close(cpi_v1->DT, cpi_model.DT)) {
      printf("  FAIL DT: CpiV1=%.12e CpiModel=%.12e\n", cpi_v1->DT, cpi_model.DT);
      ++failed;
    } else {
      printf("  OK DT: %.12e\n", cpi_model.DT);
    }

    // 比较 alpha_tau, beta_tau
    Eigen::Vector3d alpha_v1 = cpi_v1->alpha_tau;
    Eigen::Vector3d beta_v1 = cpi_v1->beta_tau;
    if (!matrix_close(alpha_v1, cpi_model.alpha_tau)) {
      printf("  FAIL alpha_tau\n");
      ++failed;
    } else {
      printf("  OK alpha_tau\n");
    }
    if (!matrix_close(beta_v1, cpi_model.beta_tau)) {
      printf("  FAIL beta_tau\n");
      ++failed;
    } else {
      printf("  OK beta_tau\n");
    }

    // 比较 R_k2tau（旋转矩阵，避免四元数格式差异）
    if (!matrix_close(cpi_v1->R_k2tau, cpi_model.R_k2tau)) {
      printf("  FAIL R_k2tau\n");
      ++failed;
    } else {
      printf("  OK R_k2tau\n");
    }

    // 比较雅可比
    if (!matrix_close(cpi_v1->J_q, cpi_model.J_q)) {
      printf("  FAIL J_q\n");
      ++failed;
    } else {
      printf("  OK J_q\n");
    }
    if (!matrix_close(cpi_v1->J_a, cpi_model.J_a)) {
      printf("  FAIL J_a\n");
      ++failed;
    } else {
      printf("  OK J_a\n");
    }
    if (!matrix_close(cpi_v1->J_b, cpi_model.J_b)) {
      printf("  FAIL J_b\n");
      ++failed;
    } else {
      printf("  OK J_b\n");
    }
    if (!matrix_close(cpi_v1->H_a, cpi_model.H_a)) {
      printf("  FAIL H_a\n");
      ++failed;
    } else {
      printf("  OK H_a\n");
    }
    if (!matrix_close(cpi_v1->H_b, cpi_model.H_b)) {
      printf("  FAIL H_b\n");
      ++failed;
    } else {
      printf("  OK H_b\n");
    }

    // 比较协方差 P_meas
    if (!matrix_close(cpi_v1->P_meas, cpi_model.P_meas())) {
      printf("  FAIL P_meas\n");
      ++failed;
    } else {
      printf("  OK P_meas\n");
    }

    // CpiSequence（AddImu 时预积分）与 CpiModel 一致
    CpiSequence seq(sigma_w, sigma_wb, sigma_a, sigma_ab, imu_avg);
    seq.SetBias(b_w, b_a);
    double t_first = pairs.front().t0, t_last = pairs.back().t1;
    for (const auto& p : pairs) {
      seq.AddImu(p.t0, p.w0, p.a0);
    }
    seq.AddImu(t_last, pairs.back().w1, pairs.back().a1);
    CpiResult res_seq;
    if (seq.GetPreintegration(t_first, t_last, &res_seq)) {
      bool ok = is_close(cpi_model.DT, res_seq.DT) && matrix_close(cpi_model.alpha_tau, res_seq.alpha_tau) &&
                matrix_close(cpi_model.beta_tau, res_seq.beta_tau) && matrix_close(cpi_model.R_k2tau, res_seq.R_k2tau);
      if (!ok) {
        printf("  FAIL CpiSequence vs CpiModel (DT diff=%.2e alpha=%.2e beta=%.2e R=%.2e)\n",
               std::abs(cpi_model.DT - res_seq.DT),
               (cpi_model.alpha_tau - res_seq.alpha_tau).lpNorm<Eigen::Infinity>(),
               (cpi_model.beta_tau - res_seq.beta_tau).lpNorm<Eigen::Infinity>(),
               (cpi_model.R_k2tau - res_seq.R_k2tau).lpNorm<Eigen::Infinity>());
        ++failed;
      } else {
        printf("  OK CpiSequence (AddImu 时预积分) vs CpiModel\n");
      }
    } else {
      printf("  FAIL CpiSequence GetPreintegration\n");
      ++failed;
    }
  }

  printf("\n=== CpiSequence 单段验证 ===\n");
  {
    CpiModel m(1e-3, 1e-4, 1e-2, 1e-3, true);
    m.InitBias(vec3(0, 0, 0), vec3(0, 0, -9.81));
    m.FeedImu(0.0, 0.01, vec3(0.1, 0, 0), vec3(0, 0, -9.81), vec3(0.1, 0, 0), vec3(0, 0, -9.81));
    CpiSequence seq(1e-3, 1e-4, 1e-2, 1e-3, true);
    seq.SetBias(vec3(0, 0, 0), vec3(0, 0, -9.81));
    seq.AddImu(0.0, vec3(0.1, 0, 0), vec3(0, 0, -9.81));
    seq.AddImu(0.01, vec3(0.1, 0, 0), vec3(0, 0, -9.81));
    CpiResult r;
    seq.GetPreintegration(0.0, 0.01, &r);
    if (is_close(m.DT, r.DT) && matrix_close(m.alpha_tau, r.alpha_tau) &&
        matrix_close(m.beta_tau, r.beta_tau) && matrix_close(m.R_k2tau, r.R_k2tau)) {
      printf("  单段 OK\n");
    } else {
      printf("  单段 FAIL (DT=%.2e alpha=%.2e)\n", std::abs(m.DT - r.DT),
             (m.alpha_tau - r.alpha_tau).lpNorm<Eigen::Infinity>());
      ++failed;
    }
  }

  printf("\n=== 汇总 ===\n");
  if (failed > 0) {
    printf("失败 %d 项\n", failed);
    return EXIT_FAILURE;
  }
  printf("全部通过，CpiModel 与 CpiV1 一致。\n");
  return EXIT_SUCCESS;
}
