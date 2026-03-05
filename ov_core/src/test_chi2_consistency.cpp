/**
 * @file test_chi2_consistency.cpp
 * @brief 验证手写 chi2_quantile_095 与 Boost.Math 的一致性
 *
 * 编译方式（需安装 Boost）:
 *   g++ -std=c++17 -O2 -I. test_chi2_consistency.cpp -o test_chi2_consistency -lboost_math_c99
 *
 * 或使用 scipy 验证（无需 Boost）:
 *   python3 script/verify_chi2_consistency.py
 */

#include "utils/chi_squared_quantile.h"

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

#ifdef HAS_BOOST_MATH
#include <boost/math/distributions/chi_squared.hpp>
#endif

int main() {
  const int test_dfs[] = {1, 2, 3, 5, 10, 15, 20, 25, 50, 100, 200, 300, 500, 999, 1000, 2000};
  const int n_tests = sizeof(test_dfs) / sizeof(test_dfs[0]);

  std::cout << "=== 卡方 0.95 分位数一致性检验 ===\n";
  std::cout << std::fixed << std::setprecision(6);
  std::cout << std::setw(6) << "df" << std::setw(16) << "手写实现" << std::setw(16) << "Boost.Math"
            << std::setw(14) << "绝对误差" << std::setw(12) << "相对误差%\n";
  std::cout << "------------------------------------------------------------\n";

  double max_abs_err = 0.0;
  double max_rel_err = 0.0;
  int fail_count = 0;

  for (int i = 0; i < n_tests; i++) {
    int df = test_dfs[i];
    double ours = ov_core::chi2_quantile_095(df);

#ifdef HAS_BOOST_MATH
    boost::math::chi_squared dist(df);
    double boost_val = boost::math::quantile(dist, 0.95);
    double abs_err = std::fabs(ours - boost_val);
    double rel_err = (boost_val != 0.0) ? (abs_err / boost_val * 100.0) : 0.0;

    max_abs_err = std::max(max_abs_err, abs_err);
    max_rel_err = std::max(max_rel_err, rel_err);

    bool ok = (abs_err < 0.01) || (rel_err < 0.1);  // 容差：0.01 或 0.1%
    if (!ok) fail_count++;

    std::cout << std::setw(6) << df << std::setw(16) << ours << std::setw(16) << boost_val
              << std::setw(14) << abs_err << std::setw(11) << rel_err << "%"
              << (ok ? "  OK" : "  FAIL") << "\n";
#else
    std::cout << std::setw(6) << df << std::setw(16) << ours
              << "  (无 Boost，请用 script/verify_chi2_consistency.py 对比 scipy)\n";
#endif
  }

#ifdef HAS_BOOST_MATH
  std::cout << "------------------------------------------------------------\n";
  std::cout << "最大绝对误差: " << max_abs_err << "\n";
  std::cout << "最大相对误差: " << max_rel_err << "%\n";
  if (fail_count > 0) {
    std::cout << "失败: " << fail_count << " 个测试点\n";
    return 1;
  }
  std::cout << "全部通过\n";
#endif
  return 0;
}
