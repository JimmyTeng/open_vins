/*
 * 加计零偏 ba 根式软限幅（保方向、保留各轴正负）：out = in * L / sqrt(||in||^2 + k^2)
 */
#ifndef OV_CORE_ACC_BIAS_SOFT_LIMIT_H
#define OV_CORE_ACC_BIAS_SOFT_LIMIT_H

#include <cmath>

namespace ov_core {

/** L>0 且 k>0 时缩放；否则 out = in（原样通过） */
inline void acc_bias_sqrt_soft_limit_xyz(double bx, double by, double bz,
                                         double L, double k, double *out_x,
                                         double *out_y, double *out_z) {
  const double m2 = bx * bx + by * by + bz * bz;
  if (!(L > 0.0) || !(k > 0.0)) {
    *out_x = bx;
    *out_y = by;
    *out_z = bz;
    return;
  }
  const double scale = L / std::sqrt(m2 + k * k);
  *out_x = bx * scale;
  *out_y = by * scale;
  *out_z = bz * scale;
}

} // namespace ov_core

#endif // OV_CORE_ACC_BIAS_SOFT_LIMIT_H
