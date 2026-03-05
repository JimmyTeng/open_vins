/*
 * OpenVINS: Simple string utilities (replacing Boost.Algorithm)
 */
#ifndef OV_EVAL_STRING_OPS_H
#define OV_EVAL_STRING_OPS_H

#include <string>

namespace ov_eval {

inline void replace_all(std::string& s, const std::string& from, const std::string& to) {
  for (size_t pos = 0; (pos = s.find(from, pos)) != std::string::npos; pos += to.size()) {
    s.replace(pos, from.size(), to);
  }
}

inline bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

} // namespace ov_eval

#endif /* OV_EVAL_STRING_OPS_H */
