/*
 * 编译信息导出实现（Build Info）
 * 依赖编译时生成的 build_info.h（由 ov_core CMake 的 ov_core_build_info 目标生成）
 */
#include "ext/ss_vio debug.h"
#include <cstring>
#include <cstdio>

#include "build_info.h"  /* 由 ov_core CMake 的 ov_core_build_info 目标在编译时生成 */

namespace {

void copy_str(char* dst, size_t dst_size, const char* src) {
  if (!dst || dst_size == 0) return;
  size_t n = strlen(src);
  if (n >= dst_size) n = dst_size - 1;
  memcpy(dst, src, n);
  dst[n] = '\0';
}

const char* get_timestamp() {
#ifdef BUILD_TIMESTAMP
  return BUILD_TIMESTAMP;
#else
  return "unknown";
#endif
}

const char* get_timezone() {
#ifdef BUILD_TIMEZONE
  return BUILD_TIMEZONE;
#else
  return "";
#endif
}

const char* get_git_commit() {
#ifdef BUILD_GIT_COMMIT
  return BUILD_GIT_COMMIT;
#else
  return "unknown";
#endif
}

const char* get_git_hash() {
#ifdef BUILD_GIT_HASH
  return BUILD_GIT_HASH;
#else
  return "unknown";
#endif
}

const char* get_git_branch() {
#ifdef BUILD_GIT_BRANCH
  return BUILD_GIT_BRANCH;
#else
  return "unknown";
#endif
}

const char* get_git_tag() {
#ifdef BUILD_GIT_TAG
  return BUILD_GIT_TAG;
#else
  return "none";
#endif
}

const char* get_git_status() {
#ifdef BUILD_GIT_DIRTY
  return BUILD_GIT_DIRTY;
#else
  return "unknown";
#endif
}

const char* get_git_user() {
#ifdef BUILD_GIT_USER
  return BUILD_GIT_USER;
#else
  return "unknown";
#endif
}

const char* get_git_email() {
#ifdef BUILD_GIT_EMAIL
  return BUILD_GIT_EMAIL;
#else
  return "unknown";
#endif
}

}  // namespace

extern "C" {

int SS_VIO_GetBuildInfo(SS_VIO_BuildInfo* out) {
  if (!out) return -1;
  copy_str(out->build_time, sizeof(out->build_time), get_timestamp());
  copy_str(out->build_timezone, sizeof(out->build_timezone), get_timezone());
  copy_str(out->git_commit, sizeof(out->git_commit), get_git_commit());
  copy_str(out->git_hash, sizeof(out->git_hash), get_git_hash());
  copy_str(out->git_branch, sizeof(out->git_branch), get_git_branch());
  copy_str(out->git_tag, sizeof(out->git_tag), get_git_tag());
  copy_str(out->git_status, sizeof(out->git_status), get_git_status());
  copy_str(out->git_user, sizeof(out->git_user), get_git_user());
  copy_str(out->git_email, sizeof(out->git_email), get_git_email());
  return 0;
}

int SS_VIO_FormatBuildInfo(char* buffer, size_t size) {
  char tmp[512];
  int n = snprintf(tmp, sizeof(tmp),
      "\n========== Build Info ==========\n"
      "  Build Time:   %s %s\n"
      "  Git Commit:   %s (%s)\n"
      "  Git Branch:   %s\n"
      "  Git Tag:      %s\n"
      "  Git Status:   %s\n"
      "  Git User:     %s <%s>\n"
      "=================================\n",
      get_timestamp(), get_timezone(),
      get_git_commit(), get_git_hash(),
      get_git_branch(), get_git_tag(), get_git_status(),
      get_git_user(), get_git_email());
  if (n < 0) return -1;
  size_t len = static_cast<size_t>(n);
  if (buffer && size > 0) {
    size_t copy = len < size ? len : size - 1;
    memcpy(buffer, tmp, copy);
    buffer[copy] = '\0';
  }
  return static_cast<int>(len);
}

}  // extern "C"
