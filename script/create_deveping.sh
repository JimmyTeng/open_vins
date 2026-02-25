#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法：
  bash script/create_deveping.sh <n> [n2 n3 ...]
  bash script/create_deveping.sh deveping4 deveping5
  bash script/create_deveping.sh --force 4

说明：
  - 会在 src/devepingN/ 下创建：
      - CMakeLists.txt
      - src/main.cpp
      - tests/smoke_test.cpp
      - README.md
  - 默认不覆盖已存在目录；使用 --force 才会覆盖同名文件。
  - 不会修改任何 git 或提交。

启用构建（示例）：
  cmake -S . -B build/dev -DMYVIO_BUILD_DEVEPING=ON -DMYVIO_DEVEPING_PROJECTS=deveping4
EOF
}

FORCE=0
ARGS=()
for a in "$@"; do
  case "$a" in
    -h|--help) usage; exit 0 ;;
    --force) FORCE=1 ;;
    *) ARGS+=("$a") ;;
  esac
done

if [[ ${#ARGS[@]} -lt 1 ]]; then
  usage
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

to_name() {
  local x="$1"
  if [[ "$x" =~ ^[0-9]+$ ]]; then
    echo "deveping${x}"
  elif [[ "$x" == deveping* ]]; then
    echo "$x"
  else
    echo "deveping${x}"
  fi
}

write_file() {
  local path="$1"
  local content="$2"
  if [[ -e "$path" && "$FORCE" -ne 1 ]]; then
    echo "skip (exists): $path"
    return 0
  fi
  mkdir -p "$(dirname "$path")"
  printf "%s" "$content" > "$path"
  echo "write: $path"
}

for raw in "${ARGS[@]}"; do
  name="$(to_name "$raw")"
  dir="${REPO_ROOT}/src/${name}"

  if [[ -d "$dir" && "$FORCE" -ne 1 ]]; then
    echo "skip (dir exists): $dir"
    continue
  fi

  mkdir -p "${dir}/src" "${dir}/tests"

  write_file "${dir}/CMakeLists.txt" "$(cat <<EOF
#
# ${name}: 并行开发/试验沙盒子项目
#

add_executable(myvio_${name}_sandbox
    src/main.cpp
)

target_link_libraries(myvio_${name}_sandbox
    PRIVATE myvio_utils
    PRIVATE myvio_sensor_models
    PRIVATE myvio_thirdparty_usage_tests
)

if(BUILD_TESTS)
    add_executable(myvio_${name}_smoke_test
        tests/smoke_test.cpp
    )
    target_link_libraries(myvio_${name}_smoke_test
        PRIVATE myvio_utils
        PRIVATE myvio_sensor_models
        PRIVATE myvio_thirdparty_usage_tests
        PRIVATE GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(myvio_${name}_smoke_test)
endif()
EOF
)"

  write_file "${dir}/src/main.cpp" "$(cat <<EOF
#include <sensor_models/pinhole_camera.h>
#include <thirdparty_usage_tests/thirdparty_usage_tests.h>
#include <utils/timer.h>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

int main() {
  utils::Timer timer;

  sensor_models::PinholeCamera cam(/*fx=*/500.0, /*fy=*/500.0, /*cx=*/320.0,
                                   /*cy=*/240.0, /*k1=*/0.0, /*k2=*/0.0,
                                   /*p1=*/0.0, /*p2=*/0.0);
  const Eigen::Vector2d uv = cam.Project(Eigen::Vector3d(0.1, 0.2, 1.0));

  spdlog::info("[${name}] uv=({}, {}), thirdparty_module={}", uv.x(), uv.y(),
               myvio::thirdparty_usage_tests::kModuleName);
  timer.PrintElapsed("[${name}] finished");
  return 0;
}
EOF
)"

  write_file "${dir}/tests/smoke_test.cpp" "$(cat <<EOF
#include <sensor_models/pinhole_camera.h>
#include <thirdparty_usage_tests/thirdparty_usage_tests.h>
#include <utils/timer.h>

#include <gtest/gtest.h>

TEST(${name}, CanLinkExistingModules) {
  utils::Timer timer;
  EXPECT_GE(timer.ElapsedMs(), 0.0);

  sensor_models::PinholeCamera cam(/*fx=*/500.0, /*fy=*/500.0, /*cx=*/320.0,
                                   /*cy=*/240.0, /*k1=*/0.0, /*k2=*/0.0,
                                   /*p1=*/0.0, /*p2=*/0.0);
  const Eigen::Vector2d uv = cam.Project(Eigen::Vector3d(0.1, 0.2, 1.0));
  EXPECT_TRUE(uv.allFinite());

  EXPECT_NE(myvio::thirdparty_usage_tests::kModuleName, nullptr);
}
EOF
)"

  write_file "${dir}/README.md" "$(cat <<EOF
# ${name}

这是一个 **并行开发/试验沙盒子项目**，用于把临时代码隔离在 \`src/${name}/\` 里，方便多个 agent 同时开发、互不干扰。

## 启用构建

顶层默认不编译 deveping 沙盒。启用方式（示例）：

- \`-DMYVIO_BUILD_DEVEPING=ON\`
- 仅构建本沙盒：\`-DMYVIO_DEVEPING_PROJECTS=${name}\`

## 迁移（开发完成后）

当某段代码成熟可复用时，把它（以及对应测试）移动到正式模块目录（如 \`src/utils\` / \`src/sensor_models\`），并尽量保持本目录“临时/一次性”。
EOF
)"

  echo
  echo "created: src/${name}"
  echo "enable : -DMYVIO_BUILD_DEVEPING=ON -DMYVIO_DEVEPING_PROJECTS=${name}"
  echo
done

