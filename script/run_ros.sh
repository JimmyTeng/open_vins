#!/usr/bin/env bash
#
# OpenVINS ROS 运行脚本
#
# 用法：
#   ./script/run_ros.sh                      # 默认参数启动
#   ./script/run_ros.sh --data_dir /path     # 指定数据目录
#   ./script/run_ros.sh --no-rviz            # 不启动 RVIZ
#   ./script/run_ros.sh --subscribe         # 仅订阅模式（需外部发布 /imu0, /cam0/image_raw）
#   ./script/run_ros.sh --serial <bag>       # 从 rosbag 读取（替代 YUV 播包）
#
# 环境变量（可选）：
#   OPENVINS_WS     - catkin 工作空间路径（含 devel 或 install）
#   ROS_DISTRO      - 需已设置（source /opt/ros/noetic/setup.bash）
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 默认参数
DATA_DIR="${HOME}/project/open_vins_ws/data/yuv_data/250704/8"
LAUNCH_MODE="run_vio"    # run_vio | subscribe | serial
DO_RVIZ="true"
PUBLISH_RATE="1.0"
VERBOSITY="INFO"
CONFIG_PATH=""
BAG_PATH=""
BAG_START="0"
DOSAVE="false"
PATH_EST="/tmp/traj_estimate.txt"

usage() {
  cat <<'EOF'
用法: ./script/run_ros.sh [选项]

选项:
  --data_dir PATH      YUV+IMU 数据目录（run_vio 模式）
  --bag PATH           使用 rosbag 播放（serial 模式）
  --bag_start SEC      从 rosbag 第 SEC 秒开始
  --subscribe           仅订阅模式，不启动播包（需外部发布话题）
  --serial BAG          serial 模式，从 rosbag 读取
  --no-rviz            不启动 RVIZ
  --save PATH          保存轨迹到文件
  --rate RATE          播包倍率（默认 1.0）
  -h, --help           显示帮助
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --data_dir)
      [[ $# -ge 2 ]] || { echo "错误: $1 需要参数" >&2; exit 1; }
      DATA_DIR="$2"
      shift 2
      ;;
    --bag)
      [[ $# -ge 2 ]] || { echo "错误: $1 需要参数" >&2; exit 1; }
      BAG_PATH="$2"
      LAUNCH_MODE="serial"
      shift 2
      ;;
    --bag_start)
      [[ $# -ge 2 ]] || { echo "错误: $1 需要参数" >&2; exit 1; }
      BAG_START="$2"
      shift 2
      ;;
    --subscribe)
      LAUNCH_MODE="subscribe"
      shift
      ;;
    --serial)
      [[ $# -ge 2 ]] || { echo "错误: $1 需要参数" >&2; exit 1; }
      BAG_PATH="$2"
      LAUNCH_MODE="serial"
      shift 2
      ;;
    --no-rviz)
      DO_RVIZ="false"
      shift
      ;;
    --save)
      [[ $# -ge 2 ]] || { echo "错误: $1 需要参数" >&2; exit 1; }
      PATH_EST="$2"
      DOSAVE="true"
      shift 2
      ;;
    --rate)
      [[ $# -ge 2 ]] || { echo "错误: $1 需要参数" >&2; exit 1; }
      PUBLISH_RATE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知选项: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

# 查找工作空间
find_workspace() {
  local dir="$PROJECT_ROOT"
  while [ -n "$dir" ] && [ "$dir" != "/" ]; do
    if [ -f "${dir}/devel/setup.bash" ]; then
      echo "${dir}"
      return 0
    fi
    if [ -f "${dir}/install/setup.bash" ]; then
      echo "${dir}"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

echo "========================================"
echo "OpenVINS ROS 运行"
echo "========================================"

# Source ROS
if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -f "/opt/ros/noetic/setup.bash" ]; then
    echo "加载 ROS noetic..."
    source /opt/ros/noetic/setup.bash
  elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    echo "加载 ROS melodic..."
    source /opt/ros/melodic/setup.bash
  else
    echo "错误: 未找到 ROS，请先执行: source /opt/ros/<distro>/setup.bash" >&2
    exit 1
  fi
fi

# Source 工作空间
WS="${OPENVINS_WS:-}"
if [ -z "$WS" ]; then
  WS="$(find_workspace || true)"
fi
if [ -n "$WS" ]; then
  if [ -f "${WS}/devel/setup.bash" ]; then
    echo "加载工作空间: ${WS} (devel)"
    source "${WS}/devel/setup.bash"
  elif [ -f "${WS}/install/setup.bash" ]; then
    echo "加载工作空间: ${WS} (install)"
    source "${WS}/install/setup.bash"
  fi
else
  echo "提示: 未找到 catkin 工作空间，若 roslaunch 失败请设置 OPENVINS_WS" >&2
  # 尝试将当前项目加入 ROS_PACKAGE_PATH
  if [ -d "${PROJECT_ROOT}/ov_ros" ]; then
    export ROS_PACKAGE_PATH="${PROJECT_ROOT}:${ROS_PACKAGE_PATH:-}"
    echo "已将项目加入 ROS_PACKAGE_PATH"
  fi
fi

# 选择 launch 文件并构造参数
case "$LAUNCH_MODE" in
  run_vio)
    echo "模式: run_vio (YUV+IMU 播包 + OpenVINS)"
    echo "数据目录: ${DATA_DIR}"
    echo "RVIZ: ${DO_RVIZ}"
    roslaunch ov_ros run_vio.launch \
      data_dir:="${DATA_DIR}" \
      publish_rate:="${PUBLISH_RATE}" \
      rviz_enable:="${DO_RVIZ}" \
      verbosity:="${VERBOSITY}" \
      dosave:="${DOSAVE}" \
      path_est:="${PATH_EST}"
    ;;
  subscribe)
    echo "模式: subscribe (仅 OpenVINS，需外部发布 /imu0、/cam0/image_raw)"
    roslaunch ov_ros run_openvins.launch \
      rviz_enable:="${DO_RVIZ}" \
      verbosity:="${VERBOSITY}" \
      dosave:="${DOSAVE}" \
      path_est:="${PATH_EST}"
    ;;
    serial)
    if [ -z "$BAG_PATH" ] || [ ! -f "$BAG_PATH" ]; then
      echo "错误: serial 模式需要有效的 bag 文件: $BAG_PATH" >&2
      exit 1
    fi
    echo "模式: serial (从 rosbag 读取)"
    echo "Bag: ${BAG_PATH}"
    roslaunch ov_ros serial.launch \
      bag:="${BAG_PATH}" \
      bag_start:="${BAG_START}" \
      dosave:="${DOSAVE}" \
      path_est:="${PATH_EST}"
    ;;
  *)
    echo "错误: 未知模式 $LAUNCH_MODE" >&2
    exit 1
    ;;
esac
