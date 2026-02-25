#!/usr/bin/env bash
# 启用 build_preset.sh 的 Tab 补全（preset 名、选项等）
#
# 用法（在项目根目录或任意目录）：
#   source script/activate.sh
# 或
#   . script/activate.sh
#
# 启用后，输入 ./script/build_preset.sh 后按 Tab 可补全 preset 与选项。

if [[ -z "${BASH_SOURCE[0]:-}" ]]; then
  echo "请在 bash 中执行: source script/activate.sh" >&2
  return 1 2>/dev/null || exit 1
fi

_activate_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_build_preset="${_activate_script_dir}/build_preset.sh"

if [[ ! -x "${_build_preset}" && ! -f "${_build_preset}" ]]; then
  echo "未找到 script/build_preset.sh，请确认在仓库内执行。" >&2
  return 1 2>/dev/null || exit 1
fi

# 加载补全
if source <("${_build_preset}" --completion-bash 2>/dev/null); then
  echo "build_preset.sh 补全已启用（输入 ./script/build_preset.sh 后按 Tab 试一下）"
else
  echo "加载 build_preset.sh 补全失败。" >&2
  return 1 2>/dev/null || exit 1
fi

unset _activate_script_dir _build_preset
