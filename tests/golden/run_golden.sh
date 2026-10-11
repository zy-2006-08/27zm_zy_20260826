#!/bin/bash
# 用法(仓库根目录): bash tests/golden/run_golden.sh <label>
# 用当前代码编 replay_dump, 对 tests/golden/logs.txt 里每份日志跑 bullet / laser 两种模式,
# 输出到 /home/seven/RM27_zimiao/golden/<label>/<日志名>.{bullet,laser}.txt (已过滤程序日志行)。
set -euo pipefail

label="${1:?usage: run_golden.sh <label>}"
root="$(cd "$(dirname "$0")/../.." && pwd)"
out="${GOLDEN_ROOT:-/home/seven/RM27_zimiao/golden}/$label"
cfg="$root/configs/sb_long.yaml"
laser_cfg="/tmp/golden_laser_$$.yaml"

cd "$root"
cmake -S . -B build > /tmp/golden_cmake_$$.log 2>&1 || { tail -30 /tmp/golden_cmake_$$.log; exit 1; }
make -C build -j8 replay_dump > /tmp/golden_make_$$.log 2>&1 || { tail -40 /tmp/golden_make_$$.log; exit 1; }

# 激光模式配置: 有 fire_mode 行就改成 laser, 否则把 laser_pitch_mode 1 改成 0
if grep -q '^fire_mode:' "$cfg"; then
  sed 's/^fire_mode:.*/fire_mode: laser/' "$cfg" > "$laser_cfg"
else
  grep -q '^laser_pitch_mode: *1' "$cfg" || { echo "找不到 fire_mode 或 laser_pitch_mode: 1" >&2; exit 1; }
  sed 's/^laser_pitch_mode: *1/laser_pitch_mode: 0/' "$cfg" > "$laser_cfg"
fi
cmp -s "$cfg" "$laser_cfg" && { echo "激光配置没改到" >&2; exit 1; }

mapfile -t logs < <(grep -v '^\s*$' "$root/tests/golden/logs.txt")
for f in "${logs[@]}"; do
  [ -f "$f" ] || { echo "日志不存在: $f" >&2; exit 1; }
done

rm -rf "$out"
mkdir -p "$out"
cd build
pids=()
for f in "${logs[@]}"; do
  n="$(basename "$f" .log)"
  ( ./replay_dump "$f" "$cfg" 2>/dev/null | grep -v '^\[' > "$out/$n.bullet.txt" ) &
  pids+=($!)
  ( ./replay_dump "$f" "$laser_cfg" 2>/dev/null | grep -v '^\[' > "$out/$n.laser.txt" ) &
  pids+=($!)
done
fail=0
for p in "${pids[@]}"; do wait "$p" || fail=1; done
rm -f "$laser_cfg"
[ "$fail" = 0 ] || { echo "有回放失败" >&2; exit 1; }

for f in "$out"/*.txt; do
  [ -s "$f" ] || { echo "空输出: $f" >&2; exit 1; }
done
wc -l "$out"/*.txt | tail -1
echo "OK $out"
