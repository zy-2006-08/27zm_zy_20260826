#!/usr/bin/env bash
# ============================================================================
#  Generate the TensorRT engine for the yolov8 armor detector (Jetson only).
# ----------------------------------------------------------------------------
#  A .engine is tied to this exact GPU + TensorRT version, so it is NOT in git
#  and cannot be copied between machines. Run this once per Jetson (and again
#  after a JetPack/TensorRT upgrade):
#
#      bash scripts/build_trt_engine.sh
#
#  Input : assets/yolov8.onnx   (exported from assets/yolov8.xml by tools/ir2onnx.py)
#  Output: assets/yolov8_fp16.engine  (default path read by yolov8_trt.cpp)
#  Takes a few minutes. x86 machines do not need this: they use OpenVINO.
# ============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ONNX="$ROOT/assets/yolov8.onnx"
ENGINE="$ROOT/assets/yolov8_fp16.engine"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

if [ ! -x "$TRTEXEC" ]; then
    echo "!! trtexec not found at $TRTEXEC (TensorRT missing? this is only for Jetson)" >&2
    exit 1
fi
if [ ! -f "$ONNX" ]; then
    echo "!! $ONNX not found. Export it first: python3 tools/ir2onnx.py" >&2
    exit 1
fi

echo "==> building $ENGINE (fp16) ..."
"$TRTEXEC" --onnx="$ONNX" --saveEngine="$ENGINE" --fp16 --skipInference
echo "==> done: $ENGINE"
