#!/bin/bash
# ============================================================================
#  生成大恒相机 SDK 的占位库（stub），供容器内编译链接使用
# ============================================================================
#
#  为什么需要：
#  io/CMakeLists.txt 最后一行链接了 gxiapi，但这个库【不在仓库内】——
#  io/daheng/ 下只有源码和厂商头文件，没有 lib/ 目录。真库要去大恒官网
#  下载 Galaxy SDK 全局安装（见 io/CMakeLists.txt 里那段说明）。
#
#  而且它不是可选项：camera.cpp 的 DahengCamera::initSDK() 无条件调用，
#  无论 YAML 里选哪个品牌的相机都要链上。缺了的失败方式很坑 ——
#  cmake 能过、每个 .cpp 都能编过，只有最后一步链接报 "cannot find -lgxiapi"，
#  也就是等十几分钟之后在最后一秒失败。
#
#  容器里跑的是 auto_aim_test（读录像离线测试），根本不碰真相机，
#  所以这里生成一个函数全部返回错误码的占位库，让链接能过。
#  这和 WSL 环境里的做法一致（见 怎么编译运行.md 最后一节的说明）。
#
#  ⚠️ 占位库只能满足"编译 + 跑离线录像"。真要连相机必须装官方 SDK。
# ============================================================================
set -e

HEADER_DIR=/workspace/io/daheng/include
OUT_DIR=/usr/local/lib
SRC=/tmp/daheng_stub.c

if [ -f "$OUT_DIR/libgxiapi.so" ]; then
    exit 0
fi

# 从厂商头文件里抽出所有导出函数名。
#   GxIAPI.h    里的原型形如   GX_API GXInitLib (...)
#   DxImageProc.h 里形如        VxInt32 DxRaw8toRGB24 (...)
# 只取函数名，不管参数：C 语言里未声明参数的定义可以接受任意调用，
# 而符号名一致就足以让链接器满意。
# 注意 DxImageProc.h 的原型中间夹了调用约定宏：
#   VxInt32 DHDECL DxRaw8toRGB24(...)
#            ^^^^^^ 这个
# 所以不能要求返回类型和函数名相邻，中间要允许若干个标识符。
gen_stub() {
    local header=$1 pattern=$2
    grep -ohE "$pattern" "$header" 2>/dev/null \
        | grep -ohE '\b(GX|Dx|DX)[A-Za-z0-9_]+' \
        | grep -vE '^(DX_|DHDECL)' \
        | sort -u
}

{
    echo '/* 自动生成的占位实现，请勿手工编辑。所有函数返回非零错误码。 */'
    echo 'typedef int GX_STATUS;'
    # -1 对应大恒的 GX_STATUS_ERROR。程序若真去调用会拿到失败，
    # 而不是拿到看似成功的假数据 —— 后者更难排查。
    for fn in $(gen_stub "$HEADER_DIR/GxIAPI.h" 'GX_API +GX[A-Za-z0-9_]+'); do
        echo "int ${fn}() { return -1; }"
    done
    for fn in $(gen_stub "$HEADER_DIR/DxImageProc.h" '(VxInt32|VxUint32|DX_API|void) +([A-Za-z0-9_]+ +)?D[xX][A-Za-z0-9_]+ *\('); do
        echo "int ${fn}() { return -1; }"
    done
} > "$SRC"

COUNT=$(grep -c 'return -1' "$SRC")
if [ "$COUNT" -lt 20 ]; then
    echo "占位库生成失败：只解析出 $COUNT 个函数，头文件格式可能变了" >&2
    exit 1
fi

# -shared -fPIC 生成动态库。不加 -Wl,--no-undefined，
# 因为占位实现之间没有真实依赖关系。
gcc -shared -fPIC -o "$OUT_DIR/libgxiapi.so" "$SRC"
ldconfig

echo "已生成大恒占位库（$COUNT 个函数）→ $OUT_DIR/libgxiapi.so"
