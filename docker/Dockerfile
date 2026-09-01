# ============================================================================
#  在 Docker 里编译运行本工程（macOS / Windows 皆可，不必装 WSL）
# ============================================================================
#
#  为什么需要这个文件：
#  本工程只能在 Linux 上编译（OpenVINO / 大恒相机 SDK 都是 Linux 的），
#  但代码可能在 Mac 上写。容器提供一个和车上小电脑一致的 Ubuntu 22.04 环境。
#
#  ⚠️ 架构必须是 x86_64（linux/amd64）。
#  OpenVINO 是 Intel 的产品，apt 源里只有 x86_64 包，arm64 上装不了。
#  Apple Silicon 上靠 Rosetta/QEMU 翻译执行，编译会比原生慢，但能跑通。
#  build.sh 已经带上 --platform linux/amd64，不要手动改成别的架构。
#
#  版本对齐车上环境（见 怎么编译运行.md 最后一节）：
#    Ubuntu 22.04 / g++ 11 / OpenCV 4.5.4 / Eigen 3.4 / OpenVINO 2024.6.0
# ============================================================================
FROM --platform=linux/amd64 ubuntu:22.04

# 避免 apt 在构建时弹交互式时区选择卡住
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# ---------------------------------------------------------------------------
# 换成国内 apt 源。
# 默认的 archive.ubuntu.com 在国内拉几百个包会很慢；这一层要装 400+ 个包
# （libopencv-dev 会带进一大串依赖），换源能省下大量时间。
# 实测阿里云约 0.36s 响应，是几个常用镜像里最快的。
# ---------------------------------------------------------------------------
# 这里有两个坑，都会表现为一堆 "Unable to locate package"，而报错信息
# 完全指不到真正的原因，所以写下来：
#
# 1) 用 http 而不是 https。ubuntu:22.04 基础镜像里没有 ca-certificates，
#    走 https 会因证书无法验证而拉不到索引 —— 而 ca-certificates 本身又得靠
#    apt 才能装上，形成死循环。apt 自己会校验包签名，http 不影响安全性。
# 2) 整份重写，不用 sed 替换域名。镜像里的地址是 archive.ubuntu.com/ubuntu/
#    （结尾带斜杠），按域名替换很容易拼出 .../ubuntu/ubuntu/ 这种重复路径。
RUN printf '%s\n' \
      'deb http://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse' \
      'deb http://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse' \
      'deb http://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse' \
      'deb http://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse' \
      > /etc/apt/sources.list

# ---------------------------------------------------------------------------
# 第一层：编译工具链 + 工程的六个第三方库
# 和 CMakeLists.txt 注释里那行 apt install 完全对应。
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        pkg-config \
        ca-certificates \
        curl \
        gnupg \
        libopencv-dev \
        libfmt-dev \
        libeigen3-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 第二层：OpenVINO 2024.6.0（CMakeLists.txt 里是 REQUIRED，不是可选项）
#
# 用 Intel 官方 apt 源装，比下压缩包省事。装完的 cmake 配置在
#   /usr/lib/x86_64-linux-gnu/cmake/openvino2024.6.0
# 而 CMakeLists.txt 的默认回退路径是 /opt/intel/openvino_2024.6.0/runtime/cmake，
# 两者不一致 —— 所以下面用 ENV 设置 OpenVINO_DIR。
#
# CMakeLists.txt 第 56 行特意写了 if(NOT DEFINED ENV{OpenVINO_DIR})，
# 就是为了让这个环境变量能生效，不必改动仓库里的文件。
# ---------------------------------------------------------------------------
# 公钥放在 intel-gpg-keys/ 下，不在 openvino/2024/ 里
# （后者返回 403，是常见的抄错地址）。
#
# 包名带版本号后缀，且必须装 -dev 那个：
#   libopenvino-2024.6.0        只有运行库 .so
#   libopenvino-dev-2024.6.0    含头文件和 CMake 配置，find_package 要靠它
# 运行时插件要单独装，缺了会编译通过但一运行就抛 ov::Exception：
#   intel-cpu-plugin   demo.yaml 里 device: CPU
#   ir-frontend        模型是 .xml/.bin（IR 格式）
#   onnx-frontend      classify_model 是 tiny_resnet.onnx
#   auto-plugin        classifier.cpp:13 硬编码 compile_model(..., "AUTO", ...)，
#                      缺它报 'Device with "AUTO" name is not registered'
#   auto-batch-plugin  AUTO 在内部会用到，一并装上
RUN curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
      | gpg --dearmor -o /usr/share/keyrings/intel-openvino.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/intel-openvino.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu22 main" \
      > /etc/apt/sources.list.d/intel-openvino-2024.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        libopenvino-dev-2024.6.0 \
        libopenvino-intel-cpu-plugin-2024.6.0 \
        libopenvino-ir-frontend-2024.6.0 \
        libopenvino-onnx-frontend-2024.6.0 \
        libopenvino-auto-plugin-2024.6.0 \
        libopenvino-auto-batch-plugin-2024.6.0 \
    && rm -rf /var/lib/apt/lists/*

# apt 装出来的 cmake 配置路径和 CMakeLists.txt 第 57 行的默认回退路径
# (/opt/intel/openvino_2024.6.0/runtime/cmake) 不同，所以用环境变量指过去。
# CMakeLists.txt 特意写了 if(NOT DEFINED ENV{OpenVINO_DIR}) 来让这里生效，
# 不必改动仓库里的文件。路径已在容器内实测确认。
ENV OpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0

# ---------------------------------------------------------------------------
# 补一个 plugins.xml —— 这是 apt 装 OpenVINO 特有的坑。
#
# 上面那几个 plugin 包只往 /usr/lib/openvino-2024.6.0/ 放了 .so，
# 却没有附带插件注册表 plugins.xml，而 libopenvino.so 是靠读这个 xml 才知道
# "CPU" / "AUTO" 这些设备名对应哪个 .so。缺了它的表现是：
#   编译链接全部正常，一运行就抛 ov::Exception
#   'Device with "AUTO" name is not registered in the OpenVINO Runtime'
# 看起来像没装插件，其实 .so 就在那儿，只是没注册。
#
# 位置必须是 /usr/lib/plugins.xml：运行时按 libopenvino.so 自身所在目录
# （即 /usr/lib）去找同级的 plugins.xml。
#
# MULTI 也指向 auto_plugin —— 这两个设备名由同一个 .so 实现。
# ---------------------------------------------------------------------------
RUN printf '%s\n' \
      '<ie>' \
      '  <plugins>' \
      '    <plugin name="CPU" location="/usr/lib/openvino-2024.6.0/libopenvino_intel_cpu_plugin.so"></plugin>' \
      '    <plugin name="AUTO" location="/usr/lib/openvino-2024.6.0/libopenvino_auto_plugin.so"></plugin>' \
      '    <plugin name="MULTI" location="/usr/lib/openvino-2024.6.0/libopenvino_auto_plugin.so"></plugin>' \
      '    <plugin name="BATCH" location="/usr/lib/openvino-2024.6.0/libopenvino_auto_batch_plugin.so"></plugin>' \
      '  </plugins>' \
      '</ie>' \
      > /usr/lib/plugins.xml

# ---------------------------------------------------------------------------
# 第三层：图形显示
#
# auto_aim_test 会调 cv::imshow 弹窗，容器里没有显示器。
# 这里用 Xvfb（虚拟屏幕）+ x11vnc（把屏幕内容变成 VNC 流）+ noVNC（网页版 VNC 客户端），
# 于是在 Mac 上用浏览器打开 http://localhost:6080 就能看画面，
# 不必安装 XQuartz，也不用配 X11 转发。
#
# 另外装 libgl1 / libglib2.0-0：OpenCV 的 highgui 运行时依赖，
# 少了会在 imshow 时报 "libGL.so.1: cannot open shared object file"。
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb \
        x11vnc \
        novnc \
        websockify \
        libgl1 \
        libglib2.0-0 \
        x11-utils \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 第四层：libusb
# io/hikrobot/hikrobot.cpp 直接 #include <libusb-1.0/libusb.h>，用它复位
# 相机的 USB 设备；io/CMakeLists.txt 也链接了 usb-1.0。
# 缺了会在编译 hikrobot.cpp 时报 "libusb.h: No such file or directory"。
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

COPY make-daheng-stub.sh /usr/local/bin/make-daheng-stub.sh
RUN chmod +x /usr/local/bin/make-daheng-stub.sh

# OpenCV 在容器里默认可能选 Wayland 后端而渲染不出画面（和 WSLg 上同一个坑，
# 见 怎么编译运行.md「窗口显示不出来」一节），强制走 X11 + 软件渲染。
ENV DISPLAY=:99
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_X11_NO_MITSHM=1

# ---------------------------------------------------------------------------
# 入口脚本：拉起虚拟屏幕和 noVNC，再把控制权交给用户命令
# ---------------------------------------------------------------------------
COPY docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# 源码通过 -v 挂载进来（build.sh 里做），不 COPY 进镜像：
# 这样在 Mac 上改代码，容器里立刻可见，不必重新 build 镜像。
WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]
