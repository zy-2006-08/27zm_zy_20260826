# 步兵装甲板自瞄

新手向说明。照着从上往下做，命令都可以直接复制粘贴。

## 1. 这个仓库是什么

一份精简过的 RoboMaster 步兵**装甲板自瞄**代码。做的事情只有一条链路：

工业相机取图 → 神经网络找装甲板 → PnP 解算出位置 → 卡尔曼滤波跟踪 → 弹道解算 → 算出 yaw / pitch 和开火指令 → 串口发给电控 C 板。

上场跑的只有一个程序：`rb_auto_aim_debug`。其余 8 个可执行文件都是围着它转的工具（离线测试、相机标定、视频抽帧）。

范围界定，看清楚可以省很多时间：

- 只做装甲板自瞄，不含能量机关相关功能；
- 只做单目，不含双目；
- 不含哨兵全向感知，也不含英雄、无人机的专用逻辑；
- 不依赖任何机器人中间件框架，就是一个普通的 CMake 工程。

推理后端固定用 OpenVINO（Intel 的 CPU / iGPU 推理库），权重在 `assets/` 下已经放好。

## 2. 只能在 Linux 上编译

x86_64（各种 NUC）或 aarch64（Jetson Orin Nano 之类）。Windows 上编译不过去，不是配置问题，是三条硬性原因：

1. **相机 SDK 只有 Linux 的 `.so`。** 海康的 `libMvCameraControl.so` 放在 `io/hikrobot/lib/amd64` 和 `io/hikrobot/lib/arm64`，都是 ELF 动态库；大恒的 `libgxiapi.so` 要装系统级 SDK，同样只有 Linux 版。
2. **代码里直接用了 Linux 内核头文件。** `io/cboard.hpp` 包含 `io/socketcan.hpp`，后者 `#include <linux/can.h>`；`tools/plotter.cpp` 用的是 POSIX socket（`<arpa/inet.h>`、`<sys/socket.h>`、`<unistd.h>`）。
3. **CPU 架构是硬编码检查的。** `io/CMakeLists.txt` 只认 `x86_64` 和 `aarch64`，其它架构直接 `FATAL_ERROR` 停下，不会给你留一堆看不懂的链接错误。

另外 OpenVINO 默认按装在 `/opt/intel` 下找，这个路径也是 Linux 的习惯。

系统建议 Ubuntu 22.04。24.04 也能用，但第 3 节的 OpenVINO apt 源要注意。

## 3. 依赖安装

### 3.1 apt 能装的部分

先看一个坑：**Jetson 系列不要执行下面这条命令里的 `libopencv-dev`**。Jetson 的镜像自带了带 CUDA 的 OpenCV，apt 装一遍会把它覆盖掉。Jetson 上请把 `libopencv-dev` 从下面的列表里删掉再执行，用系统自带的 OpenCV，或者自己手动编译。

x86 小电脑直接整段复制：

```bash
sudo apt update
sudo apt install -y \
    git \
    g++ \
    cmake \
    gdb \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    libusb-1.0-0-dev \
    screen
```

每个包对应什么，别删着玩：

| 包 | 用途 |
| --- | --- |
| `git` | 拉代码 |
| `g++` | 编译器，需要支持 C++17 |
| `cmake` | 构建工具，版本要求 >= 3.16.3 |
| `gdb` | VS Code 按 F5 调试要用，`.vscode/launch.json` 指名了它 |
| `libopencv-dev` | 图像处理，全工程到处在用 |
| `libfmt-dev` | 字符串格式化 `fmt::format` |
| `libeigen3-dev` | 矩阵运算 `Eigen/Dense` |
| `libspdlog-dev` | 日志 |
| `libyaml-cpp-dev` | 读 `configs/*.yaml` |
| `nlohmann-json3-dev` | 曲线图工具发的 JSON 报文 |
| `libusb-1.0-0-dev` | 海康相机复位 USB 设备用 |
| `screen` | 上场时把程序挂在后台，断开 SSH 也不会被杀掉 |

### 3.2 大恒 Galaxy SDK（apt 装不了，必须手装）

链接库 `libgxiapi.so` **不在本仓库里**。`io/daheng/` 下只有源码和厂商头文件，没有 `lib/` 目录，所以这个 `.so` 只能靠系统级安装提供。

去大恒官网 <https://www.daheng-imaging.com/>，下载中心 -> 软件下载 -> `Galaxy_Linux_CN-EN_32bits/64bits`，解压 tar.gz，然后：

```bash
cd Galaxy_camera
sudo ./Galaxy_camera.run
```

（解压出来的目录名各版本略有不同，`cd` 到含 `Galaxy_camera.run` 的那一层就行。）

两点要记住：

- **不装的失败方式特别坑。** cmake 配置能过，每个 `.cpp` 都能编过，只有最后一步链接会报 `cannot find -lgxiapi`。也就是忙活十来分钟，在最后一秒才失败。
- **哪怕你用的是海康相机，这个也躲不掉。** `io/camera.cpp` 里 `DahengCamera::initSDK()` 是无条件调用的，YAML 里选哪个品牌都要链上它。海康那边相反，`.so` 已经随仓库放好，不用额外装。

### 3.3 OpenVINO 2024.6.0

装法二选一，版本号务必对上 `2024.6.0`。

**方式 A：apt 源（网络要给力）**

```bash
curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | sudo gpg --dearmor -o /usr/share/keyrings/intel-openvino-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/intel-openvino-archive-keyring.gpg] https://apt.repos.intel.com/openvino ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino.list
sudo apt update
sudo apt install -y openvino-2024.6.0
```

注意上面那行 apt 源里写死了 `ubuntu22`。这是 Ubuntu 22.04 (jammy) 的源，你在 22.04 上用没问题。**如果你的系统是 24.04，这个源里没有对应的包，`apt install` 会告诉你找不到，等于什么都没装**，请改用方式 B。

apt 装完，cmake 配置文件落在 `/usr/lib/cmake/openvino2024.6.0`，所以要加两个环境变量（写进 `~/.bashrc` 免得每次都设）：

```bash
export OpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0
export LD_LIBRARY_PATH=/usr/lib/openvino-2024.6.0:$LD_LIBRARY_PATH
```

**方式 B：官网压缩包**

按官方文档做：<https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html>
解压到 `/opt/intel/openvino_2024.6.0`。这条路是本工程的**默认路径**，什么环境变量都不用设。

## 4. 编译

在仓库根目录：

```bash
bash make.sh
```

`make.sh` 就干两件事，你也可以手敲：

```bash
cmake -B build
make -C build/ -j$(nproc)
```

`-j$(nproc)` 是按 CPU 核数并行编译。小电脑内存小（4G 及以下）容易被 OOM killer 杀掉，那就换成 `-j2`。

默认构建类型是 Release。自瞄吃帧率，Debug 会慢好几倍，想调试再显式指定：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
```

**cmake 找不到 OpenVINO 就手动指路**，两种写法任选：

```bash
export OpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0
cmake -B build
```

```bash
cmake -B build -DOpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0
```

工程里做了 `if(NOT DEFINED ...)` 的保护，你设了就用你的，没设才回退到 `/opt/intel/openvino_2024.6.0/runtime/cmake`。

VS Code 用户：`make.sh` 也是默认构建任务，按 F5 会先构建再启动调试，工作目录已经设成 `build/`。

## 5. 运行

### 先记住这一条

**所有程序都必须在 `build/` 目录里运行。**

代码里的配置和素材路径全是 `../` 开头的相对路径（比如 `../configs/sb_long.yaml`、`../assets/demo/demo`）。在仓库根目录敲 `./build/rb_auto_aim_debug` 会因为找不到 yaml 而失败。正确姿势：

```bash
cd build
./rb_auto_aim_debug ../configs/sb_long.yaml
```

配置文件路径是位置参数，不写就用各程序自己的默认值（下面括号里标了）。

### 9 个可执行文件

| 程序 | 干什么 |
| --- | --- |
| `rb_auto_aim_debug` | 上场 / 调试主程序：接相机 + 串口，跑完整自瞄链路并 imshow 显示（默认 `../configs/sb_long.yaml`） |
| `auto_aim_test` | 拿录好的视频离线跑同一条链路，不需要相机和云台（默认 `../configs/demo.yaml`，素材 `../assets/demo/demo`） |
| `capture` | 标定数据采集：遥控云台拍照，同时存下每张图对应的四元数 |
| `calibrate_input` | 标定数据的交互式检查与录入，挑掉角点没识别好的图 |
| `calibrate_camera` | 相机内参标定，读采集好的图片输出内参和畸变系数 |
| `calibrate_handeye` | 手眼标定（标定板位姿固定），算出相机相对云台的安装偏角 |
| `calibrate_robotworld_handeye` | 手眼标定的另一种解法，同时把标定板位置也解出来 |
| `split_video` | 视频抽帧 / 裁剪，按帧下标区间把 avi + txt 切成一段 |
| `test_simple` | 最小相机测试：只开相机连续 imshow，用来确认相机能出图、调焦 |

跑法举例：

```bash
cd build
./auto_aim_test
./auto_aim_test ../assets/demo/demo -c ../configs/demo.yaml
./test_simple
./capture ../configs/calibration.yaml
```

标定程序默认读写 `../assets/img_with_q`，这个目录仓库里没有，第一次用先在仓库根目录建好：

```bash
mkdir -p assets/img_with_q
```

### 两个容易卡住新手的运行前提

**串口权限。** 云台通信走 `/dev/ttyACM*`（配置文件里的 `com_port`）。普通用户默认打不开，报 permission denied。把自己加进 `dialout` 组，然后**注销重登**才生效：

```bash
sudo usermod -aG dialout $USER
```

**看曲线图要装 PlotJuggler。** `tools/plotter.hpp` 把数据打成 JSON，用 UDP 发到 `127.0.0.1:9870`。接收端是 PlotJuggler，它不是 matplotlib，也不需要 Python：

```bash
sudo apt install -y plotjuggler
```

打开 PlotJuggler，选 UDP Server，端口 9870，数据格式选 JSON，再启动自瞄程序就能看到曲线。不装也不影响程序跑，只是没图看。

## 6. WSL 里能做什么，不能做什么

**在 WSL 里只能编译和跑离线视频测试（`auto_aim_test`）。接真相机必须用真机 Ubuntu。**

- **能编译。** WSL2 是真 Linux 内核，`<linux/can.h>` 有，编译链接都没问题（前提是 Galaxy SDK 和 OpenVINO 照第 3 节装好）。
- **能跑 `auto_aim_test`。** 它只读 `assets/demo/` 下的 avi + txt，不碰硬件。但它会 `imshow` 开窗口，所以需要 WSLg（Win11 自带）或者自己装 X server（VcXsrv 之类）。没有图形环境会直接报 X 相关错误。
- **不能接工业相机。** 原生 WSL2 不支持 USB 直通，相机枚举不到。更狠的是 `io/camera.cpp` 里 `initSDK()` 无条件执行，相机这一步过不去就往下走不了。
- **不能接串口 C 板。** 同理，没有 `/dev/ttyACM*`。
- **不能上车。** 上场、标定、调延迟这些必须在小电脑的真机 Ubuntu 上做。

一句话：WSL 用来看代码、编译、跑 demo 视频；调车用真机。

## 7. 目录说明

| 目录 | 内容 |
| --- | --- |
| `src/` | 应用层，上场主程序 `rb_auto_aim_debug.cpp` 就在这儿 |
| `tasks/auto_aim/` | 功能层，自瞄算法本体：识别、解算、跟踪、火控、轨迹规划 |
| `io/` | 硬件抽象层，相机（大恒 / 海康）、云台串口、C 板 CAN 通信 |
| `tools/` | 工具层，数学、卡尔曼滤波、弹道、日志、YAML 解析、曲线图、录像 |
| `calibration/` | 标定程序，相机内参与手眼标定的完整流程 |
| `configs/` | 每台机器人一个 YAML，相机参数、串口、偏移量、滤波与火控参数都在这里 |
| `assets/` | 神经网络权重和 demo 视频素材 |
| `tests/` | 离线测试程序 `auto_aim_test.cpp` |

配置文件现有三个：`configs/sb_long.yaml`（上场用）、`configs/demo.yaml`（离线 demo 用）、`configs/calibration.yaml`（标定用）。换机器人就复制一份改参数，不要直接改别人的。

## 8. 常见报错

**`Could not find a package configuration file provided by "OpenVINO"`**
cmake 阶段就挂了，说明 OpenVINO 的 cmake 目录没找对。apt 装的要设 `export OpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0`；压缩包装的确认解压位置是 `/opt/intel/openvino_2024.6.0`。设完删掉 `build/` 重新 `cmake -B build`，cmake 有缓存，不删可能还是旧结果。

**`cannot find -lgxiapi`**
所有源文件都编过了，最后链接时挂掉。大恒 Galaxy SDK 没装，回到 3.2 节跑一遍 `./Galaxy_camera.run`。装完重新编译。

**`[YAML] xxx not found!` 然后程序立刻退出**
配置文件里缺了 `xxx` 这个 key。`tools/yaml.hpp` 读不到 key 时不抛异常，直接 `exit(1)`，所以看起来像莫名其妙地死掉。照报错里的名字去 YAML 里补上，可以对照 `configs/` 下另一个能跑的配置文件抄。

**yaml 加载失败 / 提示找不到配置文件或视频**
八成是没在 `build/` 目录下运行。所有路径都是 `../` 相对的，先 `cd build` 再跑，见第 5 节。

**`bash: ./make.sh: /bin/bash^M: bad interpreter`**
文件是 Windows 换行符（CRLF），行尾多了个 `\r`。修掉：

```bash
sed -i 's/\r$//' make.sh
```

别指望换个跑法绕过去：`bash make.sh` 不会报这个 interpreter 错，但行尾那个 `\r` 会被当成参数的一部分，于是 `cmake -B build` 变成建一个名字带回车的怪目录，错得更隐蔽。老老实实先 `sed` 一下。同类问题也会出现在其它从 Windows 拷过来的 `.sh` 上，同样办法处理。

**`Unsupported architecture: xxx!`**
cmake 直接报错停下。CPU 不是 x86_64 也不是 aarch64，本工程不支持，见第 2 节。

**相机打不开 / 找不到设备**
先确认相机线插稳、`lsusb` 能看到设备，再核对配置文件里的 `camera_name`（`daheng` 或 `hikrobot`）和 `camera_sn` 跟手上这台相机一致。序列号写错的表现就是找不到相机。

**串口 permission denied**
`dialout` 组没加，或者加了没重新登录，见第 5 节。
