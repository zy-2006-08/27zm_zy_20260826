# 自瞄架构图：来源与验证

## 产物与范围

- 图：`/Users/zy/RM27_zimiao/27zm_zy_20260831/pages/archify-auto-aim/index.html`
- 可编辑规格：同目录 `architecture.json`，11 个组件、11 条关系、3 张说明卡，`meta.locale: zh-CN`。
- 使用 tt-a1i/archify 官方 architecture renderer 生成，未手写替代 HTML、未修改模板或业务代码。
- 保持默认 classic 风格与静态展示；沿 ①→⑦ 读主链路，分类/PnP 为调用分支。配置、调试、反馈的次要扇出放卡片，不代表不存在这些依赖。
- 本图依据当前本地源码，不是运行中二进制或 STM32 固件的验收报告；没有伪造公开仓库 URL 或 revision。

## 官方工具来源

- 已读官方说明：https://raw.githubusercontent.com/tt-a1i/archify/main/archify/SKILL.md （version 2.17）。
- 本轮官方归档：https://codeload.github.com/tt-a1i/archify/tar.gz/refs/heads/main
- 下载日期：2026-09-20。归档 SHA-256：`6ed06f76e6d0c075fabf22ac52e3d3a7e330f05e9633ed0c4322ec98de64a227`。
- 本轮包位置：`/var/folders/pg/x951cm011zbfh6j4rn58cm6h0000gn/T/opencode/archify-resume-official/archify`。
- 编写前已读 `schemas/architecture.schema.json`、`schemas/common.schema.json`、`examples/web-app.architecture.json`；交付遵循 `references/delivery-contract.md`。
- 系统 PATH 无 Node，本轮复用批准的临时目录中已有 Node v22.16.0 darwin-arm64，未全局安装。交互测试依赖 playwright-core 只安装在临时目录 `archify-resume-official`。
- 全部 Archify 命令设置 `ARCHIFY_UPDATE_CHECK_DISABLED=1`；checker 返回 `status: silent, reason: disabled`。

## 本地源码证据

以下路径均相对 `/Users/zy/RM27_zimiao/27zm_zy_20260831`；行号来自本轮只读核对，不保证未来修改后仍相同。

| 图中组件/结论 | 源码位置 | 核对结果 |
|---|---|---|
| 活动入口 | `CMakeLists.txt:91-99` | rb_auto_aim_debug 与离线 auto_aim_test 是不同目标。 |
| 启动配置 | `src/rb_auto_aim_debug.cpp:29-61` | 位置参数默认 `../configs/sb_long.yaml`，用于构造相机、检测、跟踪、解算、规划和串口等对象。不能套用 onboarding 中 calibration.yaml 唯一生效的泛化描述。 |
| 相机 | `io/camera.cpp:34-74,79-140,149-161` | 按 YAML 选择驱动，read 获取图像/时间戳；auto 模式探测海康再大恒。 |
| 图像主循环 | `src/rb_auto_aim_debug.cpp:187-199` | 采图、读取相应姿态、Detector.detect、Tracker.track；注释中“检测内部调用 Solver”不等于真实调用关系。 |
| 检测 | `tasks/auto_aim/detector.cpp:49-180` | 灰度阈值、轮廓、灯条筛选/配对、装甲板筛选、分类、去重；122-124 调 Classifier。 |
| 数字分类 | `tasks/auto_aim/classifier.cpp:7-59,62-96` | ONNX 由 OpenCV DNN 加载并 forward；另有 OpenVINO 构造/备选方法，活动调用不是 ovclassify。 |
| 跟踪 | `tasks/auto_aim/tracker.cpp:126-229,389-448` | 使用模式/颜色反馈；初始化和更新目标时调用 solver_->solve，预测并更新状态。非 lost 状态可返回目标，不能简化成仅 tracking/temp_lost。 |
| 空间解算 | `tasks/auto_aim/solver.cpp:48-90,101-139` | 读取内外参、SOLVEPNP_IPPE、相机/云台/世界坐标变换和朝向处理。main 187-193 给姿态 `gimbal.q(t - 3ms)`。 |
| 最新目标邮箱 | `tools/thread_safe_queue.hpp:12-36,84-91`; `src/rb_auto_aim_debug.cpp:65-66,95,224-226,289-292` | 容量 1；满时覆盖旧值；front 加锁复制且不弹出；写首个目标或 nullopt。最新发布不等于永远新鲜：相机停滞时无过期机制。 |
| 规划 | `tasks/auto_aim/planner/planner.hpp:49-67`; `tasks/auto_aim/planner/planner.cpp:331-411,462-467` | 空目标返回不可控；预测到当前时刻/延迟，弹道飞行时间预测、yaw/pitch MPC、开火判断。 |
| 独立规划循环 | `src/rb_auto_aim_debug.cpp:92-122,172-174` | mailbox → planner → send；读取 yaw/弹速，计算/发送/绘图之后 sleep 5 ms，不保证固定 200 Hz。 |
| 串口发送 | `io/gimbal/gimbal.cpp:18-31,102-125,179-182`; `io/gimbal/gimbal.hpp:80-96` | 波特率硬编码 460800，串口设备来自 YAML。TX 29 字节：0x66、mode、6 个 float、CRC16、0x11；mode=0/1/2 表示无控制/控制/控制并开火。target_x/y/name 参数被忽略、不上线路。 |
| STM32 回传 | `io/gimbal/gimbal.hpp:19-56`; `io/gimbal/gimbal.cpp:39-42,63-97,217-275` | 主机期望 RX 29 字节：5A 53、mode/color、四元数 wxyz、弹速/计数、CRC16；接收线程校验并更新反馈。固件实际接受条件与执行逻辑未独立核验。 |
| 调试 | `src/rb_auto_aim_debug.cpp:124-174,202-206,294-325`; `tasks/auto_aim/detector.cpp:64-67` | GUI、重投影、按键、FPS、JSON 曲线。GimbalState 中速度/q2yaw 未由 RX 填写，不能把相应曲线宣称为有效实测反馈。 |
| YOLO 范围 | `src/rb_auto_aim_debug.cpp:53-54,198-199`; `tests/auto_aim_test.cpp:19-24,43-55,71-94` | 活动入口构造 YOLO 但逐帧用 Detector；离线测试走 YOLO → tracker.test_track → Aimer，不画进活动链路。不声称其他工具没有 YOLO。 |

## 复现命令

在官方包目录作为 working directory 执行，以下变量不会改全局配置：

```sh
export PATH="/var/folders/pg/x951cm011zbfh6j4rn58cm6h0000gn/T/opencode/node-v22.16.0-darwin-arm64/bin:$PATH"
export ARCHIFY_UPDATE_CHECK_DISABLED=1
export ARCHIFY_CHROME="/Users/zy/Library/Caches/ms-playwright/chromium-1148/chrome-mac/Chromium.app/Contents/MacOS/Chromium"
OUT="/Users/zy/RM27_zimiao/27zm_zy_20260831/pages/archify-auto-aim"
node scripts/check-update.mjs
node bin/archify.mjs doctor
node bin/archify.mjs validate architecture "$OUT/architecture.json" --quality showcase --json
node bin/archify.mjs deliver architecture "$OUT/architecture.json" "$OUT/index.html" --quality showcase --json
node bin/archify.mjs visual-check "$OUT/index.html" --json
```

doctor 全部就绪；最终 validate/deliver/visual-check 均退出 0。HTML 由 deliver 原子生成，未再编辑。

## 交付收据

| 项目 | 结果 |
|---|---|
| diagram_type | architecture |
| specification SHA-256 | `82d0a5673a49457f1a6235e89821cd05b5129398b7141e4dcc6d6dd995892050` |
| specification bytes | 4402 |
| artifact SHA-256 | `f5ff8a53924cfb1bca5ab94498070530862df0900d93746a404b40f892ad9e79` |
| artifact bytes | 811731 |
| validation | 9/9 showcase；0 errors，0 warnings |
| browser_evidence | passed；完整机器收据 `index.visual-check.json` |
| visual_review | passed（规定桌面尺寸）；附加 768px 平板检查发现导航遮挡，不能声称全尺寸视觉通过 |
| correction_rounds | 本轮 0：复用通过验证的 JSON，未修改官方 HTML 或布局 |

本轮首次 visual-check 未自动找到 Chromium（exit 2），指定已有可执行文件后完成检查。最终 `scrollWidth/scrollHeight` 分别为 1440/900、1600/1000、1920/1080、2048/1320，四档无溢出；导航无遮挡。

## 浏览器与截图

- 官方截图：`index.visual-check.1440x900.light.png`、`index.visual-check.1440x900.dark.png`、`index.visual-check.2048x1320.light.png`、`index.visual-check.2048x1320.dark.png`；同目录 `index.visual-check.html` 为联系表。
- 已直接读图检查布局、中文、卡片与箭头。主路径连续，调用分支和 TX/RX 可辨，无节点/标签遮挡；大型桌面图与卡片纵向平衡。
- 独立复核使用只读 explore：`bg_7811dda3` 实际查看桌面浅/深色及搜索/聚焦截图，结论 PASS，注明副标题偏小。`bg_bca9eacd` 核验全部九张截图、真实 SVG、官方视觉系统、哈希和交互，桌面项通过，但对附加 768px 截图提出 REVISE。机器收据保留官方 `visualReview: pending`，不篡改它来冒充机器视觉批准。
- 小屏桌面默认节点副标题较细小：官方测得最小投影字号 7.875px（1440）/8.458px（1600）/9px（其余），超过工具 6px 门槛，但不等于通用无障碍大字标准；读细节建议内置放大或浏览器放大。
- Playwright skill/MCP 已尝试，但 MCP 宿主找不到 npx；未修改 MCP 配置。改用临时目录 playwright-core 驱动同一本机 Chromium，打开准确的 file URL。
- 本轮实际交互：浅/深色切换并恢复；Tracker 聚焦及关闭；输入中文“跟踪”并回车选中 Tracker；放大改变 data-view-scale、保持原始 viewBox。`browser-check.json` 保存绑定 HTML SHA-256 的通过收据，pageErrors 为空；`browser-check.mjs` 是可复现脚本。
- 附加截图：`interaction-search.png`、`interaction-focus.png`、`responsive-375.png`、`responsive-768.png`、`responsive-1280.png`。375/768/1280 宽度均无水平溢出；手机细字需使用放大。不声称穷尽导出、剪贴板、演示、全部路径/透镜功能。
- 平板限制：等待字体加载和 1500ms 稳定布局后重新拍摄，768px 下导航仍覆盖下排串口/规划节点底部及 RX 区域。这是官方查看器在该额外尺寸的可见限制，未通过改模板、隐藏控件或伪造截图消除。交互收据的 responsive 项仅证明页面无水平溢出，不证明无遮挡；375px 使用官方内部横向滚动。推荐以已通过官方检查的 1440px 及以上桌面窗口阅读。全尺寸视觉验收不宣称通过。
- browser-check.mjs 的 LSP 因 typescript-language-server 未安装不可用；未安装或改全局配置。`node --check` 与实际浏览器脚本执行通过；不宣称 LSP 零错误。规格由官方 schema/渲染检查验证。
- 无应用构建、无相机/串口运行、无固件测试；无仓库外发、全局安装、配置修改或提交。仅新增本输出目录内容。
