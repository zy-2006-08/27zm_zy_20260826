# sb_long.yaml 参数说明

这份文件是上场主程序 `rb_auto_aim_debug` 默认读的配置。下面的数值是 2026-10-30 的。

## 一眼看懂：137 个参数分 5 档

| 档 | 个数 | 什么时候动 |
|---|---|---|
| ① 常调 | 14 | 每次调车都可能动，必须弄懂 |
| ② 有问题才调 | 43 | 出了对应的现象再看 |
| ③ 装车或换场地时调一次 | 34 | 换相机、换镜头、换场地、换电脑才动 |
| ④ 不起作用，但留着 | 16 | 行尾标了【不起作用】。别改，也别删 |
| ⑤ 已注释 | 30 | 行首是 `#【未用】`。主程序用不到 |

**yaml 里的两种标记**

- 行首 `#【未用】`：主程序用不到，已经注释掉，程序效果完全不变。要放开就删掉行首的 `#【未用】`。
- 行尾【不起作用】：现在不起作用，但程序启动时要读，注释掉会直接退出；或者注释掉以后会变成别的值。所以留着。

---

## ① 常调（14 个）

| 环节 | 参数（现在的值） | 管什么 | 调大 | 调小 |
|---|---|---|---|---|
| 识别 | `enemy_color`（"auto"） | 打哪个颜色。"auto" 跟电控发来的颜色走，自己测试时可以写死 "red" 或 "blue" | 不是旋钮 | 识别率突然变 0，先查它 |
| 识别 | `min_confidence`（0.8） | 数字分类器要多有把握才算一块板，0~1 | 认错的少，真板也容易漏 | 认出的多，认错的也多 |
| 跟踪 | `min_detect_count`（5） | 连续认出几帧才开始跟。约 70 帧/秒，5 帧约 0.07 秒 | 不容易被误识别骗，锁定慢半拍 | 锁得快，对着反光也可能开始跟 |
| 跟踪 | `max_temp_lost_count`（11） | 连续丢几帧还不放弃，11 帧约 0.15 秒 | 板转到背面也不丢；目标真走了，还会对着空气预测一阵 | 丢得快，打小陀螺时反复重来，跟踪稳不下来 |
| 瞄准角度 | `yaw_offset`（-0.45） | 左右固定偏多少度 | 往左 | 往右 |
| 瞄准角度 | `pitch_offset`（1.2） | 上下固定偏多少度 | 往上 | 往下 |
| 开火时机 | `shoot_latency_near`（0.055）<br>`shoot_latency_far`（0.045） | 开火提前多少秒。2 米用 near，4 米用 far，中间按距离折算。这两个距离由 `shoot_latency_near_dist`（2.0）和 `shoot_latency_far_dist`（4.0）定 | 开火提前 | 开火推后 |
| 打法切换 | `spin_enter_high`（1.5）<br>`spin_exit_high`（0.3） | 敌方自转多快（rad/s）就换打法。超过 1.5，换成"枪口停在车中间，等板转过来再打"；低于 0.3，换回"枪口跟着板转"。1.5 rad/s 大约 4 秒转一圈 | enter 调大：要转得更快才换 | enter 调小：中速就换。两个数拉得越开，越不容易来回切 |
| 开火窗口 | `small_armor_tolerance`（0.05） | 慢速打法里，落点离小装甲板中心多远以内才开火，单位米。0.05 就是 ±2.5 厘米 | 开火多，打偏的也多 | 打得准，开火少 |
| 云台 | `max_pitch_acc`（25） | 慢速打法里 pitch 的最大加速度 | 跟得快，但容易震、画面糊（原来是 100，震过） | 稳，但跟得慢。25 是 9 月定的临时值 |

**数量感**

- 1 度在 3 米外大约 5 厘米，小装甲板宽 13.5 厘米。`yaw_offset`、`pitch_offset` 一次改 0.1~0.2 度就够。第一次改先改一点，看落点往哪边走，确认方向。
- 开火时机差 10 毫秒，敌方转 8 rad/s 时，板面大约挪 2 厘米。
- 角度参数用静止靶调，时机参数用小陀螺调，混在一起调会互相抵消。
- 判断开火时机还是角度的问题：靶车顺转、逆转各打一组。偏的方向跟着转向一起换边，是时机问题；怎么转都偏同一边，是角度问题。

---

## ② 有问题才调（43 个）

| 现象 | 先看日志里的 | 参数 |
|---|---|---|
| 角点精修成功率低 | `[YOLOV8] 近2秒角点精修 x/y 成功` | `threshold`、`max_angle_error`、`min_lightbar_ratio`、`max_lightbar_ratio`、`min_lightbar_length` |
| 拨盘"哒……哒哒哒"，开火断断续续 | `FIREPROBE`、`[Gimbal] mode 1 -> 2` | `fire_confirm_time`、`fire_exit_scale`、`fire_switch_settle_time` |
| 慢速打法里换板甩头、追不上板 | `CMDPROBE` | `low_speed_yaw_` 开头的 10 个、`leaving_angle`、`spin_select_tau` |
| 高速小陀螺开火太少、落点偏一侧 | `FIREPROBE` | `high_spin_` 开头的 8 个 |
| 敌方横着跑打不中 | `FIREPROBE` | `trans_fire_speed`、`fire_cmd_gimbal_tol_deg`、`fire_trans_aim_tol_deg`、`fire_trans_hold_scale`、`obs_fit_enable`、`obs_fit_window`、`obs_fit_order`、`obs_fit_max_shift`、`obs_fit_max_dv`、`obs_fit_vel_only` |
| 敌方急刹时乱打 | | `fire_brake_decel`、`fire_brake_hold` |
| 慢速打法里 yaw 震或跟不上 | `CMDPROBE` 的 `cmd_y` 和 `gs_y` | `max_yaw_acc` |
| 打大装甲板 | | `big_armor_tolerance` |
| 打比云台高 0.6 米以上的目标 | | `far_pitch_offset`。名字里带 far，但代码判断的是高度，不是距离 |

---

## ③ 装车或换场地时调一次（34 个）

| 什么时候动 | 参数 | 说明 |
|---|---|---|
| 换场地、光照变了 | `exposure_us`（3500）、`gain`（0.75）、`gamma`（0.1，大恒相机才用）、`img_gamma`（1） | 先看 `[识别率]`，不够再调。用 `camera_tuning` 现场调，按 p 存下的值要手动抄进这份 yaml。自瞄要的是灯条亮、背景黑，画面偏暗是正常的 |
| 换了相机、串口、云台 | `camera_name`、`camera_sn`、`flip`、`mirror`、`com_port`、`gimbal_y1`、`gimbal_p2`、`gimbal_r3`、`R_gimbal2imubody` | `flip` 和 `mirror` 必须一起改 |
| 换镜头、动过相机支架 | `camera_matrix`、`distort_coeffs`、`R_camera2gimbal`、`t_camera2gimbal` | 标定程序算出来的，别手改。换镜头要重标前两个，动过支架要重标后两个 |
| 换模型、换电脑 | `yolo_name`、`yolov8_model_path`、`classify_model`、`device`、`use_traditional`、`use_roi` | `use_traditional` 保持 true，角点精修对测距很重要。同一份 yaml，x86 电脑走 OpenVINO，Jetson 走 TensorRT |
| 别动（成套调好的） | `laser_mode`、`laser_pitch_mode`、`laser_latency` | `laser_mode` 一关，开火时机差 50 毫秒，`shoot_latency` 就要重调 |
| 别动（控制器内部权重） | `Q_yaw`、`R_yaw`、`Q_pitch`、`R_pitch` | |
| 别动 | `target_h_error`（0.4） | 改了可能误触发"弹道无解" |
| 打前哨站、基地才用 | `outpost_max_temp_lost_count`、`tower_and_base_armor_tolerance_`、`tower_pitch_prediction_time` | |

---

## ④ 不起作用，但留着（16 个，行尾标了【不起作用】）

| 参数 | 为什么不起作用 | 为什么不能注释 |
|---|---|---|
| `min_armor_ratio`、`max_armor_ratio`、`max_side_ratio`、`max_rectangular_error` | 只有整套传统识别才用，主程序走神经网络 | 程序启动要读，缺了会崩 |
| `roi` | `use_roi` 是 false | 程序启动要读 |
| `comming_angle` | 主程序读的是另一个名字 `low_speed_coming_angle`，yaml 里没写，用默认值 60 | 离线测试 `auto_aim_test` 要读 |
| `decision_speed` | 下面两个延迟都是 0，它只是在两个 0 里挑一个 | 程序启动要读 |
| `high_speed_delay_time`、`low_speed_delay_time` | 值是 0 | 程序启动要读 |
| `fire_thresh` | 只有主程序没在用的另外两套打法用 | 程序启动要读 |
| `target_dist_error` | 值是 0 | 程序启动要读 |
| `gimbal_control_delay` | `laser_mode: true` 时开火判断不用它 | 程序启动要读 |
| `laser_aim_lead` | `laser_pitch_mode: 1` 时不用 | 注释掉会变成 0 |
| `shoot_latency` | `shoot_latency_near/far` 四项都在，它被盖住 | 删掉那四项时才用它 |
| `trans_armor_tolerance` | 被 `fire_trans_aim_tol_deg`（平移开火第四版）盖住 | 注释掉会变成 0 |
| `fire_gimbal_lead` | 被 `fire_cmd_gimbal_tol_deg` 盖住 | 注释掉会变成 0 |

---

## ⑤ 已注释（30 个，行首是 `#【未用】`）

| 参数 | 为什么注释 |
|---|---|
| `pattern_cols`、`pattern_rows`、`square_size_mm` | 标定板尺寸，只有标定程序读，它们用的是 calibration.yaml |
| `yolo11_model_path`、`yolov5_model_path` | `yolo_name` 是 yolov8，用不到。换模型时放开对应那行 |
| `number_classifier`（整段） | 只有另一套识别器 rv_detector 读，主程序没用它。数字分类实际用的是 `classify_model` |
| `first_tolerance`、`second_tolerance`、`judge_distance`、`auto_fire` | 主程序没用 Shooter 模块。**`auto_fire` 改成 false 也不会停火** |
| `vid_pid` | 没有任何代码读它 |
| `quaternion_canid`、`bullet_speed_canid`、`send_canid`、`can_interface` | CAN 通信板才用，现在和电控走串口 |
| `spin_high_settle_time`、`spin_vyaw_settle_time`、`spin_vyaw_stable_band` | 两个等待时间是 0，等于关着。要打开，3 行一起放开 |
| `fire_max_duration`、`fire_center_ratio` | 值和代码默认值一样（0 / 1），等于关着 |
| `fire_decel_ratio`、`fire_decel_window` | ratio 是 0，等于关着，两个值都和默认值一样 |
| `trans_accel_` 开头的 5 个 | gain 是 0，等于关着，5 个值都和默认值一样。要打开，5 行一起放开 |
| `obs_fit_smooth_tau`、`obs_fit_blend`、`obs_fit_fast_thr` | 值和代码默认值一样（0 / 1.0 / 0） |

---

## 调参顺序

按流水线从前往后调。前面没调好，后面调了也白调。

1. 上车前确认：相机能出图，`enemy_color` 对，日志里 `[识别率]` 正常。识别率不正常，先查 ③ 里的相机参数
2. 识别和跟踪：`min_confidence`、`min_detect_count`、`max_temp_lost_count`
3. 用静止靶调角度：`yaw_offset`、`pitch_offset`
4. 用小陀螺调时机：`shoot_latency_near`、`shoot_latency_far`
5. 打法和开火窗口：`spin_enter_high`、`spin_exit_high`、`small_armor_tolerance`
6. 还有具体问题，再去查 ②

每次只改一个参数，记下改前、改后的值和落点。改一个不认识的参数前，先在代码里搜一下它的名字，确认它真的在起作用。

---

## 这次注释是怎么验证的

- 每个参数都在代码里查过谁读、怎么读。用 `tools::read` 读的，缺了程序直接退出；写成 `yaml["x"] ? ... : 默认值` 的，缺了就用默认值。
- 主程序用到的模块里，必须有的 72 个参数都还在。
- 用 `replay_dump` 拿 10 份实车日志（9-27 到 10-06），改前、改后各跑一遍。103 万个规划周期里，输出的 yaw、pitch、开火信号逐行完全一样。
- 用 `auto_aim_test` 离线跑 300 帧视频：识别、解算、跟踪模块用新 yaml 都能正常启动。
- 没验证到的：相机和串口，因为验证用的电脑没接这两样。这两块的参数这次一个都没动，只注释了没人读的 `vid_pid`。

## 怎么恢复

- 放开一行：删掉行首的 `#【未用】`。标了"一起放开"的整组一起放开。
- 看这次改了哪些行：`git diff configs/sb_long.yaml`
