#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

  // enum class WorkMode : uint8_t
  // {
  //   IDLE = 0,             // 空闲
  //   AUTO_AIM = 1,         // 自瞄模式
  //   OMNI_PERCEPTION = 2,  // 全向感知模式
  // };

  // ============================ 接收包：C 板 -> 视觉 ============================
  // 方向：下位机（电控 C 板）发，本程序收。收包逻辑见 gimbal.cpp:244 read_thread()。
  // __attribute__((packed)) 关闭结构体对齐填充，让内存布局和串口字节流一一对应，
  // 于是可以直接 read() 进结构体再按字段取用（gimbal.cpp:260），不用手工拆字节。
  // 电控侧对应概念：就是你在 C 板那边定义的同一个包，两边字段顺序必须逐字节对齐，
  // 少一个字段或换个顺序，收到的就是一堆乱码。
  // sizeof = 37 字节。字节偏移是按 packed 布局实测的，改字段顺序会让下面所有偏移作废。
  struct __attribute__((packed)) GimbalToVision
  {
    uint8_t head[2] = {0x5a, 0x53};  // 偏移 0，2字节：帧头。收包时校验，不对就 flushInput 重新同步（gimbal.cpp:267）
    uint8_t mode;                    // 偏移 2，1字节：0: 空闲, 1: 自瞄, 2: 小符, 3: 大符, 4:开长焦  电控控制右键0，1
                                     //               转成 GimbalMode 枚举，见 gimbal.cpp:321 的 switch
    uint16_t color;                  // 偏移 3，2字节：0: 红色, 1: 蓝色。注意这是"我方"颜色，
                                     //               取反后才是敌方颜色，见 gimbal.cpp:315 enemy_color = !rx_data_.color
    float q[4];                      // 偏移 5，16字节：wxyz顺序。云台姿态四元数，即电控侧陀螺仪的姿态输出。
                                     //               这是整条链路里最关键的输入：solver 靠它把相机坐标转到世界坐标。
                                     //               用法见 gimbal.cpp:292，先转欧拉角再按 yaml 的轴映射重组
    float bullet_speed;              // 偏移 21，4字节：弹速，单位 m/s。弹道解算的输入（trajectory）
    uint16_t bullet_count;           // 偏移 25，2字节：子弹累计发送次数
    float gimbal_yaw;                // 偏移 27，4字节：云台 yaw。注意：本仓库不消费这两个字段，
    float gimbal_pitch;              // 偏移 31，4字节：云台 pitch。yaw/pitch 实际由上面的 q[4] 反算而来
                                     //               （gimbal.cpp:309-311），这两个原始字段读进来就没人再用
    uint16_t crc16;                  // 偏移 35，2字节：CRC16 校验。收包侧是真校验的，见 gimbal.cpp:282
                                     //               check_crc16 失败则整帧丢弃。（对比发送侧：从未计算，见下）
  };

  static_assert(sizeof(GimbalToVision) <= 64);

  // ============================ 发送包 A：视觉 -> C 板（本仓库当前未使用）============================
  // 方向：本程序发，下位机收。填包+写串口见 gimbal.cpp:160 Gimbal::send()。
  // sizeof = 29 字节。
  //
  // 注意：本仓库上场跑的是下面的 sb_VisionToGimbal，不是这个。
  // 全仓库只有 src/rb_auto_aim_debug.cpp:90 和 :293 两个发送点，调的都是 sb_send()。
  //
  // 【三处疑似问题，都在发送侧，读代码时要能自己判断出来】
  // 1. crc16 声明了但从未被赋值：本结构体唯一的 get_crc16 调用在 gimbal.cpp:221-222，
  //    整段被注释掉了。也就是说这个包发出去时 crc16 是未初始化的随机值。
  //    对比接收侧 gimbal.cpp:282 是真校验的——校验只有单向做了。
  // 2. gimbal.cpp:148 和 :171 各有一条 `reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_);`
  //    这是两条遗留的空语句（逗号表达式，算完就丢），大概是删 crc16 计算时留下的残骸。
  // 3. 下面的 sb_VisionToGimbal（实际在用的那个）连 crc16 字段都没有。
  //
  // 为什么不在这里动手改：改它要电控侧固件配合，两边校验算法和包长必须同时改，
  // 离线（没车没 C 板）做不了闭环验证，改了只能是"看起来对"。
  // struct __attribute__((packed)) VisionToGimbal
  // {
  //   uint8_t head = {0x66};  // 偏移 0，1字节：帧头
  //   uint8_t mode = 0;       // 偏移 1，1字节：0: 不控制, 1: 控制云台但不开火，2:控制云台开火
  //                           //              由 send() 的 control/fire 两个 bool 拼出，见 gimbal.cpp:164
  //   float yaw = 0;          // 偏移 2，4字节：目标 yaw 绝对角，单位弧度（世界系，非增量）
  //   float yaw_vel = 0;      // 偏移 6，4字节：yaw 角速度前馈，单位 rad/s
  //   float yaw_acc = 0;      // 偏移 10，4字节：yaw 角加速度前馈，单位 rad/s^2
  //   float pitch = 0;        // 偏移 14，4字节：目标 pitch 绝对角，单位弧度
  //   float pitch_vel = 0;    // 偏移 18，4字节：pitch 角速度前馈，单位 rad/s
  //   float pitch_acc = 0;    // 偏移 22，4字节：pitch 角加速度前馈，单位 rad/s^2
  //                           //              带速度/加速度是为了让电控侧做前馈跟随，不是只给个位置让它自己追
  //   uint16_t crc16;         // 偏移 26，2字节：见上方第 1 条——从未被计算
  //   uint8_t end = {0x11};   // 偏移 28，1字节：帧尾
  // };

  // ============================ 发送包 B：视觉 -> C 板（★实际在用的就是这个）============================
  // 方向：本程序发，下位机收。填包+写串口见 gimbal.cpp:183 Gimbal::sb_send()。
  // sizeof = 36 字节。
  //
  // 与上面 VisionToGimbal 的差别：前 26 字节完全一样，之后把 crc16 换成了
  // target_x / target_y / target_name（多发一份目标信息给电控/雷达侧用）。
  // 也就是说这个包**没有任何校验字段**，只靠 head(0x66) + end(0x11) 兜着。
  //
  // 上场路径：Planner(MPC) 产出 auto_aim::Plan -> rb_auto_aim_debug.cpp:90 -> sb_send()。
  // 注意 Plan 里的 yaw/pitch 是弧度：rb_auto_aim_debug.cpp:293 那个收尾调用就明确
  // 把 state 的角度值 /57.3 转回弧度再发，可以对照着确认单位。
  struct __attribute__((packed)) sb_VisionToGimbal
  {
    uint8_t head = {0x66};    // 偏移 0，1字节：帧头
    uint8_t mode = 0;         // 偏移 1，1字节：0: 不控制, 1: 控制云台但不开火，2:控制云台开火
    float yaw = 0;            // 偏移 2，4字节：目标 yaw 绝对角，单位弧度
    float yaw_vel = 0;        // 偏移 6，4字节：yaw 角速度前馈，单位 rad/s
    float yaw_acc = 0;        // 偏移 10，4字节：yaw 角加速度前馈，单位 rad/s^2
    float pitch = 0;          // 偏移 14，4字节：目标 pitch 绝对角，单位弧度
    float pitch_vel = 0;      // 偏移 18，4字节：pitch 角速度前馈，单位 rad/s
    float pitch_acc = 0;      // 偏移 22，4字节：pitch 角加速度前馈，单位 rad/s^2
    float target_x = 0;       // 偏移 26，4字节：目标（整车旋转中心）世界系 x，单位米
    float target_y = 0;       // 偏移 30，4字节：目标世界系 y，单位米
                              //              取自 EKF 状态的第 0、2 维，见 rb_auto_aim_debug.cpp:83-85
    uint8_t target_name = 0;  // 偏移 34，1字节：目标兵种编号。注意发的是 ArmorName+1（rb_auto_aim_debug.cpp:83），
                              //              0 被留作"无目标"，见 armor.hpp 的 ArmorName
    uint8_t end = {0x11};     // 偏移 35，1字节：帧尾。本包无 crc16，见上方说明
  };

  // static_assert(sizeof(VisionToGimbal) <= 64);

  enum class GimbalMode
  {
    IDLE,              // 空闲
    AUTO_AIM,          // 自瞄
    SMALL_BUFF,        // 小符
    BIG_BUFF,          // 大符
    LONG_FOCAL_LENGTH  //长焦
  };

  // ============================ 云台状态：给算法层用的"已解析"形式 ============================
  // 这不是通信包，是 GimbalToVision 收进来后解析、换算好的内部状态，供上层 state() 取用。
  // 填写位置见 gimbal.cpp:308-317。上层通过 Gimbal::state() 拿到它的快照（加了 mutex）。
  //
  // 单位陷阱（本仓库最容易踩的一个）：这里的 yaw/pitch 是**角度**，
  // 因为 gimbal.cpp:310-311 乘了 57.3（弧度->度）。而发送包里的 yaw/pitch 是**弧度**。
  // 所以 rb_auto_aim_debug.cpp:293 回填时要 /57.3 转回去。算法内部一律用弧度。
  struct GimbalState
  {
    float yaw;              // 云台 yaw，单位【度】（gimbal.cpp:310 已乘 57.3）
    float yaw_vel;          // yaw 角速度。注意：下位机没发速度，本字段无人写入，恒为初值
    float pitch;            // 云台 pitch，单位【度】（gimbal.cpp:311 已乘 57.3）
    float pitch_vel;        // pitch 角速度。同上，无人写入
    float q2yaw;            // 由四元数反算的 yaw。本仓库无写入点，只在 rb_auto_aim_debug.cpp:104 被读去画曲线
    float q2pitch;          // 同上，无写入点。（:105 那行画的其实是 gs.pitch，不是 q2pitch）
    uint8_t mode;           // 直接取自收包的 mode 字段（gimbal.cpp:314）
    uint8_t enemy_color;    // 0: 蓝色, 1: 红色
                            // ★注意这是**敌方**颜色，由我方颜色取反得到：
                            // gimbal.cpp:315 `state_.enemy_color = !rx_data_.color`
                            // 收包里的 color 是我方颜色（0红/1蓝），取反后语义也跟着反过来
    float bullet_speed;     // 弹速，单位 m/s，弹道解算用
    uint16_t bullet_count;  // 子弹累计发射数。上层靠它跳变来判断"是否刚开了一枪"
                            // （rb_auto_aim_debug.cpp:94 的 fired 判断）
  };

  class Gimbal
  {
    public:
    Gimbal(const std::string & config_path);

    ~Gimbal();

    GimbalMode mode() const;
    GimbalState state() const;
    std::string str(GimbalMode mode) const;
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

    void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc);

    void sb_send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc, float target_x, float target_y,uint8_t target_name);

    // void send(io::VisionToGimbal VisionToGimbal);

    void sb_send(io::sb_VisionToGimbal VisionToGimbal);

    GimbalState * set_state_() { return &state_; }

    private:
    serial::Serial serial_;

    std::thread thread_;
    std::atomic<bool> quit_ = false;
    mutable std::mutex mutex_;

    GimbalToVision rx_data_;
    // VisionToGimbal tx_data_;
    sb_VisionToGimbal sb_tx_data_;

    GimbalMode mode_ = GimbalMode::IDLE;
    GimbalState state_;
    tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>> queue_{1000};

    int gimbal_yaw2vision, gimbal_pitch2vision, gimbal_roll2vision;

    bool read(uint8_t * buffer, size_t size);
    void read_thread();
    void reconnect();
  };

}  // namespace io

#endif  // IO__GIMBAL_HPP