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
//                           电控发给算法的包
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

    // ★2026-09 修正: 这里原本在 bullet_count 和 crc16 之间还有两个字段
    //     float gimbal_yaw;    // 偏移 27
    //     float gimbal_pitch;  // 偏移 31
    //   使结构体为 37 字节。但实测电控每帧只发 29 字节 —— 统计相邻帧头
    //   5A 53 的间隔, 4799 次全是 29, 无一例外。
    //
    //   为什么必须改: gimbal.cpp:152 用 sizeof(rx_data_) 读串口。按 37 读时
    //   每帧会多吞 8 字节(即下一帧的前 8 字节), 于是帧头校验失败、
    //   flushInput 丢弃、再读又错位 —— 永远拼不出一个有效帧。
    //   后果是 Gimbal 构造函数末尾的 queue_.pop() 永久阻塞, 凡是要连云台的
    //   程序(calibrate_input / capture / rb_auto_aim_debug)全部卡在启动阶段,
    //   表面现象只是"没反应", 很难看出是协议长度不一致。
    //
    //   为什么可以删: 这两个字段本仓库从未读取(改动前已全工程 grep 确认)。
    //   原注释也写明"yaw/pitch 实际由上面的 q[4] 反算而来(gimbal.cpp:309-311),
    //   这两个原始字段读进来就没人再用"。
    //
    //   ⚠️ 若换了电控固件、对方改回发 37 字节, 这里要同步加回去,
    //      否则会出现同样的错位症状。排查工具: tools/serial_monitor.py
    //      它会报"杂散字节偏多", 并可用 --frame-len 37 对比验证。
    uint16_t crc16;                  // 偏移 27，2字节：CRC16 校验。收包侧是真校验的，见 gimbal.cpp:282
                                     //               check_crc16 失败则整帧丢弃。（对比发送侧：从未计算，见下）
  };

  static_assert(sizeof(GimbalToVision) == 29, "必须与电控实际发送的帧长一致, 见上方说明");
  static_assert(sizeof(GimbalToVision) <= 64);
//                      算法发给电控的包
  // 算法发给电控的包。总长 29 字节, 必须与电控 GetReceive_SP 严格一致。
  //
  // ★2026-09 修正: 原来这里是 36 字节 —— 尾部是
  //     float target_x; float target_y; uint8_t target_name; uint8_t end;
  //   而电控 RM2023_Lib_V1.2/communication.c 的 GetReceive_SP 判定条件是
  //     if(buf[0] == SP_HEADER && buf[28] == SP_TAIL)   // 0x66 ... 0x11
  //   也就是它只认 29 字节的包, 帧尾必须落在 buf[28]。
  //
  //   36 字节的包里 buf[28] 是 target_x 的某个字节, 几乎不可能等于 0x11,
  //   于是【整包被电控丢弃】—— 云台一次指令都收不到, 表现就是"不跟随"。
  //   这个故障很难从现象看出来: 视觉端日志一切正常(照常发包、识别到了目标),
  //   电控那边也不报错(它只是静默丢弃), 两边都"看起来在工作"。
  //
  //   前 26 字节两边本来就一致(mode + yaw/vel/acc + pitch/vel/acc),
  //   分歧只在尾部: 电控要 crc16(2)+tail(1), 视觉原来发的是
  //   target_x(4)+target_y(4)+target_name(1)+tail(1)。
  //
  //   删掉的 target_x/target_y/target_name 来自 EKF 的整车中心估计
  //   (rb_auto_aim_debug.cpp:83-85), 但电控这版固件的 _SuperPower 结构体里
  //   根本没有对应字段, 收了也没处放, 所以删除不损失任何现有功能。
  //   ⚠️ 若以后要做整车跟随 / 反陀螺预测需要这三个值, 得两边一起改协议。
  struct __attribute__((packed)) sb_VisionToGimbal
  {
    uint8_t head = {0x66};    // 偏移 0，1字节：帧头。电控 SP_HEADER
    uint8_t mode = 0;         // 偏移 1，1字节：0: 不控制, 1: 控制云台但不开火，2:控制云台开火
    float yaw = 0;            // 偏移 2，4字节：目标 yaw 绝对角，单位弧度。电控 SP_OFFSET_YAW=2
    float yaw_vel = 0;        // 偏移 6，4字节：yaw 角速度前馈，单位 rad/s
    float yaw_acc = 0;        // 偏移 10，4字节：yaw 角加速度前馈，单位 rad/s^2
    float pitch = 0;          // 偏移 14，4字节：目标 pitch 绝对角，单位弧度
    float pitch_vel = 0;      // 偏移 18，4字节：pitch 角速度前馈，单位 rad/s
    float pitch_acc = 0;      // 偏移 22，4字节：pitch 角加速度前馈，单位 rad/s^2
    uint16_t crc16 = 0;       // 偏移 26，2字节：CRC16，覆盖前 26 字节。
                              //   电控当前把校验注释掉了(communication.c:115),
                              //   但这里照样正确填充 —— 以后它放开校验就不用再改视觉端。
    uint8_t end = {0x11};     // 偏移 28，1字节：帧尾。电控 SP_TAIL, 位置必须是 28
  };

  static_assert(sizeof(sb_VisionToGimbal) == 29, "必须与电控 GetReceive_SP 的 SP_TOTAL_LEN 一致");
//                      没加保险

  // static_assert(sizeof(VisionToGimbal) <= 64);


//            枚举类           
  enum class GimbalMode
  {
    IDLE,              // 空闲
    AUTO_AIM,          // 自瞄
    SMALL_BUFF,        // 小符
    BIG_BUFF,          // 大符
    LONG_FOCAL_LENGTH  //长焦
  };

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
    // std::string str(GimbalMode mode) const;
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);
    void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc);
    void sb_send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc, float target_x, float target_y,uint8_t target_name);
    GimbalState * set_state_() { return &state_; }

    private:
    serial::Serial serial_;
    std::thread thread_;
    std::atomic<bool> quit_ = false;
    mutable std::mutex mutex_;
    GimbalToVision rx_data_;
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