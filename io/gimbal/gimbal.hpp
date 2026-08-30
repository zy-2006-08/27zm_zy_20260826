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
    void sb_send(io::sb_VisionToGimbal VisionToGimbal);
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