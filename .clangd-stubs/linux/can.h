/*
 * clangd 专用桩头文件 —— 不参与任何真实编译。
 *
 * io/socketcan.hpp 包含 <linux/can.h>，那是 Linux SocketCAN 的头文件，
 * macOS 上不存在。缺了它 clangd 会在 io/cboard.hpp 处中断解析，
 * 并连带在下游文件（如 src/rb_auto_standard_debug.cpp）报出一串
 * 假的类型错误。
 *
 * 这里只声明项目实际用到的那几个类型和常量，让 clangd 能把语法树建完。
 * 真实编译在 Linux 上进行，用的是系统真正的 <linux/can.h>。
 */
#ifndef CLANGD_STUB_LINUX_CAN_H
#define CLANGD_STUB_LINUX_CAN_H

#include <stdint.h>
#include <sys/socket.h>

typedef uint32_t canid_t;

#define CAN_MAX_DLEN 8

/* socket 协议族 / 类型常量 */
#define AF_CAN 29
#define PF_CAN AF_CAN
#define CAN_RAW 1

struct can_frame
{
  canid_t can_id;
  uint8_t can_dlc;
  uint8_t __pad;
  uint8_t __res0;
  uint8_t __res1;
  uint8_t data[CAN_MAX_DLEN];
};

struct sockaddr_can
{
  sa_family_t can_family;
  int can_ifindex;
  union {
    struct
    {
      canid_t rx_id;
      canid_t tx_id;
    } tp;
  } can_addr;
};

#endif  // CLANGD_STUB_LINUX_CAN_H
