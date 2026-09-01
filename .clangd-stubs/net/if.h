/*
 * clangd 专用桩头文件 —— 不参与任何真实编译。
 *
 * macOS 上 <net/if.h> 是存在的，但 struct ifreq 的布局和 Linux 不同：
 * 没有 ifr_ifindex 成员，也没有 SIOCGIFINDEX 这个 ioctl 请求码
 * （macOS 取网卡索引用的是 if_nametoindex()）。
 *
 * io/socketcan.hpp 第 86 / 92 行用到了这两个 Linux 专属的东西，
 * 所以这里先把系统真正的 <net/if.h> 引进来（#include_next），
 * 再补上缺的那两个符号。
 *
 * ifr_ifindex 复用系统 union 里现成的 ifru_intval（同为 int），
 * 只为让 clangd 能把类型对上，不代表运行时语义等价。
 *
 * 真实编译在 Linux 上进行，用的是系统真正的 <net/if.h>。
 */
#ifndef CLANGD_STUB_NET_IF_H
#define CLANGD_STUB_NET_IF_H

/* 先加载 macOS SDK 里真正的 net/if.h */
#include_next <net/if.h>

/* Linux 的 ioctl 请求码：按网卡名查询接口索引 */
#ifndef SIOCGIFINDEX
#define SIOCGIFINDEX 0x8933
#endif

/* Linux 的 struct ifreq 成员，映射到 macOS union 中同类型的 ifru_intval */
#ifndef ifr_ifindex
#define ifr_ifindex ifr_ifru.ifru_intval
#endif

#endif  // CLANGD_STUB_NET_IF_H
