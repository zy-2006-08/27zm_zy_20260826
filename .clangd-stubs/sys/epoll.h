/*
 * clangd 专用桩头文件 —— 不参与任何真实编译。
 *
 * io/socketcan.hpp 用 epoll 做 CAN 收包的事件等待，那是 Linux 特有的 API，
 * macOS 上对应的是 kqueue，头文件 <sys/epoll.h> 并不存在。
 *
 * 这里只声明 socketcan.hpp 实际用到的部分：
 *   epoll_event / epoll_create1 / epoll_ctl / epoll_wait
 *   EPOLLIN / EPOLL_CTL_ADD / EPOLL_CTL_DEL
 *
 * 真实编译在 Linux 上进行，用的是系统真正的 <sys/epoll.h>。
 */
#ifndef CLANGD_STUB_SYS_EPOLL_H
#define CLANGD_STUB_SYS_EPOLL_H

#include <stdint.h>

#define EPOLLIN 0x001
#define EPOLLOUT 0x004
#define EPOLLERR 0x008
#define EPOLLHUP 0x010

#define EPOLL_CTL_ADD 1
#define EPOLL_CTL_DEL 2
#define EPOLL_CTL_MOD 3

typedef union epoll_data {
  void * ptr;
  int fd;
  uint32_t u32;
  uint64_t u64;
} epoll_data_t;

struct epoll_event
{
  uint32_t events;
  epoll_data_t data;
};

extern "C" {
int epoll_create(int size);
int epoll_create1(int flags);
int epoll_ctl(int epfd, int op, int fd, struct epoll_event * event);
int epoll_wait(int epfd, struct epoll_event * events, int maxevents, int timeout);
}

#endif  // CLANGD_STUB_SYS_EPOLL_H
