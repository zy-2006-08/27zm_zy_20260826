#pragma once
// macOS 上 htons/inet_addr 是宏，`::htons(port)` 这种写法语法上过不去（Linux 上是真函数）。
// 这里先包含系统头，取消宏，再补一个同名的全局函数。仓库代码一行都不用改。
#include <arpa/inet.h>
#undef htons
#undef htonl
#undef ntohs
#undef ntohl
inline uint16_t htons(uint16_t v) { return __DARWIN_OSSwapInt16(v); }
inline uint32_t htonl(uint32_t v) { return __DARWIN_OSSwapInt32(v); }
inline uint16_t ntohs(uint16_t v) { return __DARWIN_OSSwapInt16(v); }
inline uint32_t ntohl(uint32_t v) { return __DARWIN_OSSwapInt32(v); }
