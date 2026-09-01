// Mac 本地编译的兼容补丁，只在 mac/build.sh 里通过 -include 注入。
// 仓库里的代码一行都不用改，小电脑上的构建完全不受影响。
#pragma once

#include "fmt_enum_shim.hpp"
#include "mac_shim.hpp"
