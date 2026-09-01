// 只给 Mac 本地编译用：fmt 12 不再隐式格式化 unscoped enum，这里补上通用 formatter。
// 小电脑上 fmt 8 不需要它，仓库代码一行都不用改。
#pragma once
#include <fmt/format.h>
#include <type_traits>

template <typename E>
struct fmt::formatter<E, char, std::enable_if_t<std::is_enum_v<E>>>
  : fmt::formatter<std::underlying_type_t<E>>
{
  template <typename Ctx>
  auto format(E e, Ctx & ctx) const
  {
    return fmt::formatter<std::underlying_type_t<E>>::format(
      static_cast<std::underlying_type_t<E>>(e), ctx);
  }
};
