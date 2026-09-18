#include "exiter.hpp"

#include <csignal>
#include <stdexcept>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false;

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");//抛出一个运行时异常，程序立刻终止
  std::signal(SIGINT, [](int) { exit_ = true; });//绑定一个信号到来时触发执行的回调
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

}  // namespace tools