#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <string>

namespace tools
{
std::shared_ptr<spdlog::logger> logger_ = nullptr;

void set_logger()
{//                                 年份 月份 日期 24小时制小时 分钟 秒
  auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(spdlog::level::debug);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);//单独设置【终端输出槽】的日志等级为 debug

  logger_ = std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
  logger_->set_level(spdlog::level::debug);//全局最低过滤线
  logger_->flush_on(spdlog::level::info);//设置自动刷盘触发等级：当日志消息等级 ≥info时，spdlog立刻把缓冲区数据写入磁盘文件，不再留在内存缓存。
}

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
