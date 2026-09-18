#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |                           | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml       | 相机配置yaml路径 }"
  "{@output-folder |      assets/calib_imgs    | 图片输出文件夹   }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>("config-path");
  auto output_folder = cli.get<std::string>(0);

  std::filesystem::create_directories(output_folder);

  tools::Exiter exiter;
  io::Camera camera(config_path);

  // 相机读帧放后台线程：阻塞读不再卡住主线程的按键响应
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  std::mutex mtx;
  std::atomic<bool> has_frame{false};
  std::atomic<bool> quit{false};

  std::thread capture_thread([&] {
    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;
    while (!quit) {
      camera.read(frame, ts);
      std::lock_guard<std::mutex> lock(mtx);
      img = frame;
      timestamp = ts;
      has_frame = true;
    }
  });

  int count = 0;
  tools::logger()->info("按 s 保存图片，按 q/ESC 退出");
  while (!exiter.exit()) {
    auto key = cv::waitKey(30);

    if (key == 'q' || key == 27 || key == 'Q') break;

    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(mtx);
      if (has_frame) frame = img;
    }
    if (frame.empty()) continue;

    cv::imshow("Press s to save, q to quit", frame);

    if (key == 's' || key == 'S') {
      count++;
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      cv::imwrite(img_path, frame);
      tools::logger()->info("[{}] Saved: {}", count, img_path);
    }
  }

  // 直接结束进程，避免 read 线程阻塞导致无法 join
  std::exit(0);
}