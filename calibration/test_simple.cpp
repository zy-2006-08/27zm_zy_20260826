// test_simple.cpp
#include <chrono>  // 添加：支持 std::chrono
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>  // 添加：支持 std::this_thread

#include "io/camera.hpp"

int main()
{
  io::Camera camera("../configs/calibration.yaml");
  cv::Mat img;
  std::chrono::steady_clock::time_point ts;

  for (int i = 0; i < 10000; i++)
  {
    camera.read(img, ts);
    if (!img.empty())
    {
      cv::imshow("Test", img);
      cv::waitKey(1);
      std::cout << "Frame " << i << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}