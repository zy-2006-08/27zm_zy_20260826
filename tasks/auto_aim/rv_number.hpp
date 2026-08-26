#ifndef RV_AIM__NUMBER_CLASSIFIER_HPP_
#define RV_AIM__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// 引用 auto_aim 的 Armor 定义
#include "armor.hpp"

namespace rv_aim
{
class NumberClassifier
{
public:
  /**
   * @brief 构造函数
   * @param model_path ONNX模型路径
   * @param label_path 标签文件路径
   * @param threshold 分类阈值
   * @param ignore_classes 需要忽略的类别列表
   */
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold,
    const std::vector<std::string> & ignore_classes = {});

  /**
   * @brief 提取数字图像 (预处理)
   * 保留 rm_auto_aim 的透视变换 + 二值化逻辑
   * 结果将存储在 armor.pattern 中
   */
  void extractNumbers(const cv::Mat & src, std::vector<auto_aim::Armor> & armors);

  /**
   * @brief 数字分类
   * 使用 OpenCV DNN 进行推理
   */
  void classify(std::vector<auto_aim::Armor> & armors);

  double threshold;

private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
}  // namespace rv_aim

#endif  // RV_AIM__NUMBER_CLASSIFIER_HPP_