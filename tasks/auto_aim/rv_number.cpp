// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "rv_number.hpp"

// 使用 auto_aim 的命名空间以访问 ArmorType, ArmorName 等
using namespace auto_aim;

namespace rv_aim
{
  NumberClassifier::NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double thre, const std::vector<std::string> & ignore_classes)
  : threshold(thre), ignore_classes_(ignore_classes)
  {
    // 仅使用 OpenCV DNN，完全保留原逻辑，不引入 OpenVINO
    net_ = cv::dnn::readNetFromONNX(model_path);

    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line))
    {
      class_names_.push_back(line);
    }
  }

  void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
  {
    // Light length in image
    const int light_length = 12;
    // Image size after warp
    const int warp_height = 28;
    const int small_armor_width = 32;
    const int large_armor_width = 54;
    // Number ROI size
    const cv::Size roi_size(20, 28);

    for (auto & armor : armors)
    {
      // Warp perspective transform
      // 适配 auto_aim 数据结构: armor.left.top/bottom
      cv::Point2f lights_vertices[4] = {armor.left.bottom, armor.left.top, armor.right.top, armor.right.bottom};

      const int top_light_y = (warp_height - light_length) / 2 - 1;
      const int bottom_light_y = top_light_y + light_length;

      // 适配 auto_aim 数据结构: ArmorType::small (注意 auto_aim 是小写)
      const int warp_width = armor.type == ArmorType::small ? small_armor_width : large_armor_width;

      cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
      };

      cv::Mat number_image;
      auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
      cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

      // Get ROI
      number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

      // Binarize (完全保留 rm_auto_aim 的二值化逻辑)
      cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
      cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

      // 将结果存入 auto_aim::Armor 的 pattern 成员中
      armor.pattern = number_image;
    }
  }

  void NumberClassifier::classify(std::vector<Armor> & armors)
  {
    for (auto & armor : armors)
    {
      // 使用 pattern 作为输入 (这是我们在 extractNumbers 中存储二值图的地方)
      cv::Mat image = armor.pattern.clone();

      // Normalize
      image = image / 255.0;

      // Create blob from image
      cv::Mat blob;
      cv::dnn::blobFromImage(image, blob);

      // Set the input blob for the neural network
      net_.setInput(blob);
      // Forward pass the image blob through the model
      cv::Mat outputs = net_.forward();

      // Do softmax
      float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
      cv::Mat softmax_prob;
      cv::exp(outputs - max_prob, softmax_prob);
      float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
      softmax_prob /= sum;

      double confidence;
      cv::Point class_id_point;
      cv::minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
      int label_id = class_id_point.x;

      armor.confidence = confidence;

      // 获取字符串形式的标签
      std::string label_str = class_names_[label_id];

      // --- 增加适配逻辑: String -> Enum ---
      // auto_aim::Armor 使用 ArmorName 枚举，我们需要将识别出的 string 转换回 enum
      if (label_str == "1")
        armor.name = ArmorName::one;
      else if (label_str == "2")
        armor.name = ArmorName::two;
      else if (label_str == "3")
        armor.name = ArmorName::three;
      else if (label_str == "4")
        armor.name = ArmorName::four;
      else if (label_str == "5")
        armor.name = ArmorName::five;
      else if (label_str == "sentry" || label_str == "guard")
        armor.name = ArmorName::sentry;
      else if (label_str == "outpost")
        armor.name = ArmorName::outpost;
      else if (label_str == "base")
        armor.name = ArmorName::base;
      else
        armor.name = ArmorName::not_armor;
      // ---------------------------------

      // 依然可以保留 string 形式用于调试打印
      std::stringstream result_ss;
      result_ss << label_str << ": " << std::fixed << std::setprecision(1) << armor.confidence * 100.0 << "%";
      // 注意：auto_aim::Armor 结构体本身没有 classfication_result 成员，
      // 如果需要存储这个字符串，您可能需要在 armor.hpp 中添加该成员，或者直接打印日志。
      // 这里为了不报错，暂时注释掉赋值，或者您可以将其打印出来：
      // std::cout << "[RV_AIM] " << result_ss.str() << std::endl;
    }

    // 过滤逻辑 (保留 rm_auto_aim 的逻辑)
    armors.erase(
      std::remove_if(
        armors.begin(), armors.end(),
        [this](const Armor & armor) {
          if (armor.confidence < threshold)
          {
            return true;
          }

          // 需要重新反向映射 enum 到 string 来进行比较，或者直接比较 enum
          // 为了严格保留原逻辑的字符串比较风格，我们再次获取字符串：
          std::string current_label;
          if (armor.name == ArmorName::one)
            current_label = "1";
          else if (armor.name == ArmorName::two)
            current_label = "2";
          else if (armor.name == ArmorName::three)
            current_label = "3";
          else if (armor.name == ArmorName::four)
            current_label = "4";
          else if (armor.name == ArmorName::five)
            current_label = "5";
          else if (armor.name == ArmorName::sentry)
            current_label = "sentry";  // 注意这里可能是 sentry 或 guard
          else if (armor.name == ArmorName::outpost)
            current_label = "outpost";
          else if (armor.name == ArmorName::base)
            current_label = "base";
          else
            current_label = "not_armor";

          for (const auto & ignore_class : ignore_classes_)
          {
            if (current_label == ignore_class)
            {
              return true;
            }
          }

          bool mismatch_armor_type = false;
          // 适配 ArmorType::big (原 ArmorType::LARGE)
          if (armor.type == ArmorType::big)
          {
            mismatch_armor_type =
              current_label == "outpost" || current_label == "2" || current_label == "guard" || current_label == "sentry" || current_label == "base";
          }
          // 适配 ArmorType::small (原 ArmorType::SMALL)
          else if (armor.type == ArmorType::small)
          {
            mismatch_armor_type = current_label == "1";
          }
          return mismatch_armor_type;
        }),
      armors.end());
  }

}  // namespace rv_aim