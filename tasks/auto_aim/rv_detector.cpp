#include "rv_detector.hpp"

#include <fmt/format.h>  // 需要 fmt 库，如果你的环境是 fmt/core.h 请自行调整
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

#include "tools/img_tools.hpp"  // 假设你有这个工具库
#include "tools/logger.hpp"     // 假设你有这个日志库

using namespace auto_aim;

namespace rv_aim
{

  Detector::Detector(const std::string & config_path, bool debug) : debug_(debug)  // 注意：这里的构造参数需要根据 NumberClassifier 的实际构造函数调整

  {
    auto yaml = YAML::LoadFile(config_path);
    std::string model_path = yaml["number_classifier"]["model_path"].as<std::string>();
    std::string label_path = yaml["number_classifier"]["label_path"].as<std::string>();
    classifier_ = std::make_shared<NumberClassifier>(model_path, label_path, 0.5);
    // 加载 Detector 参数 (完全复用 auto_aim 的配置项)
    // 建议在 yaml 中为 rv_detector 也可以使用 keys: "threshold", "max_angle_error" 等
    threshold_ = yaml["threshold"].as<double>();
    max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;  // degree to rad
    min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
    max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
    min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();
    min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
    max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
    max_side_ratio_ = yaml["max_side_ratio"].as<double>();
    min_confidence_ = yaml["min_confidence"].as<double>();
    max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3;

    // 更新 classifier 的阈值 (如果 yaml 里有专门为 classifier 设置的阈值)
    if (yaml["number_classifier"] && yaml["number_classifier"]["threshold"])
    {
      classifier_->threshold = yaml["number_classifier"]["threshold"].as<double>();
    }
  }

  std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int frame_count)
  {
    // 1. 预处理：转灰度 + 二值化
    cv::Mat gray_img;
    cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);

    // 2. 寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // 3. 筛选灯条 (Lightbar)
    std::list<Lightbar> lightbars;
    std::size_t lightbar_id = 0;
    for (const auto & contour : contours)
    {
      auto rotated_rect = cv::minAreaRect(contour);
      auto lightbar = Lightbar(rotated_rect, lightbar_id);

      if (!check_geometry(lightbar)) continue;

      lightbar.color = get_color(bgr_img, contour);
      lightbars.emplace_back(lightbar);
      lightbar_id++;
    }

    if (lightbars.size() < 2)
    {
      if (debug_) show_result(binary_img, bgr_img, lightbars, {}, frame_count);
      return {};
    }

    // 排序灯条 (从左到右)
    lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

    // 4. 配对灯条并构建候选装甲板 (Candidate Armors)
    std::vector<Armor> candidate_armors;  // 使用 vector 以适配 rv_aim 分类器接口

    for (auto left = lightbars.begin(); left != lightbars.end(); left++)
    {
      for (auto right = std::next(left); right != lightbars.end(); right++)
      {
        // 颜色必须相同
        if (left->color != right->color) continue;

        // 构造装甲板
        auto armor = Armor(*left, *right);

        // 几何筛选 (长宽比、两灯条长度比、平行度等)
        if (!check_geometry(armor)) continue;

        // 初步判断类型 (大/小装甲)，这对于 classifier 内部的逻辑判断很重要
        armor.type = get_type(armor);

        candidate_armors.emplace_back(armor);
      }
    }

    // 5. 批量数字识别 (这是 rv_aim 的核心差异)
    if (!candidate_armors.empty())
    {
      // 步骤 A: 提取数字图像 (透视变换 + 二值化)
      classifier_->extractNumbers(bgr_img, candidate_armors);

      // 步骤 B: 神经网络推理 + 逻辑过滤
      // (classify 内部会根据 confidence 和 type/number 匹配逻辑删除无效装甲板)
      classifier_->classify(candidate_armors);
    }

    // 6. 去重 (Deduplication)
    // 处理共用灯条的情况，保留置信度更高的
    // 为了方便操作，我们先处理 vector，最后转 list
    for (size_t i = 0; i < candidate_armors.size(); i++)
    {
      if (candidate_armors[i].name == ArmorName::not_armor) continue;  // 已经被过滤的跳过

      for (size_t j = i + 1; j < candidate_armors.size(); j++)
      {
        if (candidate_armors[j].name == ArmorName::not_armor) continue;

        auto & armor1 = candidate_armors[i];
        auto & armor2 = candidate_armors[j];

        // 检查是否共享灯条
        if (armor1.left.id == armor2.left.id || armor1.left.id == armor2.right.id || armor1.right.id == armor2.left.id || armor1.right.id == armor2.right.id)
        {
          // 冲突处理：保留置信度高的
          if (armor1.confidence > armor2.confidence)
          {
            armor2.name = ArmorName::not_armor;  // 标记为无效
          }
          else
          {
            armor1.name = ArmorName::not_armor;  // 标记为无效
          }
        }
      }
    }

    // 7. 封装最终结果
    std::list<Armor> final_armors;
    for (const auto & armor : candidate_armors)
    {
      if (armor.name != ArmorName::not_armor)
      {
        // 计算归一化中心点 (用于自瞄预测)
        // 注意：armor.center 已经在构造函数里计算了，这里计算归一化坐标
        Armor output_armor = armor;
        output_armor.center_norm = cv::Point2f(armor.center.x / bgr_img.cols, armor.center.y / bgr_img.rows);
        final_armors.emplace_back(output_armor);
      }
    }

    if (debug_) show_result(binary_img, bgr_img, lightbars, candidate_armors, frame_count);

    return final_armors;
  }

  // ================== 以下为辅助函数 (逻辑复用 auto_aim) ==================

  bool Detector::check_geometry(const Lightbar & lightbar) const
  {
    auto angle_ok = lightbar.angle_error < max_angle_error_;
    auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
    auto length_ok = lightbar.length > min_lightbar_length_;
    return angle_ok && ratio_ok && length_ok;
  }

  bool Detector::check_geometry(const Armor & armor) const
  {
    auto ratio_ok = armor.ratio > min_armor_ratio_ && armor.ratio < max_armor_ratio_;
    auto side_ratio_ok = armor.side_ratio < max_side_ratio_;
    auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_;
    return ratio_ok && side_ratio_ok && rectangular_error_ok;
  }

  Color Detector::get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const
  {
    int red_sum = 0, blue_sum = 0;
    for (const auto & point : contour)
    {
      red_sum += bgr_img.at<cv::Vec3b>(point)[2];
      blue_sum += bgr_img.at<cv::Vec3b>(point)[0];
    }
    return blue_sum > red_sum ? Color::blue : Color::red;
  }

  ArmorType Detector::get_type(const Armor & armor)
  {
    // 复用 auto_aim 的几何判断逻辑
    // 注意：这只是为了给 classifier 提供一个初始 type 提示
    // classifier 之后可能会根据识别出的数字校验这个 type 是否合法
    if (armor.ratio > 3.0) return ArmorType::big;
    if (armor.ratio < 2.5) return ArmorType::small;
    return ArmorType::small;  // 默认为小装甲
  }

  void Detector::show_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars, const std::vector<Armor> & armors, int frame_count) const
  {
    auto detection = bgr_img.clone();
    tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

    // 绘制灯条
    for (const auto & lightbar : lightbars)
    {
      tools::draw_points(detection, lightbar.points, {0, 255, 255}, 1);
    }

    // 绘制装甲板
    for (const auto & armor : armors)
    {
      if (armor.name == ArmorName::not_armor) continue;  // 过滤掉无效的

      auto info = fmt::format("{:.2f} {}", armor.confidence, ARMOR_NAMES[armor.name]);

      tools::draw_points(detection, armor.points, {0, 255, 0}, 2);
      tools::draw_text(detection, info, armor.left.bottom, {0, 255, 0});

      // 可以在这里画出透视变换后的二值图用于调试
      // if (!armor.pattern.empty()) {
      //    cv::imshow("debug_pattern", armor.pattern);
      // }
    }

    cv::Mat binary_show;
    cv::resize(binary_img, binary_show, {}, 0.5, 0.5);
    cv::imshow("rv_binary", binary_show);

    cv::resize(detection, detection, {}, 0.5, 0.5);
    cv::imshow("rv_detection", detection);
  }

}  // namespace rv_aim