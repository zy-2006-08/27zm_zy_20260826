#ifndef RV_AIM__DETECTOR_HPP_
#define RV_AIM__DETECTOR_HPP_

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// 引用 auto_aim 的基础数据结构
#include "armor.hpp"
// 引用 rv_aim 的数字识别器
#include "rv_number.hpp"

namespace rv_aim
{

  class Detector
  {
    public:
    /**
   * @brief 构造函数
   * @param config_path 配置文件路径
   * @param debug 是否开启调试显示
   */
    Detector(const std::string & config_path, bool debug = true);

    /**
   * @brief 每一帧的主检测函数
   * @param bgr_img 输入图像
   * @param frame_count 帧计数（用于Debug显示）
   * @return 识别到的装甲板列表
   */
    std::list<auto_aim::Armor> detect(const cv::Mat & bgr_img, int frame_count = -1);

    private:
    // 使用 rv_aim 的分类器
    std::shared_ptr<NumberClassifier> classifier_;

    // 几何筛选参数 (与 auto_aim 保持一致)
    double threshold_;
    double max_angle_error_;
    double min_lightbar_ratio_, max_lightbar_ratio_;
    double min_lightbar_length_;
    double min_armor_ratio_, max_armor_ratio_;
    double max_side_ratio_;
    double min_confidence_;
    double max_rectangular_error_;

    bool debug_;

    // 几何筛选辅助函数
    bool check_geometry(const auto_aim::Lightbar & lightbar) const;
    bool check_geometry(const auto_aim::Armor & armor) const;

    // 辅助计算函数
    auto_aim::Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const;
    auto_aim::ArmorType get_type(const auto_aim::Armor & armor);  // 仅基于几何判断大小装甲

    // Debug 显示
    void show_result(
      const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<auto_aim::Lightbar> & lightbars, const std::vector<auto_aim::Armor> & armors,
      int frame_count) const;
  };

}  // namespace rv_aim

#endif  // RV_AIM__DETECTOR_HPP_