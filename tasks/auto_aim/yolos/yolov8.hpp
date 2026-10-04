#ifndef AUTO_AIM__YOLOV8_HPP
#define AUTO_AIM__YOLOV8_HPP

#include <chrono>
#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

  class YOLOV8 : public YOLOBase
  {
    public:
    YOLOV8(const std::string & config_path, bool debug);

    std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

    std::list<Armor> postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

    private:
    Classifier classifier_;
    Detector detector_;

    std::string device_, model_path_;
    std::string save_path_, debug_path_;
    bool debug_, use_roi_;
    // ★2026-10-03 (分支 fix/yolov8-corner-refine) 用传统CV灯条精修 YOLO 角点, yaml use_traditional 控制。
    //   yolov5 一直有这一步, yolov8 原来漏了: yaml 写 true 但从未生效。
    bool use_traditional_ = false;
    int refine_total_ = 0, refine_ok_ = 0;  // 精修成功率统计, 每 2 秒打一次日志
    std::chrono::steady_clock::time_point refine_report_t_ = std::chrono::steady_clock::now();

    const int class_num_ = 2;
    const float nms_threshold_ = 0.3;
    const float score_threshold_ = 0.5;
    double min_confidence_, binary_threshold_;

    ov::Core core_;
    ov::CompiledModel compiled_model_;

    cv::Rect roi_;
    cv::Point2f offset_;

    bool check_name(const Armor & armor) const;
    bool check_type(const Armor & armor) const;

    cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;
    ArmorType get_type(const Armor & armor);
    cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

    std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

    void save(const Armor & armor) const;
    void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
    void sort_keypoints(std::vector<cv::Point2f> & keypoints);
  };

}  // namespace auto_aim

#endif  // TOOLS__YOLOV8_HPP