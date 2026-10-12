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

    protected:
    // 推理后端钩子：输入 416x416 BGR u8 letterbox 图，输出 [14 x 3549] 的 CV_32F 矩阵
    // (行 = xywh + 2 类分数 + 8 个关键点坐标)。
    // 默认实现走 OpenVINO；Jetson 上 YOLOV8TRT 覆盖它改走 TensorRT。
    // 前处理(letterbox) 和后处理(parse/分类/角点精修/画图) 两个后端完全共用，
    // 保证 x86 与 Jetson 识别逻辑逐行一致，差别只在网络前向这一步。
    // load_model=false 时构造函数跳过 OpenVINO 模型加载(给 TRT 子类用)。
    YOLOV8(const std::string & config_path, bool debug, bool load_model);
    virtual cv::Mat infer(const cv::Mat & input_416_bgr);

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