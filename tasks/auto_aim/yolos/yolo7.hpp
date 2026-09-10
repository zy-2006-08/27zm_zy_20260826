#ifndef AUTO_AIM__YOLO7_HPP
#define AUTO_AIM__YOLO7_HPP

// ============================================================================
//  YOLO7 —— 修正版关键点排序的 YOLOV8
// ----------------------------------------------------------------------------
//  这不是 YOLOv7 网络。名字沿用 yolo_name 的命名习惯，实际加载的仍是
//  assets/yolov8.xml（416x416），推理部分与 YOLOV8 完全相同。
//
//  存在的唯一目的：替换 sort_keypoints 的实现，验证「装甲板 roll 超过约 22°
//  就识别不到」这个现象是不是排序缺陷造成的。
//
//  ---- 问题 ----
//  yolov8.cpp:300 的 sort_keypoints 先按 y 坐标把四点分成"上两个/下两个"：
//      std::sort(kps.begin(), kps.end(), [](a,b){ return a.y < b.y; });
//      top = {kps[0], kps[1]};  bottom = {kps[2], kps[3]};
//  这假设"y 最小的两点必然是上边两角"。小装甲板 135mm x 55mm，高宽比 1:2.45，
//  当 roll 超过 arctan(55/135) = 22.2° 时，左下角的 y 已经小于右上角，
//  于是 top 里混进了侧边的角，四点对应关系整体错位一格
//  （实测 30° 时排序结果从 [TL,TR,BR,BL] 变成 [BL,TL,TR,BR]）。
//
//  错位后果在 armor.cpp:135 的构造函数里：left_width/top_length/ratio/
//  rectangular_error 全部按错误的点对计算，ratio 从 2.45 掉到 0.4 左右，
//  后续 check_type 判错或几何量离谱，装甲板被丢弃。
//
//  实测印证（2026-09-05，detect_test 截图）：
//      roll  0°  conf 0.99  hit rate 100%
//      roll 19°  conf 0.96  hit rate  78.6%   ← 仍在 22° 内
//      更斜      识别 0 个   hit rate  64%     ← 越过阈值
//
//  ---- 解法 ----
//  改成按极角排序：以四点重心为原点算 atan2，绕一圈排序。
//  这个方法与旋转角无关，任意 roll 都能得到正确的绕行顺序，
//  再挑出真正的左上角作为起点即可。详见 yolo7.cpp 的 sort_keypoints。
//
//  ---- 用法 ----
//  configs/*.yaml 里写 yolo_name: yolo7 即可切换，与 yolov8 对照。
//  原 yolov8.cpp / yolo11.cpp 一行未改，随时可以切回去对比。
// ============================================================================

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

  class YOLO7 : public YOLOBase
  {
    public:
    YOLO7(const std::string & config_path, bool debug);

    std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

    std::list<Armor> postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

    private:
    Classifier classifier_;
    Detector detector_;

    std::string device_, model_path_;
    std::string save_path_, debug_path_;
    bool debug_, use_roi_;

    const int class_num_ = 2;
    const float nms_threshold_ = 0.3;
    const float score_threshold_ = 0.7;
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

    /// ★与 YOLOV8 的唯一区别：按极角排序，不按 y 坐标分组
    void sort_keypoints(std::vector<cv::Point2f> & keypoints);
  };

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO7_HPP
