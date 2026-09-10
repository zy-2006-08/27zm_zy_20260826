#include "yolo7.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <random>

#include "tasks/auto_aim/classifier.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
  YOLO7::YOLO7(const std::string & config_path, bool debug) : classifier_(config_path), detector_(config_path), debug_(debug)
  {
    auto yaml = YAML::LoadFile(config_path);

    model_path_ = yaml["yolov8_model_path"].as<std::string>();
    device_ = yaml["device"].as<std::string>();
    binary_threshold_ = yaml["threshold"].as<double>();
    min_confidence_ = yaml["min_confidence"].as<double>();
    int x = 0, y = 0, width = 0, height = 0;
    x = yaml["roi"]["x"].as<int>();
    y = yaml["roi"]["y"].as<int>();
    width = yaml["roi"]["width"].as<int>();
    height = yaml["roi"]["height"].as<int>();
    use_roi_ = yaml["use_roi"].as<bool>();
    roi_ = cv::Rect(x, y, width, height);
    offset_ = cv::Point2f(x, y);

    save_path_ = "imgs";
    std::filesystem::create_directory(save_path_);

    auto model = core_.read_model(model_path_);
    ov::preprocess::PrePostProcessor ppp(model);
    auto & input = ppp.input();

    input.tensor().set_element_type(ov::element::u8).set_shape({1, 416, 416, 3}).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);

    input.model().set_layout("NCHW");

    input.preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale(255.0);

    // TODO: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
    model = ppp.build();
    compiled_model_ = core_.compile_model(model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  }

  std::list<Armor> YOLO7::detect(const cv::Mat & raw_img, int frame_count)
  {
    if (raw_img.empty())
    {
      tools::logger()->warn("Empty img!, camera drop!");
      return std::list<Armor>();
    }

    cv::Mat bgr_img;
    if (use_roi_)
    {
      if (roi_.width == -1)
      {  // -1 表示该维度不裁切
        roi_.width = raw_img.cols;
      }
      if (roi_.height == -1)
      {  // -1 表示该维度不裁切
        roi_.height = raw_img.rows;
      }
      bgr_img = raw_img(roi_);
    }
    else
    {
      bgr_img = raw_img;
    }

    auto x_scale = static_cast<double>(416) / bgr_img.rows;
    auto y_scale = static_cast<double>(416) / bgr_img.cols;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(bgr_img.rows * scale);
    auto w = static_cast<int>(bgr_img.cols * scale);

    // preproces
    auto input = cv::Mat(416, 416, CV_8UC3, cv::Scalar(0, 0, 0));
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(bgr_img, input(roi), {w, h});
    ov::Tensor input_tensor(ov::element::u8, {1, 416, 416, 3}, input.data);

    /// infer
    auto infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    // postprocess
    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

    return parse(scale, output, raw_img, frame_count);
  }

  std::list<Armor> YOLO7::parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
  {
    // for each row: xywh + classess
    cv::transpose(output, output);

    std::vector<int> ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armors_key_points;
    for (int r = 0; r < output.rows; r++)
    {
      auto xywh = output.row(r).colRange(0, 4);
      auto scores = output.row(r).colRange(4, 4 + class_num_);
      auto one_key_points = output.row(r).colRange(4 + class_num_, 14);

      std::vector<cv::Point2f> armor_key_points;

      double score;
      cv::Point max_point;
      cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

      if (score < score_threshold_) continue;

      auto x = xywh.at<float>(0);
      auto y = xywh.at<float>(1);
      auto w = xywh.at<float>(2);
      auto h = xywh.at<float>(3);
      auto left = static_cast<int>((x - 0.5 * w) / scale);
      auto top = static_cast<int>((y - 0.5 * h) / scale);
      auto width = static_cast<int>(w / scale);
      auto height = static_cast<int>(h / scale);

      for (int i = 0; i < 4; i++)
      {
        float x = one_key_points.at<float>(0, i * 2 + 0) / scale;
        float y = one_key_points.at<float>(0, i * 2 + 1) / scale;
        cv::Point2f kp = {x, y};
        armor_key_points.push_back(kp);
      }
      ids.emplace_back(max_point.x);
      confidences.emplace_back(score);
      boxes.emplace_back(left, top, width, height);
      armors_key_points.emplace_back(armor_key_points);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

    std::list<Armor> armors;
    for (const auto & i : indices)
    {
      sort_keypoints(armors_key_points[i]);
      if (use_roi_)
      {
        armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
      }
      else
      {
        armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i]);
      }
    }

    for (auto it = armors.begin(); it != armors.end();)
    {
      it->pattern = get_pattern(bgr_img, *it);
      classifier_.classify(*it);

      if (!check_name(*it))
      {
        it = armors.erase(it);
        continue;
      }

      it->type = get_type(*it);
      if (!check_type(*it))
      {
        it = armors.erase(it);
        continue;
      }

      it->center_norm = get_center_norm(bgr_img, it->center);
      ++it;
    }

    if (debug_) draw_detections(bgr_img, armors, frame_count);

    return armors;
  }

  bool YOLO7::check_name(const Armor & armor) const
  {
    auto name_ok = armor.name != ArmorName::not_armor;
    auto confidence_ok = armor.confidence > min_confidence_;

    // 保存不确定的图案，用于分类器的迭代
    // if (name_ok && !confidence_ok) save(armor);

    return name_ok && confidence_ok;
  }

  bool YOLO7::check_type(const Armor & armor) const
  {
    auto name_ok = (armor.type == ArmorType::small) ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                                                    : (armor.name != ArmorName::two && armor.name != ArmorName::sentry && armor.name != ArmorName::outpost);

    // 保存异常的图案，用于分类器的迭代
    // if (!name_ok) save(armor);

    return name_ok;
  }

  ArmorType YOLO7::get_type(const Armor & armor)
  {
    // 英雄、基地只能是大装甲板
    if (armor.name == ArmorName::one || armor.name == ArmorName::base)
    {
      return ArmorType::big;
    }

    // 工程、哨兵、前哨站只能是小装甲板
    if (armor.name == ArmorName::two || armor.name == ArmorName::sentry || armor.name == ArmorName::outpost)
    {
      return ArmorType::small;
    }

    // 步兵假设为小装甲板
    return ArmorType::small;
  }

  cv::Point2f YOLO7::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
  {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return {center.x / w, center.y / h};
  }

  cv::Mat YOLO7::get_pattern(const cv::Mat & bgr_img, const Armor & armor) const
  {
    // 延长灯条获得装甲板角点
    // 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
    auto tl = (armor.points[0] + armor.points[3]) / 2 - (armor.points[3] - armor.points[0]) * 1.125;
    auto bl = (armor.points[0] + armor.points[3]) / 2 + (armor.points[3] - armor.points[0]) * 1.125;
    auto tr = (armor.points[2] + armor.points[1]) / 2 - (armor.points[2] - armor.points[1]) * 1.125;
    auto br = (armor.points[2] + armor.points[1]) / 2 + (armor.points[2] - armor.points[1]) * 1.125;

    auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);
    auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);
    auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
    auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);
    auto roi_tl = cv::Point(roi_left, roi_top);
    auto roi_br = cv::Point(roi_right, roi_bottom);
    auto roi = cv::Rect(roi_tl, roi_br);

    // 检查ROI是否有效
    if (roi_left < 0 || roi_top < 0 || roi_right <= roi_left || roi_bottom <= roi_top)
    {
      // std::cerr << "Invalid ROI: " << roi << std::endl;
      return cv::Mat();  // 返回一个空的Mat对象
    }

    // 检查ROI是否超出图像边界
    if (roi_right > bgr_img.cols || roi_bottom > bgr_img.rows)
    {
      // std::cerr << "ROI out of image bounds: " << roi << " Image size: " << bgr_img.size()
      //           << std::endl;
      return cv::Mat();  // 返回一个空的Mat对象
    }

    return bgr_img(roi);
  }

  void YOLO7::save(const Armor & armor) const
  {
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, ARMOR_NAMES[armor.name], file_name);
    cv::imwrite(img_path, armor.pattern);
  }

  void YOLO7::draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
  {
    auto detection = img.clone();
    tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
    for (const auto & armor : armors)
    {
      auto info = fmt::format("{:.2f} {} {}", armor.confidence, ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
      tools::draw_points(detection, armor.points, {0, 255, 0});
      tools::draw_text(detection, info, armor.center, {0, 255, 0});
    }

    if (use_roi_)
    {
      cv::Scalar green(0, 255, 0);
      cv::rectangle(detection, roi_, green, 2);
    }
    cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("detection", detection);
  }

  // ============================================================================
  //  ★这是 YOLO7 与 YOLOV8 的唯一区别：按极角排序，不按 y 坐标分上下两组
  // ----------------------------------------------------------------------------
  //  原实现（yolov8.cpp:300）的做法是先按 y 排序，取前两个当"上边两角"：
  //      sort(kps, by y);  top = {kps[0], kps[1]};  bottom = {kps[2], kps[3]};
  //  它隐含假设"y 最小的两点一定是上边两角"。小装甲板 135x55，高宽比 1:2.45，
  //  roll 超过 arctan(55/135) = 22.2° 后左下角的 y 就小于右上角，
  //  top 里混进侧边的角，四点对应关系整体错位一格
  //  （数值验证：30° 时排序结果由 [TL,TR,BR,BL] 变为 [BL,TL,TR,BR]）。
  //  错位会让 armor.cpp:135 算出的 ratio / rectangular_error 完全失真，
  //  板子随后被 check_type 或几何筛选丢掉——表现就是"斜一点就识别不到"。
  //
  //  本实现分两步，与 roll 角无关：
  //    1) 以四点重心为原点算极角 atan2(dy,dx)，按极角排序。凸四边形的四个顶点
  //       绕重心的极角必然是单调的，所以排完一定是沿边界绕行的顺序（顺时针或
  //       逆时针，取决于图像坐标系 y 向下，这里得到的是顺时针）。
  //       这一步不依赖任何"上下左右"的先验，因此不受旋转影响。
  //    2) 绕行顺序确定后还要定"从哪个点开始"。取 (x+y) 最小的点作为左上角：
  //       在图像坐标系（x 右、y 下）里，左上角的 x 和 y 同时最小，
  //       该判据在 roll 在 ±45° 内始终成立，比"y 最小"稳健得多。
  //       再从该点开始按已排好的绕行顺序取满四个点。
  //
  //  结果同样是 {左上, 右上, 右下, 左下}，与 armor.hpp 里 points 的约定一致
  //  （该顺序必须与 solver.cpp:16-25 的 3D 物点严格对应，错位不会报错但会解出
  //  离谱的位姿）。
  // ============================================================================
  void YOLO7::sort_keypoints(std::vector<cv::Point2f> & keypoints)
  {
    if (keypoints.size() != 4)
    {
      std::cout << "beyond 4!!" << std::endl;
      return;
    }

    // 1) 求重心，按绕重心的极角排序，得到沿边界的绕行顺序
    cv::Point2f center(0.f, 0.f);
    for (const auto & p : keypoints) center += p;
    center /= 4.0f;

    std::sort(keypoints.begin(), keypoints.end(), [&center](const cv::Point2f & a, const cv::Point2f & b) {
      return std::atan2(a.y - center.y, a.x - center.x) < std::atan2(b.y - center.y, b.x - center.x);
    });

    // 2) 找左上角作为起点：图像坐标系下它的 x+y 最小
    int tl_index = 0;
    float min_sum = keypoints[0].x + keypoints[0].y;
    for (int i = 1; i < 4; i++)
    {
      float sum = keypoints[i].x + keypoints[i].y;
      if (sum < min_sum)
      {
        min_sum = sum;
        tl_index = i;
      }
    }

    // 3) 从左上角起，按第 1 步的绕行顺序重排成 {TL, TR, BR, BL}
    std::vector<cv::Point2f> sorted(4);
    for (int i = 0; i < 4; i++) sorted[i] = keypoints[(tl_index + i) % 4];
    keypoints = sorted;
  }

  std::list<Armor> YOLO7::postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
  {
    return parse(scale, output, bgr_img, frame_count);
  }

}  // namespace auto_aim