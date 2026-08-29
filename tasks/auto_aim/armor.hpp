#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
  // 装甲板灯条颜色。红/蓝是两方，extinguish 指灯条没亮（熄灭），purple 是紫色（基地相关）。
  // 注意上层找的是**敌方**颜色：io/gimbal 收到的是我方颜色，取反后才是目标色（gimbal.cpp:315）。
  // 识别后会把非敌方颜色的板直接 remove_if 掉（tracker.cpp 各 track 方法开头）。
  enum Color
  {
    red,
    blue,
    extinguish,
    purple
  };
  const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

  // 装甲板尺寸类型。大板宽 230mm，小板宽 135mm（实际常量见 solver.cpp:12-14）。
  // 它决定 PnP 用哪一组 3D 物点，选错则解出的距离整体偏掉。
  // 判定在 detector.cpp 的 get_type：先看 ratio（>3.0 判大、<2.5 判小，这两个阈值
  // **写死在代码里、不在 yaml**），落在中间才按兵种猜（1号英雄=大板，其余=小板）。
  enum ArmorType
  {
    big,
    small
  };
  const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

  // 装甲板上的数字/兵种。one~five 是车号，sentry 哨兵、outpost 前哨站、base 基地，
  // not_armor 表示数字分类器认为这不是装甲板（会被 check_name 滤掉）。
  // 这个字段决定跟踪时用几块板的模型：普通车 4 块、平衡步兵 2 块、前哨站/基地 3 块
  // （见 tracker.cpp 的 set_target 按兵种分支）。
  enum ArmorName
  {
    one,
    two,
    three,
    four,
    five,
    sentry,
    outpost,
    base,
    not_armor
  };
  const std::vector<std::string> ARMOR_NAMES = {"one", "two", "three", "four", "five", "sentry", "outpost", "base", "not_armor"};

  // 打击优先级，数字越小优先级越高（1 最高）。
  // ★但目前形同虚设：Armor::priority 从未被赋值（详见下面该字段处的说明），
  // 且只有哨兵的 sb_track 真的按它排序，track/test_track 里那行都被注释掉了。
  enum ArmorPriority
  {
    first = 1,
    second,
    third,
    forth,
    fifth
  };

  // clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
  // clang-format on

  // ============================ 灯条 ============================
  // "灯条"就是装甲板两侧那两根竖着发光的灯管，在图像里是一条细长的亮斑。
  // 传统CV 的识别思路：二值化把亮的地方抠出来 -> 找轮廓 -> 每个轮廓套一个最小外接旋转矩形
  // -> 长宽比合适的认为是灯条 -> 同色的左右两根配成一块装甲板。
  // ★本结构体所有坐标都是【像素】。此时还没做 PnP，根本不知道距离。
  // 构造过程见 armor.cpp 的 Lightbar::Lightbar。
  struct Lightbar
  {
    std::size_t id;  // 本帧内的序号。用来判断两块板是否共用了同一根灯条（去重用）
    Color color;     // 灯条颜色，由 get_color 遍历轮廓内像素判定
    // center:     旋转矩形中心（像素）
    // top/bottom: 上下两条短边的**中点**（像素），不是角点——是两个角点的平均
    // top2bottom: bottom - top，灯条的方向和长度都由它导出
    cv::Point2f center, top, bottom, top2bottom;
    std::vector<cv::Point2f> points;  // 就是 {top, bottom}，方便统一遍历
    // angle:       灯条方向角 atan2(top2bottom)，单位【弧度】
    // angle_error: |angle - π/2|，偏离"竖直"多少弧度。对应 yaml 的 max_angle_error
    //              （yaml 里写的是度，构造函数读入时 /57.3 转弧度）
    // length:      灯条长度（像素）。对应 yaml 的 min_lightbar_length，滤掉远处小亮点
    // width:       灯条宽度（像素）
    // ratio:       length/width。灯条是细长的，这个比值太小说明是个方块而非灯条。
    //              对应 yaml 的 min_lightbar_ratio / max_lightbar_ratio
    double angle, angle_error, length, width, ratio;
    cv::RotatedRect rotated_rect;  // 原始最小外接旋转矩形，留着画图/调试用

    Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
    Lightbar() {};
  };

  // ============================ 装甲板 ============================
  // 整条链路里最重要的数据结构：识别的输出、跟踪的输入。
  // 前半段（color~confidence）是图像域的量，单位像素；
  // 后半段（xyz_in_*/ypr_in_*）要等 solver.solve() 做完 PnP 和坐标变换才被填上，单位米/弧度。
  // 也就是说 detector 刚吐出来的 Armor，后半段是无效的。
  struct Armor
  {
    Color color;
    Lightbar left, right;     //used to be const
    // ★不是对角线交点，不能作为实际中心！
    // 它是"左右两根灯条中心的中点"：armor.cpp 里 center = (left.center + right.center) / 2。
    // 装甲板倾斜或有透视时，这个点和真正的几何中心有偏差，所以它只能用于粗筛和排序
    // （例如 tracker.cpp 里按"离图像中心的远近"排序），绝不能当解算输入。
    // 真正参与 PnP 的是下面 points 里那 4 个角点。
    cv::Point2f center;
    cv::Point2f center_norm;  // 归一化坐标：与图像尺寸无关的比例值，便于跨分辨率比较和画曲线
    // ★PnP 的图像点，顺序是 {左灯条上, 右灯条上, 右灯条下, 左灯条下}
    // （armor.cpp 的 Armor 构造里按此顺序 emplace）。
    // 这个顺序必须与 solver.cpp:16-25 那两组 3D 物点严格一一对应；
    // 错位不会报错，但会解出完全离谱的位姿，是新手最容易踩的坑之一。
    std::vector<cv::Point2f> points;

    // 下面三个是配对后的几何筛选量，都由 check_geometry(Armor) 使用，是调识别最常动的旋钮：
    double ratio;              // 两灯条的中点连线与长灯条的长度之比。
                               // 对应 yaml 的 min_armor_ratio / max_armor_ratio。
                               // 大板比小板"宽"，所以它也是区分大小板的依据（get_type）
    double side_ratio;         // 长灯条与短灯条的长度之比。对应 yaml 的 max_side_ratio。
                               // 正对时两根灯条一样长（≈1），侧得越厉害差得越多
    double rectangular_error;  // 灯条与中点连线的夹角偏离 π/2 多少，单位【弧度】。
                               // 对应 yaml 的 max_rectangular_error（yaml 写度，读入时 /57.3）。
                               // 真装甲板是矩形，灯条应垂直于左右连线，偏差大就是误配对

    ArmorType type;  // 大板/小板，决定 PnP 用哪组 3D 物点
    ArmorName name;  // 兵种/数字，由数字分类器给出
    // ★这个字段从未被计算或赋值过。全仓库唯一涉及它的写操作是 target.cpp:33 把它复制走，
    // 没有任何地方给它算出一个值，所以读到的是未初始化内容。
    // 加上只有 sb_track（tracker.cpp）真的按它排序，所以现在别指望用它控制打击顺序。
    ArmorPriority priority;
    int class_id;       // YOLO 输出的类别下标（传统CV 路径不使用）
    cv::Rect box;       // YOLO 输出的检测框（传统CV 路径不使用）
    cv::Mat pattern;    // 抠出来的数字贴图，送给分类器认数字
    double confidence;  // 数字分类置信度。对应 yaml 的 min_confidence，低于它会被 check_name 滤掉
    bool duplicated;    // 与别的板共用了灯条的标记，由 detector.cpp 的去重段设置后统一 remove_if

    Eigen::Vector3d xyz_in_gimbal;  // 单位：m
    Eigen::Vector3d xyz_in_world;   // 单位：m
    Eigen::Vector3d ypr_in_gimbal;  // 单位：rad
    Eigen::Vector3d ypr_in_world;   // 单位：rad
    Eigen::Vector3d ypd_in_world;   // 球坐标系

    double yaw_raw;  // rad

    Armor(const Lightbar & left, const Lightbar & right);
    Armor(int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
    Armor(int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
    Armor(int color_id, int num_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
    Armor(int color_id, int num_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
  };

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP