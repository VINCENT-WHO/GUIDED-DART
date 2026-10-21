#pragma once  // 同一编译单元只包含本头文件一次，避免重复定义。
#include <opencv2/opencv.hpp>  // 图像、矩阵、标定、三角化等 OpenCV 接口。
#include <filesystem>         // 跨平台文件路径。
#include <optional>           // optional 无值表示当前没有检测到目标。

// 一台相机的标定参数。所有检测像素坐标必须使用与此标定相同的成像设置。
struct Intrinsics {
    cv::Mat K, D;   // K：3×3 内参矩阵（焦距和主点，单位 px）；D：畸变系数。
    cv::Size size;  // 标定时的图像宽、高；缩放预览不能改变实际计算坐标。
};
// 从 YAML 读取内参，并检查尺寸、焦距和畸变模型是否有效。
Intrinsics loadIntrinsics(const std::filesystem::path& path);
// 实际图像分辨率不一致时抛异常，防止把不同尺度的像素代入原内参。
void checkImageSize(const Intrinsics& intr, cv::Size size);

// 这里只定义缺省值：主程序随后加载 YAML 公共值、左右独立值及保存的覆盖值。
struct DetectorSettings {
    int hMin=35, hMax=90, sMin=90, vMin=70;  // H∈[0,179]；S、V∈[0,255]，上下限均包含。
    double minArea=12;             // 轮廓面积下限，单位为原图像素²。
    double maxAreaFraction=0.45;   // 轮廓面积最多占整幅图像的比例。
    double minCircularity=0.45;    // 最小圆度 4πA/P²；理想圆约为 1。
    int morphologySize=3;          // 椭圆形态学核的边长，奇数；1 表示不做开闭运算。
};
// 一帧中选出的绿色目标；圆心用轮廓面积矩求得，并非强制椭圆拟合。
struct Target {
    cv::Point2d center;               // 原始图像中的圆心 (u,v)，单位 px。
    double radius=0, area=0;          // 等效半径 sqrt(A/π) 和实际轮廓面积。
    std::vector<cv::Point> contour;   // 轮廓点，保留供后续分析或绘图。
};
// 记录 HSV 分割和形状筛选的诊断信息，帮助区分“没分割出目标”和“轮廓不合格”。
struct DetectionInfo {
    int whitePixels=0, contours=0, accepted=0;          // 白像素数、外轮廓数、合格轮廓数。
    int clipped=0, small=0, large=0, noncircular=0;     // 因触边、面积或形状被剔除的数量。
    double largestArea=0, largestCircularity=0;        // 最大原始轮廓的面积、圆度。
    std::string status;                               // 用于界面和终端的诊断说明。
};
// 输入 BGR 原图，输出形态学处理后的 0/255 掩码。
// info 可省略；previous 提供上一目标以选择位置连续的轮廓，无合格目标就返回空值。
std::optional<Target> detectGreen(const cv::Mat& bgr, const DetectorSettings& cfg,
    cv::Mat& mask, DetectionInfo* info=nullptr, const Target* previous=nullptr);

// 坐标约定：X_right = R * X_left + T，平移和三维坐标的单位都是 mm。
struct StereoGeometry {
    cv::Mat R, T;             // R 为 3×3 旋转矩阵；T 为 3×1 平移向量。
    bool approximate=false;   // true 表示由机械基线和完全平行假设构造，并非实测标定外参。
};
// 读取外参，验证单位、非零基线及旋转矩阵的正交性。
StereoGeometry loadGeometry(const std::filesystem::path& path);

// 三维定位及距离结果。valid 只表示通过基本可计算性检查；重投影残差不再参与门限筛选。
struct RangeResult {
    cv::Point3d leftPoint, rightPoint;  // 同一目标在左右相机坐标系中的位置，单位 mm。
    double baselineDistance=0;        // 主测距结果：目标到两光心所在无限直线的垂直距离，mm。
    double leftDistance=0, rightDistance=0;  // 辅助结果：目标到左右光心的直线距离，mm。
    double reprojection=0;             // 左右回投影残差的较大值，px；仅执行到回投影阶段才有意义。
    double rayAngleDegrees=0;          // 两条观察射线在同一坐标系下的夹角，单位度。
    bool valid=false;                  // 默认不可计算；正深度、足够夹角且数值有限时允许显示估计。
    std::string reason;                // 当前有效状态或失败原因，失败时解释 N/A。
};
// 像素去畸变 → 三角化 → 点到基线距离 → 记录回投影残差（无像素门限）。
// minAngleDegrees 为最小射线夹角（度），防止射线几乎平行导致深度退化。
RangeResult triangulateTarget(const cv::Point2d& l, const cv::Point2d& r,
    const Intrinsics& kl, const Intrinsics& kr, const StereoGeometry& geo,
    double minAngleDegrees=0.02);
