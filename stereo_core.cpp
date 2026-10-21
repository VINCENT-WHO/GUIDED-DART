// 核心算法实现：标定参数读取、绿色圆心检测、三角化及目标到基线的距离。
#include "stereo_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

// 匿名命名空间内的辅助函数只在本源文件可见。
namespace {
// 兼容 YAML 的普通数组和 !!opencv-matrix 两种格式，统一转换成双精度矩阵。
cv::Mat matrix(const cv::FileNode& node, int rows) {
    cv::Mat m;
    // 数组先读成连续向量，clone() 使矩阵拥有独立内存，避免向量销毁后数据失效。
    if(node.isSeq()) { std::vector<double> v; node >> v; m=cv::Mat(v).clone(); }
    else node >> m;
    // 检查元素数能否按所需行数重排；reshape 只改变形状，坐标值不会变化。
    if(m.empty() || m.total()%rows) throw std::runtime_error("Invalid calibration matrix");
    m.convertTo(m,CV_64F);
    m=m.reshape(1,rows).clone();
    if(!cv::checkRange(m)) throw std::runtime_error("Nonfinite calibration matrix");
    return m;
}
}
// 读取内参：K=[fx,0,cx; 0,fy,cy; 0,0,1]，D 为畸变系数。
Intrinsics loadIntrinsics(const std::filesystem::path& path) {
    cv::FileStorage f(path.string(),cv::FileStorage::READ);
    if(!f.isOpened()) throw std::runtime_error("Cannot open intrinsics: "+path.string());
    Intrinsics i;
    i.size={int(f["image_width"]),int(f["image_height"])};
    // 优先使用原标定文件中的 OpenCV 矩阵节点；不存在时读取一维数组节点。
    i.K=matrix(f[f["camera_matrix_opencv"].empty()?"camera_matrix":"camera_matrix_opencv"],3);
    i.D=matrix(f[f["dist_coeffs_opencv"].empty()?"distort_coeffs":"dist_coeffs_opencv"],1);
    // 内参必须是 3×3，焦距和图像尺寸为正；只接受 OpenCV 支持的畸变系数数量。
    if(i.K.size()!=cv::Size(3,3) || i.K.at<double>(0,0)<=0 || i.K.at<double>(1,1)<=0 ||
       i.size.width<=0 || i.size.height<=0 ||
       (i.D.total()!=4 && i.D.total()!=5 && i.D.total()!=8 && i.D.total()!=12 && i.D.total()!=14))
        throw std::runtime_error("Invalid intrinsics: "+path.string());
    return i;
}
// 图像尺寸必须匹配标定；相同尺寸仍需用户保证 ROI 原点、binning、翻转和对焦一致。
void checkImageSize(const Intrinsics& intr, cv::Size size) {
    if(intr.size!=size) throw std::runtime_error("Frame size differs from calibration (expected "+
        std::to_string(intr.size.width)+"x"+std::to_string(intr.size.height)+"); ROI/binning must match calibration");
}
// 每次只检测当前帧；不修改相机或内参。空 optional 代表没找到可信的绿色圆。
std::optional<Target> detectGreen(const cv::Mat& bgr, const DetectorSettings& c, cv::Mat& mask,
    DetectionInfo* info, const Target* previous) {
    DetectionInfo d;
    cv::Mat hsv;
    // SDK 输出 BGR，必须用 BGR2HSV。H∈[0,179]，S、V∈[0,255]。
    cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
    // 分割条件：hMin≤H≤hMax、sMin≤S≤255、vMin≤V≤255；符合条件输出 255（白），否则 0（黑）。
    cv::inRange(hsv,cv::Scalar(c.hMin,c.sMin,c.vMin),cv::Scalar(c.hMax,255,255),mask);
    // 开运算去小噪点，闭运算连接小断裂/填补小孔；目标太小时大核会误删目标。
    if(c.morphologySize>1) {
        auto k=cv::getStructuringElement(cv::MORPH_ELLIPSE,{c.morphologySize,c.morphologySize});
        cv::morphologyEx(mask,mask,cv::MORPH_OPEN,k);
        cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,k);
    }
    // 只提取外轮廓，并保存全部边界点；统计形态学处理后的白像素和轮廓数量。
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    d.whitePixels=cv::countNonZero(mask); d.contours=int(contours.size());
    std::optional<Target> best, near;
    // best 用于首次捕获最大轮廓；near 用于跟踪时选择距上一圆心最近的合格轮廓。
    double closest=std::numeric_limits<double>::max();
    for(const auto& contour:contours) {
        // A 是面积，P 是周长，圆度 C=4πA/P²；理想圆约为 1，细长物体通常较低。
        double a=cv::contourArea(contour), p=cv::arcLength(contour,true);
        auto box=cv::boundingRect(contour);
        const double circularity=p>0?4*CV_PI*a/(p*p):0;
        if(a>d.largestArea){d.largestArea=a;d.largestCircularity=circularity;}
        // 依次排除触边、过小、过大、非圆轮廓，并统计原因；触边会造成中心偏移。
        if(box.x<=0 || box.y<=0 || box.br().x>=mask.cols || box.br().y>=mask.rows){++d.clipped;continue;}
        if(a<c.minArea){++d.small;continue;}
        if(a>mask.total()*c.maxAreaFraction){++d.large;continue;}
        if(p<=0 || circularity<c.minCircularity){++d.noncircular;continue;}
        // 轮廓矩求面积中心：u=m10/m00、v=m01/m00；坐标保持原图像素尺度。
        auto m=cv::moments(contour);
        if(m.m00<=0) continue;
        Target t;
        t.center={m.m10/m.m00,m.m01/m.m00};
        // 等效半径 r=sqrt(A/π) 仅用于显示和跟踪尺度，不用它推算目标距离。
        t.radius=std::sqrt(a/CV_PI); t.area=a; t.contour=contour;
        ++d.accepted;
        if(!best || a>best->area) best=t;
        // 位置与面积连续性：位移≤max(25 px,3r)，面积比在 [0.25,4]；符合者中选最近的。
        if(previous){
            const double distance=cv::norm(t.center-previous->center);
            const double ratio=t.area/previous->area;
            if(distance<=std::max(25.0,3*previous->radius) && ratio>=0.25 && ratio<=4 && distance<closest){
                near=t;closest=distance;
            }
        }
    }
    // 跟踪时没有连续候选就返回空值，不用旧点或任意其他绿色物体冒充当前目标。
    if(previous)best=near;
    // 生成可读诊断：空掩码、等待重捕获，或各类轮廓被剔除的数量。
    if(best)d.status="OK";
    else if(d.whitePixels==0)d.status="Mask empty: check HSV / target / exposure";
    else if(d.accepted>0 && previous)d.status="Target moved / changed: reacquiring";
    else d.status="Rejected: edge="+std::to_string(d.clipped)+" small="+std::to_string(d.small)+
        " large="+std::to_string(d.large)+" shape="+std::to_string(d.noncircular);
    if(info)*info=d;
    return best;
}
// 外参遵循 X_right=R*X_left+T；平移必须以 mm 存储。
StereoGeometry loadGeometry(const std::filesystem::path& path) {
    cv::FileStorage f(path.string(),cv::FileStorage::READ);
    if(!f.isOpened()) throw std::runtime_error("Cannot open stereo extrinsics: "+path.string());
    std::string unit; f["translation_unit"]>>unit;
    if(unit!="mm") throw std::runtime_error("Extrinsics translation_unit must be mm");
    StereoGeometry g{matrix(f["R"],3),matrix(f["T"],3),false};
    // 验证 R 的尺寸、正交性及 det(R)≈1；T 必须是非零 3×1 向量，防止退化基线。
    if(g.R.size()!=cv::Size(3,3) || g.T.size()!=cv::Size(1,3) || cv::norm(g.T)<1e-6 ||
       cv::norm(g.R.t()*g.R-cv::Mat::eye(3,3,CV_64F))>1e-5 || std::abs(cv::determinant(g.R)-1)>1e-5)
        throw std::runtime_error("Invalid stereo R/T");
    return g;
}
// 输入左右原始像素圆心和各自内参/畸变，输出带有效标记的三维测距结果。
RangeResult triangulateTarget(const cv::Point2d& l, const cv::Point2d& r,
    const Intrinsics& kl,const Intrinsics& kr,const StereoGeometry& g,double minAngle) {
    RangeResult out;
    std::vector<cv::Point2d> ul,ur;
    // 先去畸变并转成归一化平面坐标 (x,y)，因此后续投影矩阵不再乘 K。
    cv::undistortPoints(std::vector<cv::Point2d>{l},ul,kl.K,kl.D);
    cv::undistortPoints(std::vector<cv::Point2d>{r},ur,kr.K,kr.D);
    // 构造射线 (x,y,1)；右射线用 Rᵀ 转回左相机坐标系后，才能与左射线比较方向。
    cv::Mat vl=(cv::Mat_<double>(3,1)<<ul[0].x,ul[0].y,1);
    cv::Mat vr=(cv::Mat_<double>(3,1)<<ur[0].x,ur[0].y,1);
    vr=g.R.t()*vr;
    // 夹角 θ=acos((vL·vR)/(|vL||vR|))；clamp 防止浮点误差让 acos 输入超出 [-1,1]。
    double cosine=vl.dot(vr)/(cv::norm(vl)*cv::norm(vr));
    out.rayAngleDegrees=std::acos(std::clamp(cosine,-1.0,1.0))*180/CV_PI;
    // 射线几乎平行时深度对像素误差非常敏感，夹角小于门限就拒绝测量。
    if(!std::isfinite(out.rayAngleDegrees) || out.rayAngleDegrees<minAngle) {
        out.reason="Too little parallax"; return out;
    }
    // 归一化投影模型：P_left=[I|0]，P_right=[R|T]；两矩阵为 3×4。
    cv::Mat p1=cv::Mat::zeros(3,4,CV_64F), p2=cv::Mat::zeros(3,4,CV_64F);
    cv::Mat eye=cv::Mat::eye(3,3,CV_64F); eye.copyTo(p1(cv::Rect(0,0,3,3)));
    g.R.copyTo(p2(cv::Rect(0,0,3,3))); g.T.copyTo(p2.col(3));
    // OpenCV 通过左右投影约束的线性三角化求齐次点 h=(Xh,Yh,Zh,W)。
    cv::Mat h; cv::triangulatePoints(p1,p2,ul,ur,h);
    // 用 W 做齐次除法；W≈0 表示点接近无穷远，不能输出稳定距离。
    double w=h.at<double>(3);
    if(!cv::checkRange(h) || std::abs(w)<1e-12) { out.reason="Point at infinity"; return out; }
    // X_left=(Xh,Yh,Zh)/W，X_right=R*X_left+T；两侧 Z 都必须为正（点在相机前方）。
    cv::Mat x=h.rowRange(0,3)/w, xr=g.R*x+g.T;
    if(x.at<double>(2)<=0 || xr.at<double>(2)<=0) { out.reason="Behind camera: check R/T and camera order"; return out; }
    out.leftPoint={x.at<double>(0),x.at<double>(1),x.at<double>(2)};
    out.rightPoint={xr.at<double>(0),xr.at<double>(1),xr.at<double>(2)};
    // 到各光心的欧氏距离只是辅助结果：DL=|X_left|，DR=|X_right|，单位 mm。
    out.leftDistance=cv::norm(x); out.rightDistance=cv::norm(xr);
    // 主结果为点到基线直线的距离。左光心为 0，右光心 C=-RᵀT。
    // D=|X_left×C|/|C|；水平基线时 D=sqrt(Y²+Z²)，不要求目标等高，也不限制垂足在线段内。
    cv::Mat center=-g.R.t()*g.T;
    cv::Vec3d baseline(center.at<double>(0),center.at<double>(1),center.at<double>(2));
    const double baselineLength=cv::norm(baseline);
    if(!std::isfinite(baselineLength) || baselineLength<1e-6){out.reason="Invalid baseline";return out;}
    cv::Vec3d point(out.leftPoint.x,out.leftPoint.y,out.leftPoint.z);
    out.baselineDistance=cv::norm(point.cross(baseline))/baselineLength;
    // 将三角化点分别投影回左右原图，包含各自 K 和 D；零旋转/平移表示点已在该相机坐标系中。
    std::vector<cv::Point2d> ql,qr;
    cv::projectPoints(std::vector<cv::Point3d>{out.leftPoint},cv::Vec3d(),cv::Vec3d(),kl.K,kl.D,ql);
    cv::projectPoints(std::vector<cv::Point3d>{out.rightPoint},cv::Vec3d(),cv::Vec3d(),kr.K,kr.D,qr);
    // 残差 e=max(|预测左像素-实际左像素|,|预测右像素-实际右像素|)，单位 px。
    // 它检查左右观测与当前几何模型的一致性；通过也不等于绝对测距精度已得到保证。
    out.reprojection=std::max(cv::norm(ql[0]-l),cv::norm(qr[0]-r));
    // 重投影误差只做观测和记录，不再使用 2 px 或其他像素门限拒绝距离。
    // 仍拒绝非有限数值；正深度、非零基线和最小射线夹角已在前面检查。
    // valid=true 只表示当前估计可计算，不代表几何参数或实际测距精度已经验证。
    out.valid=std::isfinite(out.baselineDistance) && std::isfinite(out.leftDistance) &&
        std::isfinite(out.rightDistance) && std::isfinite(out.reprojection);
    out.reason=out.valid?"Estimate (reprojection not filtered)":"Nonfinite result: check calibration";
    return out;
}
