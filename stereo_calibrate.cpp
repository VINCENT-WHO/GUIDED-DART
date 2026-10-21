// 双目外参标定工具：从成对棋盘格图像估计 R/T，固定已经标定好的单目内参。
#include "stereo_core.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
// 文件系统缩写；下方角点检测函数放在匿名命名空间，避免对其他源文件导出。
namespace fs=std::filesystem;
namespace {
// 输入一张 BGR 棋盘图，输出按网格顺序排列的内角点，坐标必须回到原始分辨率。
bool corners(const cv::Mat& image,cv::Size pattern,std::vector<cv::Point2f>& points){
    // 角点检测使用灰度图，颜色不参与棋盘格识别。
    cv::Mat gray;cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
    bool found=false;
    // 先尝试半分辨率再尝试全分辨率，兼顾高分辨率图像的速度和检测成功率。
    for(double scale:{0.5,1.0}){
        cv::Mat small;cv::resize(gray,small,{},scale,scale,cv::INTER_AREA);
        // 自适应阈值和亮度归一化帮助检测黑白格；pattern 是内角点列、行数，不是方格数。
        found=cv::findChessboardCorners(small,pattern,points,cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE);
        if(found){
            // 缩小图上的角点按像素中心坐标约定映射回原图，然后做亚像素精化。
            for(auto& p:points){p.x=float((p.x+0.5)*gray.cols/small.cols-0.5);p.y=float((p.y+0.5)*gray.rows/small.rows-0.5);}
            cv::cornerSubPix(gray,points,{11,11},{-1,-1},{cv::TermCriteria::COUNT|cv::TermCriteria::EPS,50,0.001});
            return true;
        }
    }
    // 传统检测两次都失败时再用 SB 检测，返回其亚像素角点结果。
    return cv::findChessboardCornersSB(gray,pattern,points,cv::CALIB_CB_NORMALIZE_IMAGE);
}
}
// 独立标定程序入口：本工具处理已有图片，不打开相机。
int main(int argc,char** argv){
    try{
        // 读取成对图片目录、内角点数、真实方格边长（mm）、输出文件及可接受 RMS 门限。
        cv::CommandLineParser cli(argc,argv,
            "{help h||Help}{config|stereo_config.yaml|Configuration}"
            "{pairs|captures|Directory containing left and right PNG pairs}"
            "{cols|9|Inner corner columns}{rows|6|Inner corner rows}"
            "{square-mm|18|Actual physical square size in mm}"
            "{output|stereo_extrinsics.yaml|Output R/T}{max-rms|2|Reject calibration above this RMS (px)}");
        if(cli.has("help")){cli.printMessage();return 0;}
        if(!cli.check()){cli.printErrors();return 1;}
        // 棋盘尺寸和方格边长决定三维角点坐标；实际方格长度错误会按比例影响平移尺度。
        cv::Size pattern(cli.get<int>("cols"),cli.get<int>("rows"));
        double square=cli.get<double>("square-mm"),maxRms=cli.get<double>("max-rms");
        if(pattern.width<2 || pattern.height<2 || !std::isfinite(square) || square<=0 || !std::isfinite(maxRms) || maxRms<=0)
            throw std::runtime_error("Invalid board geometry or max-rms");
        // 从配置中读取左右内参及序列号；内参相对路径以配置文件目录为基准。
        auto config=fs::absolute(cli.get<std::string>("config"));
        cv::FileStorage f(config.string(),cv::FileStorage::READ);
        if(!f.isOpened()) throw std::runtime_error("Cannot open config");
        std::string lf,rf,ls,rs;f["left_intrinsics"]>>lf;f["right_intrinsics"]>>rf;
        f["left_serial"]>>ls;f["right_serial"]>>rs;
        // 本工具要求左右原图分辨率相同；使用既有 K/D，不重新估计单目内参。
        auto kl=loadIntrinsics(config.parent_path()/lf),kr=loadIntrinsics(config.parent_path()/rf);
        if(kl.size!=kr.size)throw std::runtime_error("This calibration tool requires equal sensor resolutions");
        // --pairs 路径相对于启动目录；其中 left/right 目录保存同名 PNG 才算一对。
        fs::path root=cli.get<std::string>("pairs");
        if(!fs::is_directory(root/"left") || !fs::is_directory(root/"right"))throw std::runtime_error("Need pairs/left and pairs/right folders");
        // 如果采集时保存了相机映射，则核对其身份，避免左右调换后仍混用旧图。
        if(fs::exists(root/"camera_mapping.yaml")){
            cv::FileStorage mapping((root/"camera_mapping.yaml").string(),cv::FileStorage::READ);
            std::string ml,mr;mapping["left_serial"]>>ml;mapping["right_serial"]>>mr;
            if((!ls.empty() && ls!=ml) || (!rs.empty() && rs!=mr))throw std::runtime_error("Capture serials differ from config");
            ls=ml;rs=mr;
        }
        // 按左侧 PNG 文件名排序，随后逐一查找右侧同名图片；不使用预览或掩码文件。
        std::vector<fs::path> paths;
        for(auto& e:fs::directory_iterator(root/"left"))if(e.path().extension()==".png")paths.push_back(e.path());
        std::sort(paths.begin(),paths.end());
        // 在棋盘局部坐标系定义角点 (列×格长,行×格长,0)，单位 mm，所有点共面。
        std::vector<cv::Point3f> board;
        for(int y=0;y<pattern.height;++y)for(int x=0;x<pattern.width;++x)board.emplace_back(float(x*square),float(y*square),0);
        // 每对图存一组左像素点、右像素点及对应三维角点；names 记录用于标定的图片。
        std::vector<std::vector<cv::Point2f>> left,right;
        std::vector<std::vector<cv::Point3f>> objects;
        std::vector<std::string> names;
        fs::path debug=root/"corners";fs::create_directories(debug);
        // 逐对解码并检查分辨率，只有左右都完整检测到棋盘角点时才加入优化。
        for(auto& path:paths){
            auto other=root/"right"/path.filename();
            if(!fs::exists(other)){std::cout<<"Missing right pair: "<<path.filename()<<'\n';continue;}
            auto a=cv::imread(path.string()),b=cv::imread(other.string());
            if(a.empty() || b.empty())throw std::runtime_error("Cannot decode pair "+path.filename().string());
            checkImageSize(kl,a.size());checkImageSize(kr,b.size());
            std::vector<cv::Point2f> pa,pb;
            bool fa=corners(a,pattern,pa),fb=corners(b,pattern,pb);
            std::cout<<path.filename()<<" left="<<fa<<" right="<<fb<<std::endl;
            if(!fa || !fb)continue;
            // 普通棋盘有 180° 编号歧义；相对滚转小于 90° 的左右安装中，用网格方向统一顺序。
            // 该简单规则不能替代任意安装姿态下的可靠编号，应检查保存的角点对应图。
            if((pa.back()-pa.front()).dot(pb.back()-pb.front())<0)std::reverse(pb.begin(),pb.end());
            // 加入本对有效观测；绘制左右角点对应图并缩小拼接，供人工核查方向和精度。
            left.push_back(pa);right.push_back(pb);objects.push_back(board);names.push_back(path.filename().string());
            cv::drawChessboardCorners(a,pattern,pa,true);cv::drawChessboardCorners(b,pattern,pb,true);
            cv::resize(a,a,{},0.3,0.3);cv::resize(b,b,{},0.3,0.3);cv::hconcat(a,b,a);
            cv::imwrite((debug/path.filename()).string(),a);
        }
        // 至少需要 10 对有效图；建议 15～25 对，改变棋盘位置和倾角，相机安装关系保持不动。
        if(left.size()<10)throw std::runtime_error("Need >=10 usable stereo pairs; recommend 15-25 varied positions and tilts");
        // R/T 是所需双目外参；E/F 分别为本质矩阵和基础矩阵，描述两幅图的极线约束。
        cv::Mat R,T,E,F;
        // 固定 K/D，联合优化 R/T 和每次棋盘姿态，最小化所有角点的重投影误差。
        // 终止条件为最多 100 次迭代或参数变化足够小；返回 RMS 单位 px，不能直接当作距离误差。
        double rms=cv::stereoCalibrate(objects,left,right,kl.K,kl.D,kr.K,kr.D,kl.size,R,T,E,F,
            cv::CALIB_FIX_INTRINSIC,{cv::TermCriteria::COUNT|cv::TermCriteria::EPS,100,1e-8});
        // 拒绝无效数值、过大 RMS 或退化基线；门限通过也仍需用已知距离验证测距。
        if(!std::isfinite(rms) || rms>maxRms || !cv::checkRange(R) || !cv::checkRange(T) || cv::norm(T)<1e-6)
            throw std::runtime_error("Stereo calibration rejected; RMS="+std::to_string(rms)+" px. Check board size, ordering and captures.");
        // 不覆盖已有外参文件，避免丢失旧标定；使用其他输出文件名可保存新的标定结果。
        auto output=cli.get<std::string>("output");
        if(fs::exists(output))throw std::runtime_error("Output already exists; choose another --output to preserve it");
        cv::FileStorage out(output,cv::FileStorage::WRITE);
        if(!out.isOpened())throw std::runtime_error("Cannot write extrinsics");
        // 写入外参、单位、相机身份、质量统计和所用图片；测距程序以后按同一约定读取。
        out<<"translation_unit"<<"mm"<<"R"<<R<<"T"<<T<<"E"<<E<<"F"<<F;
        out<<"left_serial"<<ls<<"right_serial"<<rs<<"left_intrinsics"<<lf<<"right_intrinsics"<<rf;
        out<<"rms_reprojection_error_px"<<rms<<"baseline_mm"<<cv::norm(T);
        out<<"pattern_cols"<<pattern.width<<"pattern_rows"<<pattern.height<<"square_size_mm"<<square<<"used_pairs"<<names;
        // 打印结果并提示在配置中启用外参；重新移动相机或改变对焦后需重新验证。
        std::cout<<"RMS="<<rms<<" px, baseline="<<cv::norm(T)<<" mm\nR="<<R<<"\nT="<<T<<"\nSaved "<<output
            <<"\nSet extrinsics in stereo_config.yaml, keeping this camera order and rigid mounting.\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<"Calibration error: "<<e.what()<<std::endl;return 1;}
}
