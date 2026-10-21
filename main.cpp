// 实时测距入口：加载参数 → 左右取图 → 圆心检测 → 三维计算 → 显示、调参和保存。
#include "stereo_core.hpp"
#include "mvs_camera.hpp"
#include <atomic>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <chrono>
#include <csignal>
#include <future>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

// 缩写文件系统命名空间；steady_clock 不受系统校时影响，用于计算经过时间。
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
namespace {
// 信号处理函数只设置可安全读写的退出标记，不在信号回调中操作图像或硬件。
volatile std::sig_atomic_t stopRequested=0;
void signalHandler(int){stopRequested=1;}
// YAML 节点存在时覆盖变量，否则保留原值；模板兼容整数、浮点和字符串。
template<class T> void read(const cv::FileStorage& f,const char* key,T& value){if(!f[key].empty()) f[key]>>value;}
// 把浮点数格式化为固定小数位字符串，仅影响显示，不改变测距精度。
std::string number(double x,int digits=2){std::ostringstream s;s<<std::fixed<<std::setprecision(digits)<<x;return s.str();}
// 状态栏独占顶部 190 像素，避免文字遮住图像；鼠标取色必须扣除此高度。
constexpr int statusHeight=190;
// 根据指定宽度缩放预览；检测与测距始终使用未缩放原图坐标。
cv::Mat preview(const cv::Mat& image,const std::optional<Target>& target,const std::vector<std::string>& lines,int width){
    cv::Mat view;
    double scale=double(width)/image.cols;
    cv::resize(image,view,{},scale,scale,cv::INTER_AREA);
    // 将原图圆心/半径按相同比例映射到预览，画红色圆和十字；未检测到时不画旧点。
    if(target){
        cv::Point p(cvRound(target->center.x*scale),cvRound(target->center.y*scale));
        cv::circle(view,p,std::max(3,cvRound(target->radius*scale)),{0,0,255},2);
        cv::drawMarker(view,p,{0,0,255},cv::MARKER_CROSS,18,2);
    }
    // 创建独立黑色状态栏，再把图像复制到下方；逐行写入英文状态（OpenCV 默认字体不支持中文）。
    cv::Mat out(statusHeight+view.rows,width,CV_8UC3,cv::Scalar());
    view.copyTo(out(cv::Rect(0,statusHeight,width,view.rows)));
    int y=21;
    for(const auto& text:lines){cv::putText(out,text,{8,y},cv::FONT_HERSHEY_SIMPLEX,0.43,{0,255,255},1,cv::LINE_AA);y+=23;}
    return out;
}
// 滑条回调收到新值后写入对应变量；通过 void* 传入变量地址，避免旧接口的指针警告。
void trackbarChanged(int value,void* storage){*static_cast<int*>(storage)=value;}
// 创建滑条并设置初始值；storage 的生命周期必须覆盖整个 UI 循环。
void slider(const char* label,const std::string& window,int& storage,int maximum){
    cv::createTrackbar(label,window,nullptr,maximum,trackbarChanged,&storage);
    cv::setTrackbarPos(label,window,storage);
}
// 一台相机的调参和连续跟踪状态；左右各有一个实例，互不混用。
struct Tuning {
    // settings 保存实际检测参数；圆度滑条用整数百分数表示，运行时再除以 100。
    DetectorSettings settings;
    int circularity=45;
    std::string controls;
    // 保留当前原图供鼠标取色；width 是预览宽度，不是相机传感器分辨率。
    cv::Mat image;
    int width=540;
    // previous 只供下一帧筛选使用；misses 计数连续丢失，lastSettings 用于检测参数是否改变。
    std::optional<Target> previous;
    int misses=0;
    std::vector<double> lastSettings;
    // 重置跟踪记忆，要求之后重新从当前图像捕获；不会清空检测阈值。
    void reset(){previous.reset();misses=0;}
};
// 自动取色或校正 H 上下限后，把实际参数同步回界面滑条位置。
void syncControls(Tuning& t){
    cv::setTrackbarPos("H min",t.controls,t.settings.hMin);cv::setTrackbarPos("H max",t.controls,t.settings.hMax);
    cv::setTrackbarPos("S min",t.controls,t.settings.sMin);cv::setTrackbarPos("V min",t.controls,t.settings.vMin);
}
// 鼠标左键点击某侧原图时取色；data 指向该侧 Tuning，其他窗口区域点击无效。
void sampleColor(int event,int x,int y,int,void* data){
    auto& t=*static_cast<Tuning*>(data);
    if(event!=cv::EVENT_LBUTTONDOWN || t.image.empty())return;
    const double scale=double(t.width)/t.image.cols;
    // 只接受原图显示区域，忽略状态栏及下方掩码；把预览坐标反算到原图坐标。
    if(y<statusHeight || y>=statusHeight+cvRound(t.image.rows*scale))return;
    int u=std::clamp(cvRound(x/scale),0,t.image.cols-1);
    int v=std::clamp(cvRound((y-statusHeight)/scale),0,t.image.rows-1);
    // 取点击附近约 11×11 原图像素并裁到图像内，降低单像素噪声的影响。
    cv::Rect patch(std::max(0,u-5),std::max(0,v-5),11,11);
    patch &= cv::Rect(0,0,t.image.cols,t.image.rows);
    cv::Mat hsv;cv::cvtColor(t.image(patch),hsv,cv::COLOR_BGR2HSV);
    // 分别收集 H/S/V 并排序求中位数，比单点读数更稳健；这里不求圆心。
    std::vector<int> a[3];for(int row=0;row<hsv.rows;++row)for(int col=0;col<hsv.cols;++col){auto p=hsv.at<cv::Vec3b>(row,col);for(int k=0;k<3;++k)a[k].push_back(p[k]);}
    int median[3];for(int k=0;k<3;++k){std::sort(a[k].begin(),a[k].end());median[k]=a[k][a[k].size()/2];}
    std::cout<<t.controls<<" sampled HSV="<<median[0]<<","<<median[1]<<","<<median[2]<<std::endl;
    // 对自动取色做宽松绿色检查：明显非绿、过灰或过暗时保留原参数并提示重选。
    if(median[0]<30 || median[0]>100 || median[1]<35 || median[2]<10){std::cout<<"Sample is not a usable green pixel; click inside the green target.\n";return;}
    // 取色起点：H 中位数±12，S/V 下限约为中位数的一半，并保留最低门限；仍需看掩码微调。
    t.settings.hMin=std::max(0,median[0]-12);t.settings.hMax=std::min(179,median[0]+12);
    t.settings.sMin=std::max(35,median[1]/2);t.settings.vMin=std::max(10,median[2]/2);
    t.reset();syncControls(t);
}
// 按 left_ 或 right_ 前缀加载独立参数，允许左右白平衡和成像亮度不同。
void readTuning(const cv::FileStorage& f,const std::string& prefix,DetectorSettings& d){
    read(f,(prefix+"h_min").c_str(),d.hMin);read(f,(prefix+"h_max").c_str(),d.hMax);
    read(f,(prefix+"s_min").c_str(),d.sMin);read(f,(prefix+"v_min").c_str(),d.vMin);
    read(f,(prefix+"min_area_px").c_str(),d.minArea);read(f,(prefix+"min_circularity").c_str(),d.minCircularity);
}
// 按 w 将两侧参数保存到 hsv_overrides.yaml；下次启动优先加载，不覆盖完整配置文件。
void saveTuning(const fs::path& path,const Tuning& left,const Tuning& right){
    cv::FileStorage out(path.string(),cv::FileStorage::WRITE);
    if(!out.isOpened())throw std::runtime_error("Cannot write HSV overrides");
    for(const auto& entry:std::vector<std::pair<std::string,const Tuning*>>{{"left_",&left},{"right_",&right}}){
        const auto& d=entry.second->settings;const auto& p=entry.first;
        out<<p+"h_min"<<d.hMin<<p+"h_max"<<d.hMax<<p+"s_min"<<d.sMin<<p+"v_min"<<d.vMin;
        out<<p+"min_area_px"<<d.minArea<<p+"min_circularity"<<d.minCircularity;
    }
    std::cout<<"Saved HSV settings to "<<path<<std::endl;
}
// 仅调整当前进程的桌面环境；解决 VS Code/Snap 继承的 GTK/schema 与系统显示后端不兼容。
void prepareDisplay(){
#ifdef __linux__
    // 有 X11 显示时在 SDK 初始化前选择 x11，因为 SDK 本身也可能初始化图形库。
    if(std::getenv("DISPLAY")){
        setenv("GDK_BACKEND","x11",1);
        unsetenv("GTK_PATH");unsetenv("GTK_MODULES");unsetenv("GIO_EXTRA_MODULES");
        unsetenv("GSETTINGS_SCHEMA_DIR");unsetenv("GSETTINGS_SCHEMA_PATH");
        setenv("XDG_DATA_DIRS","/usr/local/share:/usr/share",1);
    }
#endif
}
// 优先按序列号绑定相机；只有未填写序列号时才按枚举下标选择。
Device choose(const std::vector<Device>& devices,const std::string& serial,int index){
    if(!serial.empty()){
        for(auto& d:devices) if(d.serial==serial) return d;
        throw std::runtime_error("Configured serial not found: "+serial);
    }
    if(index<0 || index>=int(devices.size())) throw std::runtime_error("Need two cameras; use --list to diagnose");
    return devices[index];
}
}
// 主程序入口；异常统一在最外层输出并返回非零退出码，硬件由析构函数释放。
int main(int argc,char** argv){
    try{
        // 命令行支持实时模式、无窗口模式、保存 CSV、限帧和左右原图离线回放。
        cv::CommandLineParser args(argc,argv,
            "{help h||Help}{config|stereo_config.yaml|Configuration path}{list||List cameras only}"
            "{headless||No GUI}{frames|0|Stop after N frame pairs (0 unlimited)}"
            "{save-first||Save first raw pair and annotated previews}{csv||Optional CSV path}"
            "{left-image||Replay left image without cameras}{right-image||Replay right image without cameras}");
        if(args.has("help")){args.printMessage();return 0;}
        if(!args.check()){args.printErrors();return 1;}
        // 离线回放必须同时提供左右图片；默认显示黑白掩码，headless 则完全不创建窗口。
        const bool replay=args.has("left-image") || args.has("right-image");
        if(replay && (!args.has("left-image") || !args.has("right-image")))throw std::runtime_error("Provide both replay images");
        bool headless=args.has("headless"),showMasks=true;
        if(!headless && !args.has("list"))prepareDisplay();
        // SDK 比相机对象先声明、后销毁；离线回放不初始化 SDK，也不占用真实相机。
        std::unique_ptr<MvsSdk> sdk;
        std::vector<Device> devices;
        if(!replay){
            sdk=std::make_unique<MvsSdk>();devices=sdk->enumerate();
            for(size_t n=0;n<devices.size();++n)std::cout<<"["<<n<<"] serial="<<devices[n].serial<<" model="<<devices[n].model<<std::endl;
        }
        // --list 只列出设备并退出，不加载标定、不进入采集循环。
        if(args.has("list")){if(devices.empty()){std::cerr<<"No cameras found.\n";return 2;}return 0;}
        // 配置文件转为绝对路径；其中内参、外参、保存目录的相对路径均相对配置所在目录。
        fs::path config=fs::absolute(args.get<std::string>("config"));
        cv::FileStorage f(config.string(),cv::FileStorage::READ);
        if(!f.isOpened()) throw std::runtime_error("Cannot open "+config.string());
        auto resolve=[&](const std::string& path){return fs::path(path).is_absolute()?fs::path(path):config.parent_path()/path;};
        // 先设缺省变量，再逐项读取配置；真正左右对应关系以配置中的序列号和内参路径为准。
        std::string leftFile="calibration(5-50).yaml",rightFile="calibration2.yaml",extrinsics,leftSerial,rightSerial;
        std::string saveDir="captures";
        int leftIndex=0,rightIndex=1,previewWidth=540;
        double exposure=0,minAngle=0.02,baseline=0;
        DetectorSettings detector;
        // 加载相机身份、几何模型、曝光（μs）、预览尺寸、HSV 和筛选阈值。
        read(f,"left_intrinsics",leftFile);read(f,"right_intrinsics",rightFile);
        read(f,"left_serial",leftSerial);read(f,"right_serial",rightSerial);
        read(f,"left_index",leftIndex);read(f,"right_index",rightIndex);
        read(f,"extrinsics",extrinsics);read(f,"parallel_baseline_mm",baseline);
        read(f,"exposure_us",exposure);read(f,"preview_width",previewWidth);read(f,"save_directory",saveDir);
        read(f,"min_ray_angle_deg",minAngle);
        read(f,"h_min",detector.hMin);read(f,"h_max",detector.hMax);
        read(f,"s_min",detector.sMin);read(f,"v_min",detector.vMin);
        read(f,"min_area_px",detector.minArea);read(f,"max_area_fraction",detector.maxAreaFraction);
        read(f,"min_circularity",detector.minCircularity);read(f,"morphology_size",detector.morphologySize);
        // 检查参数范围，防止非法 HSV、偶数形态学核、无效基线或不合理预览尺寸进入算法。
        if(previewWidth<320 || previewWidth>3000 || detector.hMin<0 || detector.hMax>179 || detector.hMin>detector.hMax ||
           detector.sMin<0 || detector.sMin>255 || detector.vMin<0 || detector.vMin>255 ||
           detector.minArea<=0 || detector.maxAreaFraction<=0 || detector.maxAreaFraction>1 ||
           detector.minCircularity<0 || detector.minCircularity>1 || detector.morphologySize<1 || detector.morphologySize>31 ||
           detector.morphologySize%2==0 || minAngle<=0 || !std::isfinite(baseline) || baseline<0)
            throw std::runtime_error("Invalid configuration values");
        // 分别读取两台相机内参；可用真实外参时优先使用，否则根据机械基线构造近似模型。
        auto kl=loadIntrinsics(resolve(leftFile)),kr=loadIntrinsics(resolve(rightFile));
        std::optional<StereoGeometry> geometry;
        if(!extrinsics.empty()) geometry=loadGeometry(resolve(extrinsics));
        else if(baseline>0) {
            // 平行假设：右光心位于左坐标系 (B,0,0)，所以 R=I、T=(-B,0,0)。
            // 基线大小控制距离尺度，不能补偿光轴角度、上下错位或内参误差。
            geometry=StereoGeometry{cv::Mat::eye(3,3,CV_64F),(cv::Mat_<double>(3,1)<<-baseline,0,0),true};
            std::cerr<<"APPROXIMATE: assuming parallel axes and right camera at +X, baseline="<<baseline<<" mm.\n";
        }
        else std::cerr<<"NO STEREO EXTRINSICS: detection only; distances unavailable.\n";
        // 实时模式打开相机；离线模式只保留配置中的身份信息，用同一套算法处理图片。
        Device dl{},dr{};
        dl.serial=leftSerial;dr.serial=rightSerial;
        std::unique_ptr<Camera> left,right;
        if(!replay){
            dl=choose(devices,leftSerial,leftIndex);dr=choose(devices,rightSerial,rightIndex);
            if(dl.serial==dr.serial)throw std::runtime_error("Left/right must be different cameras");
            // 有标定文件时核对其序列号与当前左右顺序，防止把标定关系套到错误相机上。
            if(!extrinsics.empty()){
                cv::FileStorage ef(resolve(extrinsics).string(),cv::FileStorage::READ);
                std::string el,er;read(ef,"left_serial",el);read(ef,"right_serial",er);
                if((!el.empty() && el!=dl.serial) || (!er.empty() && er!=dr.serial))throw std::runtime_error("Extrinsics camera order differs");
            }
            std::cout<<"LEFT "<<dl.serial<<" <- "<<leftFile<<"\nRIGHT "<<dr.serial<<" <- "<<rightFile<<std::endl;
            left=std::make_unique<Camera>(dl,exposure);right=std::make_unique<Camera>(dr,exposure);
        }
        // 参数优先级：程序缺省 < YAML 公共值 < YAML 左右值 < hsv_overrides.yaml；运行中滑条再修改。
        Tuning tuningL,tuningR;
        tuningL.settings=detector;tuningR.settings=detector;
        tuningL.controls="Left HSV controls";tuningR.controls="Right HSV controls";
        tuningL.width=tuningR.width=previewWidth;
        readTuning(f,"left_",tuningL.settings);readTuning(f,"right_",tuningR.settings);
        auto overridePath=resolve("hsv_overrides.yaml");
        if(fs::exists(overridePath)){
            cv::FileStorage overrides(overridePath.string(),cv::FileStorage::READ);
            if(!overrides.isOpened())throw std::runtime_error("Cannot open HSV overrides");
            readTuning(overrides,"left_",tuningL.settings);readTuning(overrides,"right_",tuningR.settings);
        }
        // 验证左右独立阈值；圆度转为整数百分数供 UI 使用。
        for(auto* t:{&tuningL,&tuningR}){
            const auto& d=t->settings;
            if(d.hMin<0 || d.hMax>179 || d.hMin>d.hMax || d.sMin<0 || d.sMin>255 || d.vMin<0 || d.vMin>255 ||
               !std::isfinite(d.minArea) || d.minArea<=0 || d.minCircularity<0 || d.minCircularity>1)throw std::runtime_error("Invalid per-camera HSV settings");
            t->circularity=cvRound(d.minCircularity*100);
        }
        // 离线图片只读取一次；连续回放同一对图便于重复调参，不表示真实相机时间序列。
        cv::Mat replayL,replayR;
        if(replay){replayL=cv::imread(args.get<std::string>("left-image"));replayR=cv::imread(args.get<std::string>("right-image"));
            if(replayL.empty() || replayR.empty())throw std::runtime_error("Cannot load replay pair");}
        // 实时默认无限运行；无窗口离线默认只算一对，避免无人值守时反复处理同一图。
        int limit=args.get<int>("frames");
        if(replay && headless && limit==0)limit=1;
        if(limit<0) throw std::runtime_error("frames must be >= 0");
        // CSV 可选输出；列中的像素基于原图，所有距离以 mm 保存，界面距离则以 m 显示。
        std::ofstream csv;
        auto csvPath=args.get<std::string>("csv");
        if(!csvPath.empty()){
            csv.open(csvPath);if(!csv) throw std::runtime_error("Cannot write CSV");
            csv<<"pair,time_s,left_u,left_v,right_u,right_v,valid,baseline_distance_mm,left_distance_mm,right_distance_mm,left_z_mm,reprojection_px,ray_angle_deg,trigger_gap_ms,status\n";
            csv<<std::setprecision(12);
        }
        // 创建左右画面与独立 HSV 窗口，安装取色回调，并设置初始窗口位置。
        if(!headless){
            for(const auto& name:{"Left camera","Right camera"})cv::namedWindow(name,cv::WINDOW_AUTOSIZE);
            for(auto* t:{&tuningL,&tuningR}){
                cv::namedWindow(t->controls,cv::WINDOW_NORMAL);cv::resizeWindow(t->controls,420,300);
                slider("H min",t->controls,t->settings.hMin,179);slider("H max",t->controls,t->settings.hMax,179);
                slider("S min",t->controls,t->settings.sMin,255);slider("V min",t->controls,t->settings.vMin,255);
                slider("Circularity %",t->controls,t->circularity,100);
            }
            cv::setMouseCallback("Left camera",sampleColor,&tuningL);cv::setMouseCallback("Right camera",sampleColor,&tuningR);
            cv::moveWindow("Left camera",0,0);cv::moveWindow("Right camera",previewWidth+15,0);
            cv::moveWindow(tuningL.controls,previewWidth*2+30,0);cv::moveWindow(tuningR.controls,previewWidth*2+30,360);
        }
        // 注册 Ctrl+C/终止信号，记录启动时间；主循环发现退出标记后正常释放资源。
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);
        std::cout<<"Keys (click a camera window first): q/ESC quit | m masks | s snapshot | w save HSV | click green to sample\n";
        auto start=Clock::now(),lastReport=start;
        fs::path saved=resolve(saveDir);
        int count=0;
        // 每轮产生一对观测：采集 → 参数更新 → 左右检测 → 三角化 → UI/存盘；不复用旧距离。
        while(!stopRequested && (limit==0 || count<limit)){
            cv::Mat il,ir;double gap=0;
            // 离线直接取已读图片；实时先触发左右两机，再并行等待两帧，减少串行等待。
            if(replay){il=replayL;ir=replayR;}
            else{
                auto t0=Clock::now();left->trigger();right->trigger();auto t2=Clock::now();
                // 这里只测两次软件触发调用的总耗时，不是曝光时间差，不能代表硬件同步精度。
                gap=std::chrono::duration<double,std::milli>(t2-t0).count();
                auto future=std::async(std::launch::async,[&]{return left->grab();});
                ir=right->grab();il=future.get();
            }
            // 分辨率不符合内参时立即停止；保存当前原图引用供点击取色使用。
            checkImageSize(kl,il.size());checkImageSize(kr,ir.size());
            tuningL.image=il;tuningR.image=ir;
            // ml/mr 为左右 0/255 掩码；infoL/infoR 是当前检测诊断；tl/tr 为当前目标（可为空）。
            cv::Mat ml,mr;DetectionInfo infoL,infoR;
            std::optional<Target> tl,tr;
            for(auto* t:{&tuningL,&tuningR}){
                // 每帧把圆度滑条换成 [0,1]；H 范围反向时交换两值并同步滑条。
                t->settings.minCircularity=t->circularity/100.0;
                if(t->settings.hMin>t->settings.hMax){std::swap(t->settings.hMin,t->settings.hMax);if(!headless)syncControls(*t);}
                const auto& d=t->settings;
                // 若用户改变阈值，清除旧的跟踪先验，避免旧目标约束妨碍观察新阈值效果。
                std::vector<double> current{double(d.hMin),double(d.hMax),double(d.sMin),double(d.vMin),d.minCircularity,d.minArea};
                if(current!=t->lastSettings){t->reset();t->lastSettings=std::move(current);}
            }
            // 左右分别检测，用各自上一目标维持轮廓连续性；只有当前帧检测结果参与测距。
            tl=detectGreen(il,tuningL.settings,ml,&infoL,tuningL.previous?&*tuningL.previous:nullptr);
            tr=detectGreen(ir,tuningR.settings,mr,&infoR,tuningR.previous?&*tuningR.previous:nullptr);
            // 记录有效目标；连续丢失三帧后清空跟踪记忆，下一帧允许重新捕获。
            auto remember=[](Tuning& t,const std::optional<Target>& target){
                if(target){t.previous=target;t.misses=0;}else if(++t.misses>=3)t.reset();
            };
            remember(tuningL,tl);remember(tuningR,tr);
            // 每帧先设无效：缺目标或缺几何参数都显示 N/A；两侧圆心齐全时才进行三角化。
            RangeResult result;
            // 明确指出缺失哪一侧，便于把几何失败与单侧 HSV 漏检区别开。
            result.reason=!geometry?"NO EXTRINSICS - detection only":
                (!tl && !tr?"Target missing: LEFT and RIGHT":(!tl?"Target missing: LEFT":"Target missing: RIGHT"));
            if(tl && tr && geometry) result=triangulateTarget(tl->center,tr->center,kl,kr,*geometry,minAngle);
            double seconds=std::chrono::duration<double>(Clock::now()-start).count();
            ++count;
            std::string mode=geometry?(geometry->approximate?"APPROX parallel geometry":"Calibrated geometry"):"NO EXTRINSICS";
            // 生成各窗口状态文字：身份、模型、圆心、到基线距离或失败原因、触发耗时、轮廓诊断和按键。
            auto lines=[&](bool isLeft,const std::optional<Target>& target){
                std::vector<std::string> s{std::string(isLeft?"LEFT ":"RIGHT ")+(isLeft?dl.serial:dr.serial),mode};
                s.push_back(target?"u="+number(target->center.x)+" v="+number(target->center.y)+" px":"Green circle: NOT FOUND");
                // 主显示距离是到两光心直线的垂距，同一个目标在两窗口的数值相同；mm 除 1000 转为 m。
                s.push_back(result.valid?"Distance to baseline: "+number(result.baselineDistance/1000,4)+" m":"Distance to baseline: N/A");
                s.push_back(result.valid?"Left Z="+number(result.leftPoint.z/1000,4)+" m, reproj="+number(result.reprojection)+" px":result.reason);
                s.push_back("Software trigger span="+number(gap)+" ms (not exposure sync)");
                const auto& info=isLeft?infoL:infoR;
                s.push_back(info.status+"; candidates="+std::to_string(info.accepted));
                s.push_back("Click green: sample | m mask | w save HSV | s snapshot");return s;
            };
            // 先画缩略图和状态栏，随后可把掩码拼在下方；不改变算法输入。
            cv::Mat pl=preview(il,tl,lines(true,tl),previewWidth),pr=preview(ir,tr,lines(false,tr),previewWidth);
            int key=-1;
            if(!headless){
                // 掩码用最近邻缩放，保持 0/255 黑白；转成三通道只是为了与彩色预览拼接。
                auto panel=[&](const cv::Mat& p,const cv::Mat& mask){
                    if(!showMasks)return p;
                    cv::Mat small,bgr,out;cv::resize(mask,small,{p.cols,cvRound(double(p.cols)*mask.rows/mask.cols)},0,0,cv::INTER_NEAREST);
                    cv::cvtColor(small,bgr,cv::COLOR_GRAY2BGR);cv::vconcat(p,bgr,out);return out;
                };
                cv::imshow("Left camera",panel(pl,ml));cv::imshow("Right camera",panel(pr,mr));
                // 处理图形窗口事件并读取按键；需要让窗口获得焦点，终端中的 m 不会传给此窗口。
                key=cv::waitKey(20)&0xff;
                if(key>='A' && key<='Z')key+='a'-'A';
                // GTK 可能用 -1 表示不支持可见性查询，不能把 -1 当关闭；启动几帧后仅在明确返回 0 时退出。
                if(count>5 && (cv::getWindowProperty("Left camera",cv::WND_PROP_VISIBLE)==0 ||
                   cv::getWindowProperty("Right camera",cv::WND_PROP_VISIBLE)==0)) break;
            }
            // 首帧自动保存或按 s 保存；先核对保存目录中的左右身份，避免混用标定数据。
            if((args.has("save-first") && count==1) || key=='s'){
                if(fs::exists(saved/"camera_mapping.yaml")){
                    cv::FileStorage previous((saved/"camera_mapping.yaml").string(),cv::FileStorage::READ);
                    std::string sl,sr;read(previous,"left_serial",sl);read(previous,"right_serial",sr);
                    if(sl!=dl.serial || sr!=dr.serial) throw std::runtime_error("Capture directory contains another camera order; choose a new save_directory");
                }
                // 左右原图用同一微秒时间戳命名；该时间戳是保存时刻，不是曝光同步证明。
                fs::create_directories(saved/"left");fs::create_directories(saved/"right");
                auto stamp=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                auto name=std::to_string(stamp)+".png";
                // 保存无标注原图用于标定；预览、掩码和相机映射另存，用于诊断。
                if(!cv::imwrite((saved/"left"/name).string(),il) || !cv::imwrite((saved/"right"/name).string(),ir) ||
                   !cv::imwrite((saved/"left_preview.png").string(),pl) || !cv::imwrite((saved/"right_preview.png").string(),pr))
                    throw std::runtime_error("Failed to save image pair");
                cv::imwrite((saved/"left_mask.png").string(),ml);cv::imwrite((saved/"right_mask.png").string(),mr);
                cv::FileStorage meta((saved/"camera_mapping.yaml").string(),cv::FileStorage::WRITE);
                meta<<"left_serial"<<dl.serial<<"right_serial"<<dr.serial<<"left_intrinsics"<<leftFile<<"right_intrinsics"<<rightFile;
                std::cout<<"Saved pair "<<name<<" in "<<saved<<std::endl;
            }
            // 逐帧写 CSV：无圆心/不可计算时留空；valid=1 表示可计算的估计，重投影残差仅记录不筛选。
            if(csv){
                csv<<count<<','<<seconds<<',';
                if(tl) csv<<tl->center.x<<','<<tl->center.y;else csv<<',';
                csv<<',';if(tr)csv<<tr->center.x<<','<<tr->center.y;else csv<<',';
                csv<<','<<int(result.valid)<<',';
                if(result.valid)csv<<result.baselineDistance<<','<<result.leftDistance<<','<<result.rightDistance<<','<<result.leftPoint.z<<','<<result.reprojection<<','<<result.rayAngleDegrees;
                else csv<<",,,,,";
                csv<<','<<gap<<','<<result.reason<<'\n';
            }
            // 终端最多约每秒报告一次，避免刷屏；帧计算和 CSV 保存仍然每帧执行。
            if(count==1 || Clock::now()-lastReport>std::chrono::seconds(1)){
                std::cout<<"Pair "<<count<<" L="<<(tl?number(tl->center.x)+","+number(tl->center.y):"missing")
                    <<" R="<<(tr?number(tr->center.x)+","+number(tr->center.y):"missing")<<" | "<<result.reason
                    <<" | L: "<<infoL.status<<" | R: "<<infoR.status;
                if(result.valid)std::cout<<" | distance_to_baseline(m)="<<result.baselineDistance/1000
                    <<" | reproj(px)="<<result.reprojection;
                std::cout<<std::endl;lastReport=Clock::now();
            }
            // q/Esc 退出；m 切换掩码；w 持久化当前 HSV 参数。
            if(key=='q' || key==27)break;
            if(key=='m'){showMasks=!showMasks;std::cout<<"Masks "<<(showMasks?"ON":"OFF")<<std::endl;}
            if(key=='w')saveTuning(overridePath,tuningL,tuningR);
        }
        // 循环结束关闭图形窗口；函数返回后相机和 SDK 按生命周期自动清理。
        if(!headless)cv::destroyAllWindows();
        std::cout<<"Completed "<<count<<" frame pairs.\n";return 0;
    }catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<std::endl;return 1;}
}
