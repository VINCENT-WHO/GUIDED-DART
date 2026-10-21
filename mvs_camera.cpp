// 工业相机取流封装：管理 MVS 生命周期、软件触发和像素格式转换。
#include "mvs_camera.hpp"
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <iostream>
namespace {
// SDK 返回 MV_OK 表示成功；失败时抛出包含操作名和十六进制错误码的异常。
void check(int code, const std::string& operation) {
    if(code==MV_OK) return;
    std::ostringstream s; s<<operation<<" failed, MVS 0x"<<std::hex<<static_cast<unsigned>(code);
    throw std::runtime_error(s.str());
}
// 查询实际固件支持的触发项，不能假定所有型号都支持 FrameStart。
std::string configureSoftwareTrigger(void* handle) {
    // 读出 TriggerSelector 可选值，并校验返回数量，避免访问数组越界。
    MVCC_ENUMVALUE selectors{};
    check(MV_CC_GetEnumValue(handle,"TriggerSelector",&selectors),"Read supported TriggerSelector values");
    if(selectors.nSupportedNum>MV_MAX_XML_SYMBOLIC_NUM)
        throw std::runtime_error("Invalid TriggerSelector enumeration length");
    struct Entry { unsigned value; std::string name; };
    std::vector<Entry> entries;
    std::string available;
    // 将 SDK 枚举数值转换成可读名称，例如 FrameBurstStart；保留数值用于设置。
    for(unsigned n=0;n<selectors.nSupportedNum;++n){
        MVCC_ENUMENTRY entry{};entry.nValue=selectors.nSupportValue[n];
        check(MV_CC_GetEnumEntrySymbolic(handle,"TriggerSelector",&entry),"Read TriggerSelector symbolic name");
        entries.push_back({entry.nValue,entry.chSymbolic});
        if(!available.empty())available+=", ";
        available+=entry.chSymbolic;
    }
    // 优先逐帧触发 FrameStart，否则使用 FrameBurstStart；都不支持则明确报错。
    const Entry* selected=nullptr;
    for(const char* preferred:{"FrameStart","FrameBurstStart"}){
        for(const auto& entry:entries)if(entry.name==preferred){selected=&entry;break;}
        if(selected)break;
    }
    if(!selected)throw std::runtime_error("No supported single-frame software trigger; TriggerSelector values: "+available);
    std::cout<<"TriggerSelector supported: "<<available<<"; using "<<selected->name<<std::endl;
    // 先关闭其他触发门，防止 MVS 预览或其他程序留下的触发设置阻断采集。
    for(const auto& entry:entries){
        check(MV_CC_SetEnumValue(handle,"TriggerSelector",entry.value),"Select trigger gate "+entry.name);
        check(MV_CC_SetEnumValueByString(handle,"TriggerMode","Off"),"Disable trigger gate "+entry.name);
    }
    // 选中需要使用的触发门，采集模式设为 Continuous，允许反复发送软件触发。
    check(MV_CC_SetEnumValue(handle,"TriggerSelector",selected->value),"Select "+selected->name);
    check(MV_CC_SetEnumValueByString(handle,"AcquisitionMode","Continuous"),"Continuous acquisition mode");
    // 突发触发必须设为每次 1 帧，保证左右每次请求只对应一帧，不混入旧帧。
    if(selected->name=="FrameBurstStart")
        check(MV_CC_SetIntValueEx(handle,"AcquisitionBurstFrameCount",1),"One frame per software-triggered burst");
    // 触发源设为 Software 并打开触发模式；此时还没有开始取流。
    check(MV_CC_SetEnumValueByString(handle,"TriggerSource","Software"),"Software trigger source");
    check(MV_CC_SetEnumValueByString(handle,"TriggerMode","On"),"Trigger on");
    return selected->name;
}
}
// RAII：构造时初始化 SDK，析构时释放；局部对象退出作用域会自动调用析构函数。
MvsSdk::MvsSdk(){check(MV_CC_Initialize(),"SDK initialize");}
MvsSdk::~MvsSdk(){MV_CC_Finalize();}
// 枚举 USB/GigE 相机并复制设备信息；不同接口的序列号、型号位于不同结构字段。
std::vector<Device> MvsSdk::enumerate(){
    MV_CC_DEVICE_INFO_LIST list{};
    check(MV_CC_EnumDevices(MV_USB_DEVICE|MV_GIGE_DEVICE,&list),"Enumerate cameras");
    std::vector<Device> devices;
    for(unsigned n=0;n<list.nDeviceNum;++n){
        auto d=*list.pDeviceInfo[n];
        auto serial=d.nTLayerType==MV_USB_DEVICE?d.SpecialInfo.stUsb3VInfo.chSerialNumber:d.SpecialInfo.stGigEInfo.chSerialNumber;
        auto model=d.nTLayerType==MV_USB_DEVICE?d.SpecialInfo.stUsb3VInfo.chModelName:d.SpecialInfo.stGigEInfo.chModelName;
        devices.push_back({d,reinterpret_cast<char*>(serial),reinterpret_cast<char*>(model)});
    }
    return devices;
}
// 按顺序创建句柄、打开设备、配置曝光及触发，任何步骤失败都会清理已获取的资源。
Camera::Camera(const Device& device,double exposure){
    try {
        auto info=device.info;
        check(MV_CC_CreateHandle(&handle_,&info),"Create handle "+device.serial);
        check(MV_CC_OpenDevice(handle_),"Open "+device.serial+" (close MVS viewer if busy)"); opened_=true;
        // 只有 GigE 相机需要探测并设置合适的网络包大小，USB 相机跳过此步骤。
        if(info.nTLayerType==MV_GIGE_DEVICE){
            int packet=MV_CC_GetOptimalPacketSize(handle_);
            if(packet>0) check(MV_CC_SetIntValueEx(handle_,"GevSCPSPacketSize",packet),"Packet size");
        }
        check(MV_CC_SetEnumValueByString(handle_,"TriggerMode","Off"),"Trigger off");
        // 先关闭自动曝光，再按微秒写入固定曝光；60000 μs=60 ms。0 表示不改曝光。
        if(exposure>0){
            check(MV_CC_SetEnumValueByString(handle_,"ExposureAuto","Off"),"Exposure auto off");
            check(MV_CC_SetFloatValue(handle_,"ExposureTime",float(exposure)),"Exposure time");
        }
        // 回读真实曝光和增益供终端检查；读取诊断失败不影响之后的采集。
        MVCC_FLOATVALUE exposureValue{},gainValue{};
        std::cout<<"Camera "<<device.serial;
        if(MV_CC_GetFloatValue(handle_,"ExposureTime",&exposureValue)==MV_OK)
            std::cout<<" ExposureTime="<<exposureValue.fCurValue<<" us";
        if(MV_CC_GetFloatValue(handle_,"Gain",&gainValue)==MV_OK)
            std::cout<<" Gain="<<gainValue.fCurValue;
        std::cout<<std::endl;
        // 保持当前分辨率、ROI、binning、像素格式和白平衡；配置触发后设置两个缓存并开始取流。
        auto triggerSelector=configureSoftwareTrigger(handle_);
        check(MV_CC_SetImageNodeNum(handle_,2),"Image nodes");
        check(MV_CC_StartGrabbing(handle_),"Start grabbing"); streaming_=true;
        std::cout<<"Opened "<<device.serial<<" ("<<triggerSelector<<", paired software triggers; NOT hardware synchronized)\n";
    } catch(...) { close(); throw; }
}
// 按依赖逆序释放：先停流，再关闭触发和设备，最后销毁句柄；不持久化保存相机参数。
void Camera::close(){
    if(streaming_) MV_CC_StopGrabbing(handle_);
    if(opened_) { MV_CC_SetEnumValueByString(handle_,"TriggerMode","Off"); MV_CC_CloseDevice(handle_); }
    if(handle_) MV_CC_DestroyHandle(handle_);
    handle_=nullptr; opened_=streaming_=false;
}
// 析构自动释放；trigger 仅发送一次采集命令，主程序会先触发左右，再等待两幅图。
Camera::~Camera(){close();}
void Camera::trigger(){check(MV_CC_SetCommandValue(handle_,"TriggerSoftware"),"TriggerSoftware");}
// 最多等 2000 ms 获取一帧 SDK 缓冲；超时抛异常，不返回旧图冒充新帧。
cv::Mat Camera::grab(){
    MV_FRAME_OUT frame{};
    check(MV_CC_GetImageBuffer(handle_,&frame,2000),"Get frame (USB bandwidth/power, trigger, exposure)");
    // held 记录 SDK 缓冲是否仍需释放，确保成功或异常退出时都只释放一次。
    bool held=true;
    try {
        auto& i=frame.stFrameInfo;
        // 优先用扩展宽高字段，兼容 SDK 的新旧帧信息；图像必须与内参分辨率一致。
        unsigned w=i.nExtendWidth?i.nExtendWidth:i.nWidth, h=i.nExtendHeight?i.nExtendHeight:i.nHeight;
        // 申请自有 BGR8 三通道内存；SDK 的 Bayer/RGB 等格式转换到这里，释放 SDK 帧后图像仍有效。
        cv::Mat bgr(h,w,CV_8UC3);
        MV_CC_PIXEL_CONVERT_PARAM_EX p{};
        // 指定输入尺寸、源像素格式和数据长度，以及目标 BGR8 格式和足够大的输出缓存。
        p.nWidth=w; p.nHeight=h; p.pSrcData=frame.pBufAddr; p.nSrcDataLen=i.nFrameLen;
        p.enSrcPixelType=i.enPixelType; p.enDstPixelType=PixelType_Gvsp_BGR8_Packed;
        p.pDstBuffer=bgr.data; p.nDstBufferSize=static_cast<unsigned>(bgr.total()*3);
        check(MV_CC_ConvertPixelTypeEx(handle_,&p),"Convert frame to BGR8");
        // 格式转换完成后立即归还 SDK 帧，避免缓存用尽；held=false 防止异常处理中重复归还。
        int releaseResult=MV_CC_FreeImageBuffer(handle_,&frame);
        held=false;
        check(releaseResult,"Free image buffer");
        return bgr;
    } catch(...) { if(held) MV_CC_FreeImageBuffer(handle_,&frame); throw; }
}
