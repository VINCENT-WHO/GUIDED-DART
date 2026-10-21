#pragma once  // 避免重复包含本头文件。
#include "MvCameraControl.h"  // 海康 MVS SDK 的设备、触发、取流接口。
#include <opencv2/core.hpp>   // 返回 OpenCV 图像矩阵。
#include <string>
#include <vector>

// 复制枚举结果并保留序列号、型号；用序列号固定左右关系，避免 USB 枚举顺序变化。
struct Device { MV_CC_DEVICE_INFO info; std::string serial, model; };

// SDK 生命周期管理：构造时初始化，析构时释放；必须比 Camera 对象活得更久。
struct MvsSdk {
    MvsSdk();
    ~MvsSdk();
    std::vector<Device> enumerate();  // 搜索 USB/GigE 工业相机；不在此处打开设备。
};

// 单台相机的资源封装：打开 → 配置 → 开始采集；离开作用域时自动停止并关闭。
class Camera {
public:
    Camera(const Device& device, double exposureUs);  // 曝光单位 μs，0 保留相机当前曝光。
    ~Camera();
    Camera(const Camera&)=delete;             // 禁止复制硬件句柄，避免两个对象重复释放。
    Camera& operator=(const Camera&)=delete;
    void trigger();                          // 发出一次软件触发，不在这里等待图像。
    cv::Mat grab();                          // 等待一帧并转成独立拥有内存的 BGR8 图像。
private:
    void* handle_=nullptr;                   // SDK 设备句柄，仅本对象负责释放。
    bool opened_=false, streaming_=false;    // 记录初始化进度，支持中途失败时清理。
    void close();                            // 按“停止采集 → 关闭设备 → 销毁句柄”的顺序释放。
};
