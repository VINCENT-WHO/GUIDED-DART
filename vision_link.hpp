#pragma once
#include "dart_vision_protocol.h"
#include <chrono>
#include <optional>
#include <string>
// Thread ownership: call only from the camera/main loop. No wire metadata beyond current/target yaw.
class VisionLink {
public:
    explicit VisionLink(std::string port);
    ~VisionLink();
    VisionLink(const VisionLink&)=delete;
    VisionLink& operator=(const VisionLink&)=delete;
    void poll();
    std::optional<float> yawDegrees() const;
    bool sendYaw(float absoluteMechanicalDegrees);
    bool enabled() const { return !port_.empty(); }
    std::string summary() const;
private:
    using Clock=std::chrono::steady_clock;
    std::string port_;
    int fd_=-1;
    DVParser parser_{};
    float yawDegrees_=0;
    bool haveYaw_=false;
    Clock::time_point lastAttempt_{},lastYaw_{};
    uint8_t pending_[DV_YAW_SIZE]{};
    size_t pendingSize_=0,pendingOffset_=0;
    void disconnect();
    void flush();
};
