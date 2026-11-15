#include "vision_link.hpp"
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <utility>
VisionLink::VisionLink(std::string port):port_(std::move(port)){}
VisionLink::~VisionLink(){disconnect();}
void VisionLink::disconnect()
{
    if(fd_>=0)::close(fd_);
    fd_=-1;haveYaw_=false;pendingSize_=pendingOffset_=0;parser_.count=0;
}
void VisionLink::flush()
{
    while(fd_>=0 && pendingSize_){
        ssize_t n=::write(fd_,pending_+pendingOffset_,pendingSize_-pendingOffset_);
        if(n>0){pendingOffset_+=size_t(n);if(pendingOffset_==pendingSize_)pendingSize_=pendingOffset_=0;}
        else if(n<0 && errno==EINTR)continue;
        else if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK))return;
        else {disconnect();return;}
    }
}
void VisionLink::poll()
{
    if(!enabled())return;
    auto now=Clock::now();
    if(fd_<0){
        if(lastAttempt_!=Clock::time_point{} && now-lastAttempt_<std::chrono::seconds(1))return;
        lastAttempt_=now;
        fd_=::open(port_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
        if(fd_<0){std::cerr<<"Vision USB waiting: "<<port_<<": "<<std::strerror(errno)<<'\n';return;}
        termios settings{};
        if(tcgetattr(fd_,&settings)!=0){disconnect();return;}
        cfmakeraw(&settings);settings.c_cflag|=CLOCAL|CREAD;
#ifdef CRTSCTS
        settings.c_cflag&=~CRTSCTS;
#endif
        settings.c_cc[VMIN]=0;settings.c_cc[VTIME]=0;
        cfsetispeed(&settings,B115200);cfsetospeed(&settings,B115200);
        if(tcsetattr(fd_,TCSANOW,&settings)!=0){disconnect();return;}
        tcflush(fd_,TCIOFLUSH);
    }
    // Drop pending commands if live encoder feedback reception has stopped. Partial packets are
    // abandoned by reconnecting, allowing both parsers to resynchronize.
    if(pendingSize_ && (!haveYaw_ || now-lastYaw_>std::chrono::milliseconds(150))){disconnect();return;}
    flush();if(fd_<0)return;
    uint8_t bytes[256];
    for(unsigned budget=0;budget<32;++budget){
        ssize_t n=::read(fd_,bytes,sizeof(bytes));
        if(n>0){for(ssize_t i=0;i<n;++i){
            float yaw;if(!DVPushYawFeedback(&parser_,bytes[i],&yaw))continue;
            if(!std::isfinite(yaw) || std::abs(yaw)>6.0f){haveYaw_=false;continue;}
            yawDegrees_=yaw;haveYaw_=true;lastYaw_=now;
        }}
        else if(n<0 && errno==EINTR)continue;
        else if(n==0 || (n<0 && (errno==EAGAIN || errno==EWOULDBLOCK)))break;
        else {disconnect();return;}
    }
}
std::optional<float> VisionLink::yawDegrees() const
{
    if(!haveYaw_ || Clock::now()-lastYaw_>std::chrono::milliseconds(150))return {};
    return yawDegrees_;
}
bool VisionLink::sendYaw(float target)
{
    poll();
    if(fd_<0 || pendingSize_ || !yawDegrees() || !std::isfinite(target) || std::abs(target)>6.0f)return false;
    DVEncodeYaw(target,pending_);pendingSize_=DV_YAW_SIZE;pendingOffset_=0;flush();return fd_>=0;
}
std::string VisionLink::summary() const
{
    if(!enabled())return "disabled";
    auto yaw=yawDegrees();
    if(!yaw)return "waiting for encoder yaw";
    return "Encoder mechanical yaw="+std::to_string(*yaw)+" deg";
}
