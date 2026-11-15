#include "vision_link.hpp"
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>
namespace { volatile std::sig_atomic_t stopped=0;void stop(int){stopped=1;} }
int main(int argc,char **argv)
{
    try{
        std::string port;std::optional<float> target;
        for(int i=1;i<argc;++i){
            std::string arg=argv[i];
            if(arg=="--help"){
                std::cout<<"vision_yaw_probe --port /dev/ttyACM0 [--yaw-deg 0.1]\n"
                    "Default: monitor encoder mechanical yaw only. --yaw-deg: ONE absolute mechanical yaw target, +/-6 deg.\n";
                return 0;
            }
            if(i+1>=argc)throw std::runtime_error("Missing argument value");
            if(arg=="--port")port=argv[++i];
            else if(arg=="--yaw-deg"){
                std::string value=argv[++i];size_t consumed=0;target=std::stof(value,&consumed);
                if(consumed!=value.size())throw std::runtime_error("Invalid yaw angle");
            }else throw std::runtime_error("Unknown option: "+arg);
        }
        if(port.empty() || (target && (!std::isfinite(*target) || std::abs(*target)>6)))
            throw std::runtime_error("Provide --port; absolute yaw must be finite and within +/-6 deg");
        VisionLink link(port);bool sent=false;
        std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
        auto report=std::chrono::steady_clock::now();
        while(!stopped){
            link.poll();
            if(target && !sent){sent=link.sendYaw(*target);if(sent)std::cout<<"Sent ONE absolute yaw target: "<<*target<<" deg\n";}
            auto now=std::chrono::steady_clock::now();
            if(now-report>=std::chrono::milliseconds(500)){std::cout<<link.summary()<<std::endl;report=now;}
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
