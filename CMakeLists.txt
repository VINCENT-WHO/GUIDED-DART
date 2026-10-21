# CMake 3.16 为最低版本；配置阶段会检查版本和必需依赖。
cmake_minimum_required(VERSION 3.16)
# 声明 C++ 工程，并统一使用 C++17（filesystem、optional 等类型需要它）。
project(stereo_green_range LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 只寻找本项目需要的 OpenCV 模块和线程库；缺依赖时在配置阶段直接报错。
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d highgui imgcodecs)
find_package(Threads REQUIRED)
# 可通过 -DMVS_ROOT=其他路径 指定 SDK；默认使用本机 /opt/MVS。
set(MVS_ROOT /opt/MVS CACHE PATH "Hikrobot MVS installation")
find_path(MVS_INCLUDE MvCameraControl.h PATHS ${MVS_ROOT}/include REQUIRED)
find_library(MVS_LIBRARY MvCameraControl PATHS ${MVS_ROOT}/lib/64 ${MVS_ROOT}/lib/aarch64 REQUIRED)

# 核心算法库：不直接依赖相机硬件，实时程序和标定程序均可复用。
add_library(stereo_core stereo_core.cpp)
# PUBLIC 表示使用该库的程序也会继承头文件搜索目录和 OpenCV 链接依赖。
target_include_directories(stereo_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_core PUBLIC ${OpenCV_LIBS})

# 实时程序由 UI/主循环和 MVS 相机封装组成，再链接核心算法、SDK 和线程库。
add_executable(stereo_range main.cpp mvs_camera.cpp)
target_include_directories(stereo_range PRIVATE ${MVS_INCLUDE})
target_link_libraries(stereo_range PRIVATE stereo_core ${MVS_LIBRARY} Threads::Threads)
# 把 SDK 动态库目录写入构建产物的运行时搜索路径，方便直接执行程序。
get_filename_component(MVS_LIBRARY_DIR ${MVS_LIBRARY} DIRECTORY)
set_target_properties(stereo_range PROPERTIES BUILD_RPATH ${MVS_LIBRARY_DIR})

# 独立外参标定工具只读已有图片，不需要直接链接相机 SDK。
add_executable(stereo_calibrate stereo_calibrate.cpp)
target_link_libraries(stereo_calibrate PRIVATE stereo_core)
# 对主要编译目标开启常见警告，帮助发现参数、类型和标准兼容问题。
foreach(t stereo_core stereo_range stereo_calibrate)
  target_compile_options(${t} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
