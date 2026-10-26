cmake_minimum_required(VERSION 3.22)
project(pc_hikrobot_blue_armor_detector)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK installation directory")
find_path(MVS_INCLUDE_DIR MvCameraControl.h
  PATHS "${MVS_ROOT}/include"
  NO_DEFAULT_PATH
)
find_library(MVS_CAMERA_CONTROL_LIBRARY MvCameraControl
  PATHS "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib"
  NO_DEFAULT_PATH
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_CAMERA_CONTROL_LIBRARY)
  message(FATAL_ERROR
    "Hikrobot x86_64 MVS SDK was not found under ${MVS_ROOT}. "
    "Install MVS or configure with -DMVS_ROOT=/path/to/MVS")
endif()

add_executable(blue_armor_mvs_detector src/blue_armor_mvs_detector.cpp)
target_compile_options(blue_armor_mvs_detector PRIVATE -Wall -Wextra -Wpedantic -O3)
target_include_directories(blue_armor_mvs_detector PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${MVS_INCLUDE_DIR}
)
target_link_libraries(blue_armor_mvs_detector PRIVATE
  ${OpenCV_LIBS}
  ${MVS_CAMERA_CONTROL_LIBRARY}
)

install(TARGETS blue_armor_mvs_detector RUNTIME DESTINATION bin)
