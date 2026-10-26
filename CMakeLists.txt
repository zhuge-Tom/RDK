cmake_minimum_required(VERSION 3.22)
project(rdk_s100_blue_armor_detector)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(blue_armor_detector_node src/blue_armor_detector_node.cpp)
target_include_directories(blue_armor_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(blue_armor_detector_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(blue_armor_detector_node ${OpenCV_LIBS})

option(BUILD_MVS_DIRECT "Build the direct Hikrobot MVS camera detector" ON)
set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
if(BUILD_MVS_DIRECT)
  find_path(MVS_INCLUDE_DIR MvCameraControl.h
    PATHS "${MVS_ROOT}/include"
    NO_DEFAULT_PATH
  )
  find_library(MVS_CAMERA_CONTROL_LIBRARY MvCameraControl
    PATHS
      "${MVS_ROOT}/lib/aarch64"
      "${MVS_ROOT}/lib/64"
      "${MVS_ROOT}/lib"
    NO_DEFAULT_PATH
  )
  if(MVS_INCLUDE_DIR AND MVS_CAMERA_CONTROL_LIBRARY)
    add_executable(blue_armor_mvs_detector src/blue_armor_mvs_detector.cpp)
    target_include_directories(blue_armor_mvs_detector PRIVATE
      ${OpenCV_INCLUDE_DIRS}
      ${MVS_INCLUDE_DIR}
    )
    target_link_libraries(blue_armor_mvs_detector
      ${OpenCV_LIBS}
      ${MVS_CAMERA_CONTROL_LIBRARY}
    )
    install(TARGETS blue_armor_mvs_detector DESTINATION lib/${PROJECT_NAME})
  else()
    message(WARNING "MVS SDK not found under ${MVS_ROOT}; skipping direct camera executable")
  endif()
endif()

install(TARGETS blue_armor_detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/mjpeg_ros_bridge.py
  scripts/mvs_raw_mjpeg.py
  scripts/rdk_mjpeg_browser.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
