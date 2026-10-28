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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg" "msg/Armors.msg" "msg/ImageTrack.msg"
  "msg/ArmorPose.msg" "msg/SpatialTrack.msg" "msg/VisionResult.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

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

    add_library(blue_vision_components SHARED src/ros_nodes.cpp)
    target_include_directories(blue_vision_components PRIVATE ${OpenCV_INCLUDE_DIRS} ${MVS_INCLUDE_DIR})
    ament_target_dependencies(blue_vision_components rclcpp rclcpp_components sensor_msgs std_msgs geometry_msgs std_srvs)
    rosidl_get_typesupport_target(vision_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(blue_vision_components ${OpenCV_LIBS} ${MVS_CAMERA_CONTROL_LIBRARY} "${vision_typesupport}")
    rclcpp_components_register_nodes(blue_vision_components
      "rmcs::ros::HikCameraNode" "rmcs::ros::ArmorDetectionNode"
      "rmcs::ros::ArmorTrackingNode" "rmcs::ros::ArmorVisualizationNode")
    add_executable(blue_vision_ros src/blue_vision_ros.cpp)
    target_include_directories(blue_vision_ros PRIVATE ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(blue_vision_ros rclcpp)
    target_link_libraries(blue_vision_ros blue_vision_components ${OpenCV_LIBS})
    install(TARGETS blue_vision_components blue_vision_ros
      ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
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
  scripts/rdk_ros_browser.py
  DESTINATION lib/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
  add_executable(armor_color_test test/armor_color_test.cpp)
  target_include_directories(armor_color_test PRIVATE src ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(armor_color_test ${OpenCV_LIBS})
  add_test(NAME armor_color_regression COMMAND armor_color_test)
  add_executable(pose_tracking_test test/pose_tracking_test.cpp)
  target_include_directories(pose_tracking_test PRIVATE src ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(pose_tracking_test ${OpenCV_LIBS})
  add_test(NAME blue_pose_tracking_regression COMMAND pose_tracking_test)
  add_executable(image_tracking_test test/image_tracking_test.cpp)
  target_include_directories(image_tracking_test PRIVATE src ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(image_tracking_test ${OpenCV_LIBS})
  add_test(NAME blue_image_prediction_regression COMMAND image_tracking_test)
  if(TARGET blue_armor_mvs_detector)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME armor_cli_regression COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/check_detector_cli.py
      $<TARGET_FILE:blue_armor_mvs_detector>)
    add_test(NAME armor_browser_regression COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/check_browser.py
      $<TARGET_FILE:blue_armor_mvs_detector>
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rdk_mjpeg_browser.py)
    add_test(NAME blue_ros_pipeline_regression COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/check_ros_pipeline.py
      $<TARGET_FILE:blue_vision_ros>
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rdk_ros_browser.py)
    set_tests_properties(blue_ros_pipeline_regression PROPERTIES TIMEOUT 55
      ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}")
  endif()
endif()

ament_package()
