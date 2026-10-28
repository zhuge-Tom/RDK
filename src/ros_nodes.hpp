#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace rmcs::ros {
std::shared_ptr<rclcpp::Node> make_camera(const rclcpp::NodeOptions &options);
std::shared_ptr<rclcpp::Node> make_detector(const rclcpp::NodeOptions &options);
std::shared_ptr<rclcpp::Node> make_tracker(const rclcpp::NodeOptions &options);
std::shared_ptr<rclcpp::Node> make_visualizer(const rclcpp::NodeOptions &options);
} // namespace rmcs::ros
