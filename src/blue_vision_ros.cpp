#include "ros_nodes.hpp"
#include <iostream>
#include <opencv2/core.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    cv::setNumThreads(1);
    try {
        auto options = rclcpp::NodeOptions{}.use_intra_process_comms(true);
        auto camera = rmcs::ros::make_camera(options);
        auto detector = rmcs::ros::make_detector(options);
        auto tracker = rmcs::ros::make_tracker(options);
        auto visualizer = rmcs::ros::make_visualizer(options);
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3);
        for (const auto &node : {camera, detector, tracker, visualizer})
            executor.add_node(node);
        executor.spin();
    } catch (const std::exception &error) {
        std::cerr << "ROS vision failed: " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
