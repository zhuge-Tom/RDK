#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

namespace rmcs::tools {

class BlueArmorDetectorNode final : public rclcpp::Node {
private:
    struct Config {
        int hue_min;
        int hue_max;
        int saturation_min;
        int value_min;
        int gray_min;
        int close_kernel_height;
        double min_light_area;
        double max_light_area;
        double min_light_aspect_ratio;
        int min_light_height;
        int min_light_width;
        int max_light_height;
        int max_light_width;
        double min_blue_red_ratio;
        double max_pair_height_ratio;
        double max_pair_y_ratio;
        double min_pair_x_ratio;
        double max_pair_x_ratio;
        double min_armor_aspect_ratio;
        double max_armor_aspect_ratio;
    } config_;

    struct Lightbar {
        cv::Rect box;
        cv::RotatedRect rotated_box;
        cv::Point2f top;
        cv::Point2f bottom;
        cv::Point2f center;
        double area;
    };

    struct Armor {
        std::size_t left_index;
        std::size_t right_index;
        std::array<cv::Point2f, 4> corners; // TL, TR, BR, BL
        cv::Rect bounding_box;
        double score;
    };

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr armors_publisher_;

    static auto long_axis_endpoints(const cv::RotatedRect& box)
        -> std::pair<cv::Point2f, cv::Point2f> {
        auto points = std::array<cv::Point2f, 4> { };
        box.points(points.data());

        const auto edge_01 = cv::norm(points[0] - points[1]);
        const auto edge_12 = cv::norm(points[1] - points[2]);
        auto first         = cv::Point2f { };
        auto second        = cv::Point2f { };
        if (edge_01 >= edge_12) {
            first  = 0.5F * (points[1] + points[2]);
            second = 0.5F * (points[3] + points[0]);
        } else {
            first  = 0.5F * (points[0] + points[1]);
            second = 0.5F * (points[2] + points[3]);
        }
        if (first.y <= second.y) return { first, second };
        return { second, first };
    }

    auto find_lightbars(const cv::Mat& bgr, cv::Mat& mask) const -> std::vector<Lightbar> {
        auto hsv  = cv::Mat { };
        auto gray = cv::Mat { };
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

        auto blue_mask   = cv::Mat { };
        auto bright_mask = cv::Mat { };
        cv::inRange(hsv,
            cv::Scalar { static_cast<double>(config_.hue_min),
                static_cast<double>(config_.saturation_min),
                static_cast<double>(config_.value_min) },
            cv::Scalar { static_cast<double>(config_.hue_max), 255, 255 }, blue_mask);
        cv::inRange(gray, config_.gray_min, 255, bright_mask);
        cv::bitwise_and(blue_mask, bright_mask, mask);

        // The light cores can be white and split one blue contour into several vertical pieces.
        // A vertical close reconnects those pieces before contour filtering.
        const auto close_kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size { 3, config_.close_kernel_height });
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);

        auto contours = std::vector<std::vector<cv::Point>> { };
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        auto lightbars = std::vector<Lightbar> { };
        lightbars.reserve(contours.size());
        for (const auto& contour : contours) {
            const auto area = cv::contourArea(contour);
            if (area < config_.min_light_area || area > config_.max_light_area) continue;

            const auto box = cv::boundingRect(contour);
            if (box.height < config_.min_light_height || box.width < config_.min_light_width
                || box.height > config_.max_light_height || box.width > config_.max_light_width) {
                continue;
            }
            if (static_cast<double>(box.height) / std::max(box.width, 1)
                < config_.min_light_aspect_ratio) {
                continue;
            }

            const auto mean = cv::mean(bgr(box));
            if (mean[0] < mean[2] * config_.min_blue_red_ratio) continue;

            const auto rotated_box  = cv::minAreaRect(contour);
            const auto [top, bottom] = long_axis_endpoints(rotated_box);
            lightbars.push_back(Lightbar {
                .box         = box,
                .rotated_box = rotated_box,
                .top         = top,
                .bottom      = bottom,
                .center      = 0.5F * (top + bottom),
                .area        = area,
            });
        }

        std::ranges::sort(lightbars, { }, [](const Lightbar& lightbar) { return lightbar.center.x; });
        return lightbars;
    }

    auto pair_lightbars(const std::vector<Lightbar>& lightbars) const -> std::vector<Armor> {
        auto candidates = std::vector<Armor> { };
        for (std::size_t i = 0; i < lightbars.size(); ++i) {
            const auto& left = lightbars[i];
            for (std::size_t j = i + 1; j < lightbars.size(); ++j) {
                const auto& right = lightbars[j];
                const auto min_height = std::max(1, std::min(left.box.height, right.box.height));
                const auto height_ratio =
                    static_cast<double>(std::max(left.box.height, right.box.height)) / min_height;
                if (height_ratio > config_.max_pair_height_ratio) continue;

                const auto average_height = 0.5 * (left.box.height + right.box.height);
                const auto y_difference   = std::abs(left.center.y - right.center.y);
                if (y_difference > average_height * config_.max_pair_y_ratio) continue;

                const auto x_difference = right.center.x - left.center.x;
                if (x_difference < average_height * config_.min_pair_x_ratio
                    || x_difference > average_height * config_.max_pair_x_ratio) {
                    continue;
                }

                const auto armor_width  = x_difference + average_height * 0.25;
                const auto armor_height = average_height * 0.95;
                const auto armor_ratio  = armor_width / std::max(armor_height, 1.0);
                if (armor_ratio < config_.min_armor_aspect_ratio
                    || armor_ratio > config_.max_armor_aspect_ratio) {
                    continue;
                }

                auto corners = std::array { left.top, right.top, right.bottom, left.bottom };
                auto points  = std::vector<cv::Point2f> { corners.begin(), corners.end() };
                candidates.push_back(Armor {
                    .left_index   = i,
                    .right_index  = j,
                    .corners      = corners,
                    .bounding_box = cv::boundingRect(points),
                    .score        = x_difference + average_height - y_difference,
                });
            }
        }

        std::ranges::sort(candidates, std::greater { }, &Armor::score);

        // A lightbar belongs to at most one published candidate in each frame.
        auto selected = std::vector<Armor> { };
        auto used     = std::vector<bool>(lightbars.size(), false);
        for (const auto& candidate : candidates) {
            if (used[candidate.left_index] || used[candidate.right_index]) continue;
            used[candidate.left_index]  = true;
            used[candidate.right_index] = true;
            selected.push_back(candidate);
        }
        return selected;
    }

    static auto into_message(const std_msgs::msg::Header& header,
        const std::vector<Armor>& armors) -> sensor_msgs::msg::PointCloud {
        auto message   = sensor_msgs::msg::PointCloud { };
        message.header = header;
        message.points.reserve(armors.size() * 4);
        message.channels.resize(3);
        message.channels[0].name = "armor_id";
        message.channels[1].name = "corner_id";
        message.channels[2].name = "geometry_score";
        for (auto& channel : message.channels) channel.values.reserve(armors.size() * 4);

        for (std::size_t armor_index = 0; armor_index < armors.size(); ++armor_index) {
            const auto& armor = armors[armor_index];
            for (std::size_t corner_index = 0; corner_index < armor.corners.size(); ++corner_index) {
                const auto& corner = armor.corners[corner_index];
                auto point         = geometry_msgs::msg::Point32 { };
                point.x            = corner.x;
                point.y            = corner.y;
                point.z            = 0.0F;
                message.points.push_back(point);
                message.channels[0].values.push_back(static_cast<float>(armor_index));
                message.channels[1].values.push_back(static_cast<float>(corner_index));
                message.channels[2].values.push_back(static_cast<float>(armor.score));
            }
        }
        return message;
    }

    static auto draw(cv::Mat& image, const std::vector<Lightbar>& lightbars,
        const std::vector<Armor>& armors) -> void {
        for (const auto& lightbar : lightbars) {
            auto points = std::array<cv::Point2f, 4> { };
            lightbar.rotated_box.points(points.data());
            for (std::size_t index = 0; index < points.size(); ++index) {
                cv::line(image, points[index], points[(index + 1) % points.size()],
                    cv::Scalar { 0, 255, 255 }, 1, cv::LINE_AA);
            }
        }

        for (std::size_t index = 0; index < armors.size(); ++index) {
            const auto& armor = armors[index];
            for (std::size_t corner = 0; corner < armor.corners.size(); ++corner) {
                cv::line(image, armor.corners[corner],
                    armor.corners[(corner + 1) % armor.corners.size()],
                    cv::Scalar { 0, 255, 0 }, 2, cv::LINE_AA);
                cv::circle(image, armor.corners[corner], 3, cv::Scalar { 0, 0, 255 }, -1,
                    cv::LINE_AA);
            }
            cv::rectangle(image, armor.bounding_box, cv::Scalar { 255, 0, 255 }, 1,
                cv::LINE_AA);
            cv::putText(image, "BLUE ARMOR " + std::to_string(index),
                cv::Point { armor.bounding_box.x, std::max(15, armor.bounding_box.y - 5) },
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar { 0, 255, 0 }, 1, cv::LINE_AA);
        }
    }

    auto on_image(const sensor_msgs::msg::Image::ConstSharedPtr& message) -> void {
        try {
            auto bridge = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
            auto mask   = cv::Mat { };
            const auto lightbars = find_lightbars(bridge->image, mask);
            const auto armors    = pair_lightbars(lightbars);

            armors_publisher_->publish(into_message(message->header, armors));

            draw(bridge->image, lightbars, armors);
            visualization_publisher_->publish(*bridge->toImageMsg());

            auto mask_message = cv_bridge::CvImage {
                message->header, sensor_msgs::image_encodings::MONO8, mask };
            mask_publisher_->publish(*mask_message.toImageMsg());
        } catch (const cv_bridge::Exception& error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "Cannot convert input image to BGR8: %s", error.what());
        } catch (const cv::Exception& error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "OpenCV failed while processing a frame: %s", error.what());
        }
    }

    template <typename T>
    auto parameter(const std::string& name, const T& default_value) -> T {
        return declare_parameter<T>(name, default_value);
    }

public:
    BlueArmorDetectorNode()
        : Node("blue_armor_detector")
        , config_ {
            .hue_min                  = parameter("hue_min", 90),
            .hue_max                  = parameter("hue_max", 135),
            .saturation_min           = parameter("saturation_min", 50),
            .value_min                = parameter("value_min", 50),
            .gray_min                 = parameter("gray_min", 110),
            .close_kernel_height      = parameter("close_kernel_height", 7),
            .min_light_area           = parameter("min_light_area", 150.0),
            .max_light_area           = parameter("max_light_area", 80000.0),
            .min_light_aspect_ratio   = parameter("min_light_aspect_ratio", 1.5),
            .min_light_height         = parameter("min_light_height", 35),
            .min_light_width          = parameter("min_light_width", 5),
            .max_light_height         = parameter("max_light_height", 700),
            .max_light_width          = parameter("max_light_width", 200),
            .min_blue_red_ratio       = parameter("min_blue_red_ratio", 1.05),
            .max_pair_height_ratio    = parameter("max_pair_height_ratio", 1.8),
            .max_pair_y_ratio         = parameter("max_pair_y_ratio", 0.8),
            .min_pair_x_ratio         = parameter("min_pair_x_ratio", 0.8),
            .max_pair_x_ratio         = parameter("max_pair_x_ratio", 5.0),
            .min_armor_aspect_ratio   = parameter("min_armor_aspect_ratio", 1.2),
            .max_armor_aspect_ratio   = parameter("max_armor_aspect_ratio", 5.5),
        } {
        const auto input_topic = parameter<std::string>("input_topic", "/image_raw");
        const auto qos         = rclcpp::SensorDataQoS { };
        visualization_publisher_ =
            create_publisher<sensor_msgs::msg::Image>("~/visualization", qos);
        mask_publisher_ = create_publisher<sensor_msgs::msg::Image>("~/mask", qos);
        armors_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud>("~/armors", qos);
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(input_topic, qos,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& message) { on_image(message); });

        RCLCPP_INFO(get_logger(), "Detecting blue lightbars on %s", input_topic.c_str());
    }
};

} // namespace rmcs::tools

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rmcs::tools::BlueArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
