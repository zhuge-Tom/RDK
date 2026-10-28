#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace rmcs::armor {

enum class Color { Blue, Red };
enum class Mode { Blue, Red, Both };

inline auto color_name(Color color) -> const char* {
    return color == Color::Red ? "RED" : "BLUE";
}

inline auto parse_mode(const std::string& value) -> Mode {
    if (value == "blue") return Mode::Blue;
    if (value == "red") return Mode::Red;
    if (value == "both") return Mode::Both;
    throw std::invalid_argument("--color must be blue, red or both");
}

struct Config {
    int hue_min = 90;
    int hue_max = 135;
    int saturation_min = 50;
    int value_min = 50;
    int gray_min = 110;
    int red_hue_low_max = 10;
    int red_hue_high_min = 170;
    int red_gray_min = 60;
    double min_red_blue_ratio = 1.05;
    int close_kernel_height = 7;
    double min_light_area = 150.0;
    double max_light_area = 80000.0;
    double min_light_aspect_ratio = 1.5;
    int min_light_height = 35;
    int min_light_width = 5;
    int max_light_height = 700;
    int max_light_width = 200;
    double min_blue_red_ratio = 1.05;
    double max_pair_height_ratio = 1.8;
    double max_pair_y_ratio = 0.8;
    double min_pair_x_ratio = 0.8;
    double max_pair_x_ratio = 5.0;
    double min_armor_aspect_ratio = 1.2;
    double max_armor_aspect_ratio = 5.5;
};

struct Lightbar {
    cv::Rect box;
    cv::RotatedRect rotated_box;
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    Color color = Color::Blue;
};

struct Armor {
    std::size_t left_index;
    std::size_t right_index;
    std::array<cv::Point2f, 4> corners;
    cv::Rect bounding_box;
    double score;
    Color color = Color::Blue;
};

inline auto long_axis_endpoints(const cv::RotatedRect& box)
    -> std::pair<cv::Point2f, cv::Point2f> {
    auto points = std::array<cv::Point2f, 4> {};
    box.points(points.data());
    const auto edge_01 = cv::norm(points[0] - points[1]);
    const auto edge_12 = cv::norm(points[1] - points[2]);
    auto first = cv::Point2f {};
    auto second = cv::Point2f {};
    if (edge_01 >= edge_12) {
        first = 0.5F * (points[1] + points[2]);
        second = 0.5F * (points[3] + points[0]);
    } else {
        first = 0.5F * (points[0] + points[1]);
        second = 0.5F * (points[2] + points[3]);
    }
    return first.y <= second.y ? std::pair { first, second } : std::pair { second, first };
}

inline auto find_lightbars(const cv::Mat& bgr, cv::Mat& mask, const Config& config, Color color)
    -> std::vector<Lightbar> {
    auto hsv = cv::Mat {};
    auto gray = cv::Mat {};
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    auto color_mask = cv::Mat {};
    auto bright_mask = cv::Mat {};
    if (color == Color::Red) {
        auto high_red = cv::Mat {};
        cv::inRange(hsv, cv::Scalar { 0, static_cast<double>(config.saturation_min),
            static_cast<double>(config.value_min) },
            cv::Scalar { static_cast<double>(config.red_hue_low_max), 255, 255 }, color_mask);
        cv::inRange(hsv, cv::Scalar { static_cast<double>(config.red_hue_high_min),
            static_cast<double>(config.saturation_min), static_cast<double>(config.value_min) },
            cv::Scalar { 179, 255, 255 }, high_red);
        cv::bitwise_or(color_mask, high_red, color_mask);
    } else {
        cv::inRange(hsv,
            cv::Scalar { static_cast<double>(config.hue_min),
                static_cast<double>(config.saturation_min), static_cast<double>(config.value_min) },
            cv::Scalar { static_cast<double>(config.hue_max), 255, 255 }, color_mask);
    }
    cv::inRange(gray, color == Color::Red ? config.red_gray_min : config.gray_min,
        255, bright_mask);
    cv::bitwise_and(color_mask, bright_mask, mask);
    const auto close_kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size { 3, config.close_kernel_height });
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);

    auto contours = std::vector<std::vector<cv::Point>> {};
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    auto lightbars = std::vector<Lightbar> {};
    for (const auto& contour : contours) {
        const auto area = cv::contourArea(contour);
        if (area < config.min_light_area || area > config.max_light_area) continue;
        const auto box = cv::boundingRect(contour);
        if (box.height < config.min_light_height || box.width < config.min_light_width
            || box.height > config.max_light_height || box.width > config.max_light_width) {
            continue;
        }
        if (static_cast<double>(box.height) / std::max(box.width, 1)
            < config.min_light_aspect_ratio) {
            continue;
        }
        const auto mean = cv::mean(bgr(box));
        if (color == Color::Red) {
            if (mean[2] < mean[0] * config.min_red_blue_ratio) continue;
        } else if (mean[0] < mean[2] * config.min_blue_red_ratio) {
            continue;
        }
        const auto rotated_box = cv::minAreaRect(contour);
        const auto [top, bottom] = long_axis_endpoints(rotated_box);
        lightbars.push_back({ box, rotated_box, top, bottom, 0.5F * (top + bottom), color });
    }
    std::ranges::sort(lightbars, {}, [](const Lightbar& light) { return light.center.x; });
    return lightbars;
}

inline auto pair_lightbars(const std::vector<Lightbar>& lightbars, const Config& config)
    -> std::vector<Armor> {
    auto candidates = std::vector<Armor> {};
    for (std::size_t i = 0; i < lightbars.size(); ++i) {
        for (std::size_t j = i + 1; j < lightbars.size(); ++j) {
            const auto& left = lightbars[i];
            const auto& right = lightbars[j];
            if (left.color != right.color) continue;
            const auto min_height = std::max(1, std::min(left.box.height, right.box.height));
            const auto height_ratio =
                static_cast<double>(std::max(left.box.height, right.box.height)) / min_height;
            if (height_ratio > config.max_pair_height_ratio) continue;
            const auto average_height = 0.5 * (left.box.height + right.box.height);
            const auto y_difference = std::abs(left.center.y - right.center.y);
            if (y_difference > average_height * config.max_pair_y_ratio) continue;
            const auto x_difference = right.center.x - left.center.x;
            if (x_difference < average_height * config.min_pair_x_ratio
                || x_difference > average_height * config.max_pair_x_ratio) continue;
            const auto armor_ratio =
                (x_difference + average_height * 0.25) / (average_height * 0.95);
            if (armor_ratio < config.min_armor_aspect_ratio
                || armor_ratio > config.max_armor_aspect_ratio) continue;
            auto corners = std::array { left.top, right.top, right.bottom, left.bottom };
            const auto points = std::vector<cv::Point2f> { corners.begin(), corners.end() };
            candidates.push_back({ i, j, corners, cv::boundingRect(points),
                x_difference + average_height - y_difference, left.color });
        }
    }
    std::ranges::sort(candidates, std::greater {}, &Armor::score);
    auto selected = std::vector<Armor> {};
    auto used = std::vector<bool>(lightbars.size(), false);
    for (const auto& candidate : candidates) {
        if (used[candidate.left_index] || used[candidate.right_index]) continue;
        used[candidate.left_index] = true;
        used[candidate.right_index] = true;
        selected.push_back(candidate);
    }
    return selected;
}

inline void draw(cv::Mat& image, const std::vector<Lightbar>& lights,
    const std::vector<Armor>& armors, bool show_labels = true) {
    for (const auto& light : lights) {
        auto points = std::array<cv::Point2f, 4> {};
        light.rotated_box.points(points.data());
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
        cv::rectangle(image, armor.bounding_box, cv::Scalar { 255, 0, 255 }, 1, cv::LINE_AA);
        if (show_labels) {
            cv::putText(image, std::string(color_name(armor.color)) + " ARMOR " + std::to_string(index),
                cv::Point { armor.bounding_box.x, std::max(15, armor.bounding_box.y - 5) },
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar { 0, 255, 0 }, 1, cv::LINE_AA);
        }
    }
}

struct Result {
    std::vector<Lightbar> lights;
    std::vector<Armor> armors;
    cv::Mat mask;
};

inline auto detect(const cv::Mat& image, const Config& config, Mode mode) -> Result {
    auto result = Result {};
    result.mask = cv::Mat::zeros(image.size(), CV_8UC1);
    for (const auto color : { Color::Blue, Color::Red }) {
        if (mode == Mode::Blue && color != Color::Blue) continue;
        if (mode == Mode::Red && color != Color::Red) continue;
        auto mask = cv::Mat {};
        auto lights = find_lightbars(image, mask, config, color);
        auto armors = pair_lightbars(lights, config);
        for (auto& armor : armors) {
            armor.left_index += result.lights.size();
            armor.right_index += result.lights.size();
        }
        result.lights.insert(result.lights.end(), lights.begin(), lights.end());
        result.armors.insert(result.armors.end(), armors.begin(), armors.end());
        cv::bitwise_or(result.mask, mask, result.mask);
    }
    return result;
}

} // namespace rmcs::armor
