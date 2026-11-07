#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <ranges>
#include <sstream>
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
    int min_light_width = 3;
    int max_light_height = 700;
    int max_light_width = 200;
    double min_blue_red_ratio = 1.05;
    double max_pair_height_ratio = 1.8;
    double max_pair_y_ratio = 0.8;
    double min_pair_x_ratio = 0.5;
    double max_pair_x_ratio = 5.0;
    double min_armor_aspect_ratio = 0.75;
    double max_armor_aspect_ratio = 5.5;
    double max_light_tilt_deg = 75.0;
    double max_pair_angle_deg = 20.0;
    double max_pair_alignment_deg = 25.0;
    double fragment_gap_width_ratio = 3.0;
    int max_light_fragments = 128;
};

struct Diagnostics {
    int contours = 0;
    int fragment_merges = 0;
    int fragment_limit_dropped = 0;
    int lightbars = 0;
    int rejected_small = 0;
    int rejected_large = 0;
    int rejected_shape = 0;
    int rejected_tilt = 0;
    int rejected_color = 0;
    int rejected_border = 0;
    int rejected_pair_angle = 0;
    int rejected_pair_alignment = 0;
    int rejected_pair_geometry = 0;
    int armors = 0;

    auto json() const -> std::string {
        auto out = std::ostringstream {};
        out << '{';
#define COUNT(name) out << "\"" #name "\":" << name << ','
        COUNT(contours); COUNT(fragment_merges); COUNT(fragment_limit_dropped);
        COUNT(lightbars); COUNT(rejected_small); COUNT(rejected_large);
        COUNT(rejected_shape); COUNT(rejected_tilt); COUNT(rejected_color);
        COUNT(rejected_border); COUNT(rejected_pair_angle);
        COUNT(rejected_pair_alignment); COUNT(rejected_pair_geometry);
#undef COUNT
        out << "\"armors\":" << armors << '}';
        return out.str();
    }
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

inline auto long_axis(const cv::RotatedRect& box) -> cv::Point2f {
    const auto [top, bottom] = long_axis_endpoints(box);
    return (bottom - top) * (1.0F / std::max(1e-4F, static_cast<float>(cv::norm(bottom - top))));
}

inline auto tilt_degrees(const cv::Point2f& axis) -> double {
    return std::acos(std::clamp(std::abs(static_cast<double>(axis.y)), 0.0, 1.0)) * 180.0 / CV_PI;
}

struct LightFragment {
    std::vector<cv::Point> points;
    cv::RotatedRect rectangle;
    double area;
};

// Join adjacent LED pieces along their common axis, without enlarging every mask.
// Gap allowance follows emitter width, so close targets do not need a huge fixed kernel.
inline auto fragment_join_cost(const LightFragment& a, const LightFragment& b,
    const Config& config) -> double {
    const auto wa = std::min(a.rectangle.size.width, a.rectangle.size.height) + 1.0;
    const auto wb = std::min(b.rectangle.size.width, b.rectangle.size.height) + 1.0;
    const auto la = std::max(a.rectangle.size.width, a.rectangle.size.height) + 1.0;
    const auto lb = std::max(b.rectangle.size.width, b.rectangle.size.height) + 1.0;
    if (std::max(wa, wb) > 2.5 * std::min(wa, wb)) return -1;
    auto ua = long_axis(a.rectangle), ub = long_axis(b.rectangle);
    const auto elongated_a = la / wa >= 1.5, elongated_b = lb / wb >= 1.5;
    if (elongated_a && elongated_b
        && std::abs(ua.dot(ub)) < std::cos(config.max_pair_angle_deg * CV_PI / 180)) return -1;
    if (ua.dot(ub) < 0) ub = -ub;
    const auto delta = b.rectangle.center - a.rectangle.center;
    auto axis = elongated_a && elongated_b ? ua + ub
        : elongated_a ? ua : elongated_b ? ub : delta;
    const auto norm = cv::norm(axis);
    if (norm < 1e-4) return -1;
    axis *= static_cast<float>(1.0 / norm);
    if (tilt_degrees(axis) > config.max_light_tilt_deg) return -1;
    const auto normal = cv::Point2f { axis.y, -axis.x };
    const auto width = 0.5 * (wa + wb);
    if (std::abs(delta.dot(normal)) > std::max(2.0, 0.75 * width)) return -1;
    const auto extent = [&](const LightFragment& fragment) {
        std::array<cv::Point2f, 4> corners;
        fragment.rectangle.points(corners.data());
        auto lo = 1e9, hi = -1e9;
        for (const auto& p : corners) {
            const auto projection = (p - fragment.rectangle.center).dot(axis);
            lo = std::min(lo, static_cast<double>(projection));
            hi = std::max(hi, static_cast<double>(projection));
        }
        return 0.5 * (hi - lo);
    };
    const auto gap = std::abs(delta.dot(axis)) - extent(a) - extent(b);
    if (gap < -0.25 * std::min(la, lb)
        || gap > std::max(static_cast<double>(config.close_kernel_height),
                          config.fragment_gap_width_ratio * width)) return -1;
    return std::max(0.0, gap) / width + std::abs(delta.dot(normal)) / width;
}

inline void join_light_fragments(std::vector<LightFragment>& fragments,
    const Config& config, Diagnostics& diagnostics) {
    if (config.fragment_gap_width_ratio <= 0) return;
    struct Edge { double cost; std::size_t a, b; };
    std::vector<Edge> edges;
    for (std::size_t i = 0; i < fragments.size(); ++i)
        for (std::size_t j = i + 1; j < fragments.size(); ++j) {
            const auto cost = fragment_join_cost(fragments[i], fragments[j], config);
            if (cost >= 0) edges.push_back({cost, i, j});
        }
    std::ranges::sort(edges, {}, &Edge::cost);
    std::vector<std::size_t> parent(fragments.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = i;
    const auto root = [&](std::size_t i) {
        while (parent[i] != i) i = parent[i];
        return i;
    };
    for (const auto& edge : edges) {
        const auto a = root(edge.a), b = root(edge.b);
        if (a == b || fragment_join_cost(fragments[a], fragments[b], config) < 0) continue;
        auto points = fragments[a].points;
        points.insert(points.end(), fragments[b].points.begin(), fragments[b].points.end());
        const auto rectangle = cv::minAreaRect(points);
        const auto merged_width = std::min(rectangle.size.width, rectangle.size.height) + 1.0;
        const auto original_width = std::max(
            std::min(fragments[a].rectangle.size.width, fragments[a].rectangle.size.height),
            std::min(fragments[b].rectangle.size.width, fragments[b].rectangle.size.height)) + 1.0;
        if (merged_width > 1.6 * original_width + 1.0) continue;
        fragments[a] = {std::move(points), rectangle, fragments[a].area + fragments[b].area};
        parent[b] = a;
        ++diagnostics.fragment_merges;
    }
    std::vector<LightFragment> joined;
    for (std::size_t i = 0; i < fragments.size(); ++i)
        if (parent[i] == i) joined.push_back(std::move(fragments[i]));
    fragments = std::move(joined);
}

inline auto find_lightbars(const cv::Mat& bgr, cv::Mat& mask, const Config& config, Color color,
    Diagnostics* counters = nullptr)
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
    auto local = Diagnostics {};
    auto& diagnostics = counters ? *counters : local;
    diagnostics.contours += static_cast<int>(contours.size());
    std::vector<LightFragment> fragments;
    for (auto& contour : contours) {
        const auto area = cv::contourArea(contour);
        if (area < std::max(3.0, config.min_light_area * 0.1)) continue;
        fragments.push_back({contour, cv::minAreaRect(contour), area});
    }
    if (fragments.size() > static_cast<std::size_t>(config.max_light_fragments)) {
        std::ranges::sort(fragments, std::greater {}, &LightFragment::area);
        diagnostics.fragment_limit_dropped += static_cast<int>(fragments.size()) - config.max_light_fragments;
        fragments.resize(config.max_light_fragments);
    }
    join_light_fragments(fragments, config, diagnostics);
    auto lightbars = std::vector<Lightbar> {};
    for (const auto& fragment : fragments) {
        const auto& rotated_box = fragment.rectangle;
        const auto length = std::max(rotated_box.size.width, rotated_box.size.height) + 1.0;
        const auto width = std::min(rotated_box.size.width, rotated_box.size.height) + 1.0;
        if (fragment.area < config.min_light_area || length < config.min_light_height
            || width < config.min_light_width) { ++diagnostics.rejected_small; continue; }
        if (fragment.area > config.max_light_area || length > config.max_light_height
            || width > config.max_light_width) { ++diagnostics.rejected_large; continue; }
        if (length / width < config.min_light_aspect_ratio) {
            ++diagnostics.rejected_shape; continue;
        }
        if (tilt_degrees(long_axis(rotated_box)) > config.max_light_tilt_deg) {
            ++diagnostics.rejected_tilt; continue;
        }
        const auto box = cv::boundingRect(fragment.points);
        if (box.x <= 0 || box.y <= 0 || box.br().x >= bgr.cols || box.br().y >= bgr.rows) {
            ++diagnostics.rejected_border; continue;
        }
        const auto mean = cv::mean(bgr(box));
        if (color == Color::Red) {
            if (mean[2] < mean[0] * config.min_red_blue_ratio) { ++diagnostics.rejected_color; continue; }
        } else if (mean[0] < mean[2] * config.min_blue_red_ratio) {
            ++diagnostics.rejected_color; continue;
        }
        const auto [top, bottom] = long_axis_endpoints(rotated_box);
        lightbars.push_back({ box, rotated_box, top, bottom, 0.5F * (top + bottom), color });
    }
    diagnostics.lightbars += static_cast<int>(lightbars.size());
    std::ranges::sort(lightbars, {}, [](const Lightbar& light) { return light.center.x; });
    return lightbars;
}

inline auto pair_lightbars(const std::vector<Lightbar>& lightbars, const Config& config,
    Diagnostics* counters = nullptr)
    -> std::vector<Armor> {
    auto candidates = std::vector<Armor> {};
    auto local = Diagnostics {};
    auto& diagnostics = counters ? *counters : local;
    for (std::size_t i = 0; i < lightbars.size(); ++i) {
        for (std::size_t j = i + 1; j < lightbars.size(); ++j) {
            const auto& left = lightbars[i];
            const auto& right = lightbars[j];
            if (left.color != right.color) continue;
            const auto left_height = cv::norm(left.bottom - left.top) + 1.0;
            const auto right_height = cv::norm(right.bottom - right.top) + 1.0;
            if (std::max(left_height, right_height) / std::min(left_height, right_height)
                > config.max_pair_height_ratio) { ++diagnostics.rejected_pair_geometry; continue; }
            auto left_axis = long_axis(left.rotated_box), right_axis = long_axis(right.rotated_box);
            if (left_axis.dot(right_axis) < 0) right_axis = -right_axis;
            if (left_axis.dot(right_axis) < std::cos(config.max_pair_angle_deg * CV_PI / 180)) {
                ++diagnostics.rejected_pair_angle; continue;
            }
            auto axis = left_axis + right_axis;
            axis *= static_cast<float>(1.0 / cv::norm(axis));
            const auto normal = cv::Point2f { axis.y, -axis.x };
            const auto delta = right.center - left.center;
            const auto average_height = 0.5 * (left_height + right_height);
            const auto y_difference = std::abs(delta.dot(axis));
            const auto x_difference = std::abs(delta.dot(normal));
            if (y_difference > average_height * config.max_pair_y_ratio
                || std::atan2(y_difference, x_difference) * 180 / CV_PI > config.max_pair_alignment_deg) {
                ++diagnostics.rejected_pair_alignment; continue;
            }
            if (x_difference < average_height * config.min_pair_x_ratio
                || x_difference > average_height * config.max_pair_x_ratio) {
                ++diagnostics.rejected_pair_geometry; continue;
            }
            const auto armor_ratio =
                (x_difference + average_height * 0.25) / (average_height * 0.95);
            if (armor_ratio < config.min_armor_aspect_ratio
                || armor_ratio > config.max_armor_aspect_ratio) {
                ++diagnostics.rejected_pair_geometry; continue;
            }
            auto corners = std::array { left.top, right.top, right.bottom, left.bottom };
            const auto points = std::vector<cv::Point2f> { corners.begin(), corners.end() };
            if (!cv::isContourConvex(points)) { ++diagnostics.rejected_pair_geometry; continue; }
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
    diagnostics.armors += static_cast<int>(selected.size());
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
    Diagnostics diagnostics;
};

inline auto detect(const cv::Mat& image, const Config& config, Mode mode) -> Result {
    auto result = Result {};
    result.mask = cv::Mat::zeros(image.size(), CV_8UC1);
    for (const auto color : { Color::Blue, Color::Red }) {
        if (mode == Mode::Blue && color != Color::Blue) continue;
        if (mode == Mode::Red && color != Color::Red) continue;
        auto mask = cv::Mat {};
        auto lights = find_lightbars(image, mask, config, color, &result.diagnostics);
        auto armors = pair_lightbars(lights, config, &result.diagnostics);
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
