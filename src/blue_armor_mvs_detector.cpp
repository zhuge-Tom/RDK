#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <ranges>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <MvCameraControl.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

std::atomic_bool stopping = false;

struct Config {
    int hue_min = 90;
    int hue_max = 135;
    int saturation_min = 50;
    int value_min = 50;
    int gray_min = 110;
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
};

struct Armor {
    std::size_t left_index;
    std::size_t right_index;
    std::array<cv::Point2f, 4> corners;
    cv::Rect bounding_box;
    double score;
};

auto long_axis_endpoints(const cv::RotatedRect& box)
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

auto find_lightbars(const cv::Mat& bgr, cv::Mat& mask, const Config& config)
    -> std::vector<Lightbar> {
    auto hsv = cv::Mat {};
    auto gray = cv::Mat {};
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    auto blue_mask = cv::Mat {};
    auto bright_mask = cv::Mat {};
    cv::inRange(hsv,
        cv::Scalar { static_cast<double>(config.hue_min),
            static_cast<double>(config.saturation_min), static_cast<double>(config.value_min) },
        cv::Scalar { static_cast<double>(config.hue_max), 255, 255 }, blue_mask);
    cv::inRange(gray, config.gray_min, 255, bright_mask);
    cv::bitwise_and(blue_mask, bright_mask, mask);
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
        if (mean[0] < mean[2] * config.min_blue_red_ratio) continue;
        const auto rotated_box = cv::minAreaRect(contour);
        const auto [top, bottom] = long_axis_endpoints(rotated_box);
        lightbars.push_back({ box, rotated_box, top, bottom, 0.5F * (top + bottom) });
    }
    std::ranges::sort(lightbars, {}, [](const Lightbar& light) { return light.center.x; });
    return lightbars;
}

auto pair_lightbars(const std::vector<Lightbar>& lightbars, const Config& config)
    -> std::vector<Armor> {
    auto candidates = std::vector<Armor> {};
    for (std::size_t i = 0; i < lightbars.size(); ++i) {
        for (std::size_t j = i + 1; j < lightbars.size(); ++j) {
            const auto& left = lightbars[i];
            const auto& right = lightbars[j];
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
                x_difference + average_height - y_difference });
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

void draw(cv::Mat& image, const std::vector<Lightbar>& lights,
    const std::vector<Armor>& armors) {
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
        cv::putText(image, "BLUE ARMOR " + std::to_string(index),
            cv::Point { armor.bounding_box.x, std::max(15, armor.bounding_box.y - 5) },
            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar { 0, 255, 0 }, 1, cv::LINE_AA);
    }
}

void write_status(const std::vector<Armor>& armors, double fps) {
    constexpr auto temporary = "/tmp/rmcs-blue-armor-status.json.tmp";
    constexpr auto destination = "/tmp/rmcs-blue-armor-status.json";
    auto output = std::ofstream { temporary };
    output << "{\"detected\":" << (armors.empty() ? "false" : "true")
           << ",\"armors\":" << armors.size() << ",\"corners\":[";
    auto first = true;
    for (const auto& armor : armors) {
        for (const auto& corner : armor.corners) {
            if (!first) output << ',';
            first = false;
            output << '[' << corner.x << ',' << corner.y << ']';
        }
    }
    output << "],\"detect_fps\":" << fps << "}";
    output.close();
    std::rename(temporary, destination);
}

void check_mvs(const char* action, int code) {
    if (code != MV_OK) {
        throw std::runtime_error(std::string(action) + " failed, MVS code="
            + std::to_string(static_cast<std::uint32_t>(code)));
    }
}

class Camera {
public:
    Camera(float exposure, float gain, float frame_rate) {
        auto devices = MV_CC_DEVICE_INFO_LIST {};
        check_mvs("enumerate devices", MV_CC_EnumDevices(MV_USB_DEVICE, &devices));
        if (devices.nDeviceNum == 0 || devices.pDeviceInfo[0] == nullptr) {
            throw std::runtime_error("no Hikrobot USB camera found");
        }
        check_mvs("create handle", MV_CC_CreateHandle(&handle_, devices.pDeviceInfo[0]));
        check_mvs("open device", MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0));
        check_mvs("disable auto exposure",
            MV_CC_SetEnumValueByString(handle_, "ExposureAuto", "Off"));
        check_mvs("set exposure", MV_CC_SetFloatValue(handle_, "ExposureTime", exposure));
        check_mvs("disable auto gain", MV_CC_SetEnumValueByString(handle_, "GainAuto", "Off"));
        check_mvs("set gain", MV_CC_SetFloatValue(handle_, "Gain", gain));
        check_mvs("disable trigger", MV_CC_SetEnumValueByString(handle_, "TriggerMode", "Off"));
        check_mvs("enable frame rate",
            MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true));
        check_mvs("set frame rate",
            MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", frame_rate));
        check_mvs("start grabbing", MV_CC_StartGrabbing(handle_));
    }

    ~Camera() {
        if (handle_ == nullptr) return;
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
    }

    auto read() -> cv::Mat {
        auto frame = MV_FRAME_OUT {};
        if (MV_CC_GetImageBuffer(handle_, &frame, 1000) != MV_OK) return {};
        auto raw = cv::Mat(static_cast<int>(frame.stFrameInfo.nHeight),
            static_cast<int>(frame.stFrameInfo.nWidth), CV_8UC1, frame.pBufAddr);
        auto bgr = cv::Mat {};
        cv::cvtColor(raw, bgr, cv::COLOR_BayerBG2BGR);
        MV_CC_FreeImageBuffer(handle_, &frame);
        return bgr;
    }

private:
    void* handle_ = nullptr;
};

auto argument(int argc, char** argv, const std::string& name, double fallback) -> double {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] == name) return std::stod(argv[index + 1]);
    }
    return fallback;
}

} // namespace

auto main(int argc, char** argv) -> int {
    std::signal(SIGINT, [](int) { stopping = true; });
    std::signal(SIGTERM, [](int) { stopping = true; });
    std::signal(SIGPIPE, [](int) { stopping = true; });
    try {
        const auto exposure = static_cast<float>(argument(argc, argv, "--exposure", 3500.0));
        const auto gain = static_cast<float>(argument(argc, argv, "--gain", 2.0));
        const auto frame_rate = static_cast<float>(argument(argc, argv, "--fps", 30.0));
        const auto processing_width = static_cast<int>(argument(argc, argv, "--width", 960.0));
        const auto jpeg_quality = static_cast<int>(argument(argc, argv, "--quality", 75.0));
        auto camera = Camera { exposure, gain, frame_rate };
        const auto config = Config {};
        auto frame_times = std::vector<double> {};
        frame_times.reserve(30);
        auto last = cv::getTickCount();
        std::cerr << "Direct MVS C++ detector: " << processing_width << "px, "
                  << frame_rate << " FPS, exposure=" << exposure << "us\n";
        while (!stopping) {
            auto frame = camera.read();
            if (frame.empty()) continue;
            if (frame.cols > processing_width) {
                const auto height = static_cast<int>(
                    std::lround(frame.rows * processing_width / static_cast<double>(frame.cols)));
                cv::resize(frame, frame, { processing_width, height }, 0.0, 0.0, cv::INTER_AREA);
            }
            auto mask = cv::Mat {};
            const auto lights = find_lightbars(frame, mask, config);
            const auto armors = pair_lightbars(lights, config);
            draw(frame, lights, armors);

            const auto now = cv::getTickCount();
            const auto elapsed = (now - last) / cv::getTickFrequency();
            last = now;
            if (elapsed > 0.0) frame_times.push_back(elapsed);
            if (frame_times.size() > 30) frame_times.erase(frame_times.begin());
            auto total = 0.0;
            for (const auto value : frame_times) total += value;
            const auto fps = total > 0.0 ? frame_times.size() / total : 0.0;

            const auto detected = !armors.empty();
            cv::rectangle(frame, { 0, 0 }, { frame.cols, 48 },
                detected ? cv::Scalar { 20, 125, 25 } : cv::Scalar { 20, 20, 185 }, -1);
            cv::putText(frame,
                detected ? "MVS C++ | DETECTED: " + std::to_string(armors.size()) + " BLUE ARMOR"
                         : "MVS C++ | LOST: NO BLUE ARMOR",
                { 12, 33 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 255, 255 }, 2, cv::LINE_AA);
            write_status(armors, fps);
            auto jpeg = std::vector<unsigned char> {};
            if (!cv::imencode(".jpg", frame, jpeg, { cv::IMWRITE_JPEG_QUALITY, jpeg_quality })) {
                continue;
            }
            std::cout.write(reinterpret_cast<const char*>(jpeg.data()),
                static_cast<std::streamsize>(jpeg.size()));
            std::cout.flush();
            if (!std::cout) break;
        }
    } catch (const std::exception& error) {
        std::cerr << "Detector failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
