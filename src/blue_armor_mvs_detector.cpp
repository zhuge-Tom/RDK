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

#include "armor_detector.hpp"

#include <memory>
#include <MvCameraControl.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

std::atomic_bool stopping = false;

using namespace rmcs::armor;

void write_status(const std::vector<Armor>& armors, double fps,
    const std::string& mode, int width, int height) {
    const auto* status_env = std::getenv("ARMOR_STATUS_FILE");
    const auto destination = std::string(status_env != nullptr ? status_env
        : "/tmp/rmcs-blue-armor-status.json");
    const auto temporary = destination + ".tmp";
    auto output = std::ofstream { temporary };
    if (!output) throw std::runtime_error("cannot write status: " + temporary);
    const auto red_count = std::ranges::count(armors, Color::Red, &Armor::color);
    output << "{\"mode\":\"" << mode << "\",\"red_armors\":" << red_count
           << ",\"blue_armors\":" << armors.size() - red_count
           << ",\"width\":" << width << ",\"height\":" << height << ",";
    output << "\"detected\":" << (armors.empty() ? "false" : "true")
           << ",\"armors\":" << armors.size() << ",\"corners\":[";
    auto first = true;
    for (const auto& armor : armors) {
        for (const auto& corner : armor.corners) {
            if (!first) output << ',';
            first = false;
            output << '[' << corner.x << ',' << corner.y << ']';
        }
    }
    output << "],\"colors\":[";
    for (std::size_t index = 0; index < armors.size(); ++index) {
        if (index != 0) output << ',';
        output << '\"' << (armors[index].color == Color::Red ? "red" : "blue") << '\"';
    }
    output << "],\"detect_fps\":" << fps << "}";
    output.close();
    if (std::rename(temporary.c_str(), destination.c_str()) != 0) {
        throw std::runtime_error("cannot publish status: " + destination);
    }
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

auto text_argument(int argc, char** argv, const std::string& name,
    const std::string& fallback) -> std::string {
    for (int index = 1; index + 1 < argc; ++index) {
        if (argv[index] == name) return argv[index + 1];
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
        const auto color = text_argument(argc, argv, "--color", "blue");
        const auto mode = parse_mode(color);
        const auto image_path = text_argument(argc, argv, "--image", "");
        if (processing_width <= 0 || frame_rate <= 0 || exposure <= 0 || gain < 0
            || jpeg_quality < 1 || jpeg_quality > 100) {
            throw std::invalid_argument("invalid camera, width or JPEG quality value");
        }
        auto camera = std::unique_ptr<Camera> {};
        if (image_path.empty()) camera = std::make_unique<Camera>(exposure, gain, frame_rate);
        const auto config = Config {};
        auto frame_times = std::vector<double> {};
        frame_times.reserve(30);
        auto last = cv::getTickCount();
        std::cerr << "Direct MVS C++ detector: " << processing_width << "px, "
                  << frame_rate << " FPS, exposure=" << exposure << "us, color=" << color << "\n";
        while (!stopping) {
            auto frame = camera ? camera->read() : cv::imread(image_path, cv::IMREAD_COLOR);
            if (frame.empty()) {
                if (!camera) throw std::runtime_error("cannot read image: " + image_path);
                continue;
            }
            if (frame.cols > processing_width) {
                const auto height = static_cast<int>(
                    std::lround(frame.rows * processing_width / static_cast<double>(frame.cols)));
                cv::resize(frame, frame, { processing_width, height }, 0.0, 0.0, cv::INTER_AREA);
            }
            const auto result = detect(frame, config, mode);
            const auto& armors = result.armors;
            draw(frame, result.lights, armors);

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
                detected ? "MVS C++ | DETECTED: " + std::to_string(armors.size()) + " ARMOR (" + color + ")"
                         : "MVS C++ | LOST: NO ARMOR (" + color + ")",
                { 12, 33 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 255, 255 }, 2, cv::LINE_AA);
            write_status(armors, fps, color, frame.cols, frame.rows);
            auto jpeg = std::vector<unsigned char> {};
            if (!cv::imencode(".jpg", frame, jpeg, { cv::IMWRITE_JPEG_QUALITY, jpeg_quality })) {
                continue;
            }
            std::cout.write(reinterpret_cast<const char*>(jpeg.data()),
                static_cast<std::streamsize>(jpeg.size()));
            std::cout.flush();
            if (!std::cout || !camera) break;
        }
    } catch (const std::exception& error) {
        std::cerr << "Detector failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
