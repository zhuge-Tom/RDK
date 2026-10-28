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
#include <filesystem>
#include <iostream>
#include <ranges>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "armor_detector.hpp"
#include "pose_tracking.hpp"
#include "image_tracking.hpp"
#include "mvs_camera.hpp"
#include "vision_output.hpp"

#include <iomanip>
#include <sstream>

#include <memory>
#include <MvCameraControl.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

std::atomic_bool stopping = false;

using namespace rmcs::armor;
namespace vision = rmcs::vision;

using namespace rmcs::output;
using rmcs::mvs::Camera;

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
        const auto* capture_env = std::getenv("ARMOR_CAPTURE_DIR");
        const auto capture_directory = std::string { capture_env ? capture_env : "" };
        if (processing_width <= 0 || frame_rate <= 0 || exposure <= 0 || gain < 0
            || jpeg_quality < 1 || jpeg_quality > 100) {
            throw std::invalid_argument("invalid camera, width or JPEG quality value");
        }
        const auto calibration_path = text_argument(argc, argv, "--calibration", "");
        auto tracking = vision::TrackingConfig {};
        tracking.prediction_seconds = argument(argc, argv, "--prediction-ms", 100.0) / 1000.0;
        const auto calibration = vision::Calibration::load(calibration_path);
        auto geometry_pipeline = vision::GeometryPipeline { calibration, tracking };
        auto image_tracker = vision::ImageTracker { tracking.prediction_seconds };
        std::cerr << "Geometry: " << calibration.reason << "; frame=camera_optical; prediction="
                  << tracking.prediction_seconds * 1000.0 << "ms\n";
        auto camera = std::unique_ptr<Camera> {};
        if (image_path.empty()) camera = std::make_unique<Camera>(exposure, gain, frame_rate);
        const auto config = Config {};
        auto frame_times = std::vector<double> {};
        frame_times.reserve(30);
        auto last = cv::getTickCount();
        const auto started = last;
        std::cerr << "Direct MVS C++ detector: " << processing_width << "px, "
                  << frame_rate << " FPS, exposure=" << exposure << "us, color=" << color << "\n";
        while (!stopping) {
            auto frame = camera ? camera->read() : cv::imread(image_path, cv::IMREAD_COLOR);
            if (frame.empty()) {
                if (!camera) throw std::runtime_error("cannot read image: " + image_path);
                continue;
            }
            capture_calibration_frame(frame, capture_directory);
            const auto timestamp = (cv::getTickCount() - started) / cv::getTickFrequency();
            if (frame.cols > processing_width) {
                const auto height = static_cast<int>(
                    std::lround(frame.rows * processing_width / static_cast<double>(frame.cols)));
                cv::resize(frame, frame, { processing_width, height }, 0.0, 0.0, cv::INTER_AREA);
            }
            const auto result = detect(frame, config, mode);
            const auto& armors = result.armors;
            const auto geometry = geometry_pipeline.process(armors, frame.size(), timestamp);
            const auto image_tracks = image_tracker.update(armors, frame.size(), timestamp);
            // Numeric diagnostics live in the browser's switchable debug panel.
            draw(frame, result.lights, armors, false);
            if (geometry.reason == "ready" && !geometry.poses.empty()) {
                draw_geometry(frame, geometry, geometry_pipeline);
            } else {
                draw_image_tracking(frame, image_tracks);
            }

            const auto now = cv::getTickCount();
            const auto elapsed = (now - last) / cv::getTickFrequency();
            last = now;
            if (elapsed > 0.0) frame_times.push_back(elapsed);
            if (frame_times.size() > 30) frame_times.erase(frame_times.begin());
            auto total = 0.0;
            for (const auto value : frame_times) total += value;
            const auto fps = total > 0.0 ? frame_times.size() / total : 0.0;

            write_status(armors, fps, color, frame.cols, frame.rows, geometry, timestamp,
                tracking.prediction_seconds * 1000.0, image_tracks);
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
