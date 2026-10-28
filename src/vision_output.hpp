#pragma once
#include "armor_detector.hpp"
#include "image_tracking.hpp"
#include "pose_tracking.hpp"
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <ranges>
#include <sstream>

namespace rmcs::output {
using namespace rmcs::armor;
namespace vision = rmcs::vision;
// Save the next full-resolution, unannotated frame on a browser request.
inline void capture_calibration_frame(const cv::Mat &frame, const std::string &directory) {
    if (directory.empty())
        return;
    const auto root = std::filesystem::path{directory};
    const auto request = root / "capture.request";
    if (!std::filesystem::exists(request))
        return;
    const auto filename = "frame-" + std::to_string(cv::getTickCount()) + ".png";
    auto saved = false;
    try {
        saved = cv::imwrite((root / filename).string(), frame);
    } catch (const cv::Exception &error) {
        std::cerr << "Calibration capture failed: " << error.what() << '\n';
    }
    const auto report = root / "last_capture.json";
    const auto temporary = root / "last_capture.json.tmp";
    {
        auto output = std::ofstream{temporary};
        output << "{\"saved\":" << (saved ? "true" : "false") << ",\"filename\":\"" << filename
               << "\",\"width\":" << frame.cols << ",\"height\":" << frame.rows << '}';
        if (!output)
            throw std::runtime_error("cannot write calibration capture report");
    }
    std::filesystem::rename(temporary, report);
    std::filesystem::remove(request);
}

inline void write_vector(std::ostream &output, const cv::Vec3d &vector) {
    output << '[' << vector[0] << ',' << vector[1] << ',' << vector[2] << ']';
}

inline void write_geometry(std::ostream &output, const vision::GeometryResult &result,
                           double timestamp, double horizon_ms) {
    output << ",\"geometry\":{\"frame\":\"camera_optical\",\"units\":\"meters\","
           << "\"reason\":\"" << result.reason << "\",\"timestamp_seconds\":" << timestamp
           << ",\"prediction_horizon_ms\":" << horizon_ms << ",\"poses\":[";
    auto first = true;
    for (const auto &pose : result.poses) {
        if (!first)
            output << ',';
        first = false;
        output << "{\"detection_index\":" << pose.detection_index << ",\"position_m\":";
        write_vector(output, pose.position_m);
        output << ",\"rotation_vector\":";
        write_vector(output, pose.rotation_vector);
        output << ",\"distance_m\":" << cv::norm(pose.position_m)
               << ",\"reprojection_error_px\":" << pose.reprojection_error_px << '}';
    }
    output << "],\"tracks\":[";
    first = true;
    for (const auto &track : result.tracks) {
        if (!first)
            output << ',';
        first = false;
        output << "{\"id\":" << track.id << ",\"detection_index\":" << track.detection_index
               << ",\"observed\":" << (track.observed ? "true" : "false")
               << ",\"confirmed\":" << (track.confirmed ? "true" : "false")
               << ",\"observation_age_seconds\":" << track.observation_age_seconds
               << ",\"position_m\":";
        write_vector(output, track.position_m);
        output << ",\"velocity_m_s\":";
        write_vector(output, track.velocity_m_s);
        output << ",\"predicted_position_m\":";
        if (track.predicted_position_m)
            write_vector(output, *track.predicted_position_m);
        else
            output << "null";
        output << '}';
    }
    output << "]}";
}

inline void draw_geometry(cv::Mat &image, const vision::GeometryResult &geometry,
                          const vision::GeometryPipeline &pipeline) {
    for (const auto &track : geometry.tracks) {
        const auto current = pipeline.project(track.position_m, image.size());
        if (!current)
            continue;
        if (track.predicted_position_m) {
            if (const auto future = pipeline.project(*track.predicted_position_m, image.size())) {
                cv::line(image, *current, *future, {255, 255, 0}, 2, cv::LINE_AA);
                cv::circle(image, *future, 7, {255, 255, 0}, 2, cv::LINE_AA);
            }
        }
    }
}

inline void write_image_tracking(std::ostream &output,
                                 const std::vector<vision::ImageTrackResult> &tracks,
                                 double horizon_ms) {
    output << ",\"image_tracking\":{\"frame\":\"image\",\"units\":\"pixels\","
           << "\"prediction_horizon_ms\":" << horizon_ms << ",\"tracks\":[";
    auto first = true;
    for (const auto &track : tracks) {
        if (!first)
            output << ',';
        first = false;
        output << "{\"id\":" << track.id << ",\"detection_index\":" << track.detection_index
               << ",\"observed\":" << (track.observed ? "true" : "false")
               << ",\"confirmed\":" << (track.confirmed ? "true" : "false")
               << ",\"observation_age_seconds\":" << track.age_seconds << ",\"center_px\":["
               << track.center_px[0] << ',' << track.center_px[1] << "],\"velocity_px_s\":["
               << track.velocity_px_s[0] << ',' << track.velocity_px_s[1]
               << "],\"predicted_center_px\":";
        if (track.predicted_center_px) {
            output << '[' << (*track.predicted_center_px)[0] << ','
                   << (*track.predicted_center_px)[1] << ']';
        } else
            output << "null";
        output << '}';
    }
    output << "]}";
}

inline void draw_image_tracking(cv::Mat &image,
                                const std::vector<vision::ImageTrackResult> &tracks) {
    for (const auto &track : tracks) {
        const auto current = cv::Point{cvRound(track.center_px[0]), cvRound(track.center_px[1])};
        if (track.predicted_center_px) {
            const auto future = cv::Point{cvRound((*track.predicted_center_px)[0]),
                                          cvRound((*track.predicted_center_px)[1])};
            cv::line(image, current, future, {255, 255, 0}, 2, cv::LINE_AA);
            cv::circle(image, future, 8, {255, 255, 0}, 2, cv::LINE_AA);
        }
    }
}

inline auto status_json(const std::vector<Armor> &armors, double fps, const std::string &mode,
                        int width, int height, const vision::GeometryResult &geometry,
                        double timestamp, double horizon_ms,
                        const std::vector<vision::ImageTrackResult> &image_tracks) -> std::string {
    auto output = std::ostringstream{};
    output << std::setprecision(10);
    const auto red_count = std::ranges::count(armors, Color::Red, &Armor::color);
    output << "{\"mode\":\"" << mode << "\",\"red_armors\":" << red_count
           << ",\"blue_armors\":" << armors.size() - red_count << ",\"width\":" << width
           << ",\"height\":" << height << ",";
    output << "\"detected\":" << (armors.empty() ? "false" : "true")
           << ",\"armors\":" << armors.size() << ",\"corners\":[";
    auto first = true;
    for (const auto &armor : armors) {
        for (const auto &corner : armor.corners) {
            if (!first)
                output << ',';
            first = false;
            output << '[' << corner.x << ',' << corner.y << ']';
        }
    }
    output << "],\"colors\":[";
    for (std::size_t index = 0; index < armors.size(); ++index) {
        if (index != 0)
            output << ',';
        output << '\"' << (armors[index].color == Color::Red ? "red" : "blue") << '\"';
    }
    output << "],\"detect_fps\":" << fps;
    write_geometry(output, geometry, timestamp, horizon_ms);
    write_image_tracking(output, image_tracks, horizon_ms);
    output << '}';
    return output.str();
}

inline void write_status(const std::vector<Armor> &armors, double fps, const std::string &mode,
                         int width, int height, const vision::GeometryResult &geometry,
                         double timestamp, double horizon_ms,
                         const std::vector<vision::ImageTrackResult> &image_tracks) {
    const auto *status_env = std::getenv("ARMOR_STATUS_FILE");
    const auto destination =
        std::string(status_env != nullptr ? status_env : "/tmp/rmcs-blue-armor-status.json");
    const auto temporary = destination + ".tmp";
    auto output = std::ofstream{temporary};
    if (!output)
        throw std::runtime_error("cannot write status: " + temporary);
    output << status_json(armors, fps, mode, width, height, geometry, timestamp, horizon_ms,
                          image_tracks);
    output.close();
    if (std::rename(temporary.c_str(), destination.c_str()) != 0) {
        throw std::runtime_error("cannot publish status: " + destination);
    }
}

} // namespace rmcs::output
