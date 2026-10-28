#include "pose_tracking.hpp"

#include <filesystem>
#include <iostream>

using namespace rmcs::vision;
namespace armor = rmcs::armor;

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
auto calibration() -> Calibration {
    auto config = Calibration {};
    config.verified = true;
    config.reason = "ready";
    config.width = 1440;
    config.height = 1080;
    config.matrix = { 1722, 0, 701, 0, 1725, 565, 0, 0, 1 };
    config.distortion = (cv::Mat_<double>(1, 5) << -0.06, -0.08, 0.001, -0.001, 0.79);
    config.armor_width_m = 0.135;
    config.armor_height_m = 0.056;
    config.max_reprojection_error_px = 1.0;
    return config;
}
auto projected_armor(const Calibration& config, cv::Size size, const cv::Vec3d& translation,
    const cv::Vec3d& rotation = { 0.15, 0.2, 0.03 }) -> armor::Armor {
    auto points = std::vector<cv::Point2d> {};
    cv::projectPoints(config.object_points(), rotation, translation,
        *config.scaled_matrix(size), config.distortion, points);
    auto result = armor::Armor {};
    for (std::size_t i = 0; i < 4; ++i) result.corners[i] = points[i];
    result.bounding_box = cv::boundingRect(std::vector<cv::Point2f>(
        result.corners.begin(), result.corners.end()));
    result.color = armor::Color::Blue;
    return result;
}
auto observation(double x, double y = 0.0, std::size_t index = 0) -> Pose {
    return { index, { x, y, 2.0 }, {}, 0.1 };
}
void calibration_and_pose_tests() {
    const auto config = calibration();
    for (const auto& size : { cv::Size { 1440, 1080 }, cv::Size { 960, 720 },
             cv::Size { 720, 540 } }) {
        const auto translation = cv::Vec3d { 0.1, -0.05, 2.5 };
        const auto detected = projected_armor(config, size, translation);
        const auto result = solve_pose(detected, 7, config, size);
        require(result.has_value(), "calibrated PnP failed");
        require(result->detection_index == 7, "lost detection index");
        require(cv::norm(result->position_m - translation) < 0.001,
            "PnP scale/axis mismatch after resizing");
        require(result->reprojection_error_px < 0.01, "unexpected reprojection error");
    }
    require(!config.scaled_matrix({ 640, 480 + 40 }), "cropped aspect ratio accepted");
    auto detected = projected_armor(config, { 960, 720 }, { 0.1, -0.05, 2.5 });
    require(!solve_pose(detected, 0, Calibration {}, { 960, 720 }),
        "missing calibration produced a pose");
    auto red = detected;
    red.color = armor::Color::Red;
    require(!solve_pose(red, 0, config, { 960, 720 }), "red should not enter blue localization");
    auto degenerate = detected;
    degenerate.corners.fill({ 0, 0 });
    require(!solve_pose(degenerate, 0, config, { 960, 720 }), "degenerate pose accepted");
    auto crossed = detected;
    std::swap(crossed.corners[1], crossed.corners[2]);
    require(!solve_pose(crossed, 0, config, { 960, 720 }), "crossed corner order accepted");
    detected.corners[1].y += 60;
    require(!solve_pose(detected, 0, config, { 960, 720 }), "high reprojection error accepted");

    auto pipeline = GeometryPipeline { Calibration {}, {} };
    const auto disabled = pipeline.process({ crossed }, { 960, 720 }, 0.0);
    require(disabled.reason == "calibration_missing" && disabled.tracks.empty(),
        "uncalibrated geometry must be explicitly disabled");

    const auto temp = std::filesystem::temp_directory_path()
        / ("rdk-pose-config-test-" + std::to_string(cv::getTickCount()) + ".yml");
    auto write = [&](int verified, double width) {
        auto file = cv::FileStorage(temp.string(), cv::FileStorage::WRITE);
        file << "calibration_verified" << verified;
        file << "image_width" << config.width << "image_height" << config.height;
        file << "camera_matrix" << cv::Mat(config.matrix);
        file << "distortion_coefficients" << config.distortion;
        file << "armor_width_m" << width << "armor_height_m" << config.armor_height_m;
    };
    write(0, config.armor_width_m);
    require(Calibration::load(temp.string()).reason == "calibration_unverified",
        "unverified calibration accepted");
    write(1, config.armor_width_m);
    require(Calibration::load(temp.string()).verified, "valid calibration rejected");
    write(1, 0.0);
    auto rejected = false;
    try { (void)Calibration::load(temp.string()); }
    catch (const std::invalid_argument&) { rejected = true; }
    std::filesystem::remove(temp);
    require(rejected, "zero physical width accepted");
}
void tracking_tests() {
    auto tracker = Tracker {};
    auto results = tracker.update({ observation(0) }, 0);
    require(!results[0].predicted_position_m, "single-frame velocity must not be predicted");
    const auto id = results[0].id;
    double time = 0.0;
    for (int i = 1; i < 100; ++i) {
        time += (i % 3 == 0) ? 0.02 : 0.04;
        results = tracker.update({ observation(0.25 * time + 0.001 * std::sin(i)) }, time);
        require(results.size() == 1 && results[0].id == id, "track ID changed on smooth motion");
    }
    require(results[0].confirmed && results[0].predicted_position_m.has_value(),
        "confirmed track missing prediction");
    require(std::abs(results[0].velocity_m_s[0] - 0.25) < 0.02, "velocity estimate inaccurate");
    require(std::abs((*results[0].predicted_position_m)[0] - 0.25 * (time + 0.1)) < 0.005,
        "100ms prediction inaccurate");
    results = tracker.update({}, time + 0.1);
    require(results.size() == 1 && !results[0].observed && results[0].detection_index == -1,
        "short occlusion must be marked as coasting");
    results = tracker.update({}, time + 0.5);
    require(results.empty(), "expired track still published");

    auto multiple = Tracker {};
    results = multiple.update({ observation(-0.7, 0, 0), observation(0.7, 0, 1) }, 0);
    const auto left = results[0].id;
    const auto right = results[1].id;
    for (int i = 1; i < 20; ++i) {
        // Detection ordering swaps; identities must follow geometry, not array index.
        results = multiple.update({ observation(0.7, 0, 0), observation(-0.7, 0, 1) }, i * 0.03);
        require(results.size() == 2, "unexpected duplicate track");
        require(results[0].id == left && results[0].detection_index == 1,
            "left track followed wrong detection index");
        require(results[1].id == right && results[1].detection_index == 0,
            "right track followed wrong detection index");
    }
    const auto stable_id = results[0].id;
    results = multiple.update({ observation(8) }, 0.61);
    require(results.back().id != stable_id && !results.back().confirmed,
        "large jump inherited old target velocity");
    results = multiple.update({ observation(1) }, 0.1);
    require(results.size() == 1 && !results[0].confirmed,
        "backwards timestamp must reset tracking");
    auto invalid = false;
    try { (void)Tracker { TrackingConfig { .prediction_seconds = 2.0 } }; }
    catch (const std::invalid_argument&) { invalid = true; }
    require(invalid, "unbounded prediction horizon accepted");
}
void pipeline_tests() {
    const auto config = calibration();
    const auto size = cv::Size { 960, 720 };
    auto pipeline = GeometryPipeline { config, {} };
    GeometryResult result;
    for (int i = 0; i < 60; ++i) {
        const auto t = i / 30.0;
        result = pipeline.process({ projected_armor(config, size, { 0.1 * t, 0.03, 2.5 }) },
            size, t);
    }
    require(result.reason == "ready" && result.poses.size() == 1 && result.tracks.size() == 1,
        "PnP-to-tracker pipeline failed");
    require(result.tracks[0].predicted_position_m.has_value(), "pipeline prediction missing");
    require(std::abs(result.tracks[0].velocity_m_s[0] - 0.1) < 0.005,
        "PnP-to-tracker velocity inaccurate");
    require(pipeline.project(*result.tracks[0].predicted_position_m, size).has_value(),
        "prediction projection failed");
    result = pipeline.process({}, { 640, 400 }, 2.1);
    require(result.reason == "image_aspect_mismatch" && result.tracks.empty(),
        "resolution change did not invalidate geometry");
}
} // namespace

int main() {
    try {
        calibration_and_pose_tests();
        tracking_tests();
        pipeline_tests();
        std::cout << "PASS: calibration, distorted/resized PnP, invalid corners, "
                     "time-based CV Kalman, stable IDs, occlusion, expiry, jumps, "
                     "blue PnP-to-prediction pipeline\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
