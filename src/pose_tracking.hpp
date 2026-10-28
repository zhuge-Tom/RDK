#pragma once

#include "armor_detector.hpp"

#include <limits>
#include <optional>
#include <opencv2/calib3d.hpp>

namespace rmcs::vision {

// OpenCV optical frame, in meters: X right, Y down, Z forward.
struct Calibration {
    bool verified = false;
    int width = 0;
    int height = 0;
    cv::Matx33d matrix = cv::Matx33d::eye();
    cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
    double armor_width_m = 0.0;
    double armor_height_m = 0.0;
    double max_reprojection_error_px = 4.0;
    std::string reason = "calibration_missing";

    static auto load(const std::string& path) -> Calibration {
        auto config = Calibration {};
        if (path.empty()) return config;
        auto file = cv::FileStorage(path, cv::FileStorage::READ);
        if (!file.isOpened()) throw std::invalid_argument("cannot read calibration: " + path);
        int verified = 0;
        file["calibration_verified"] >> verified;
        if (verified != 1) {
            config.reason = "calibration_unverified";
            return config;
        }
        file["image_width"] >> config.width;
        file["image_height"] >> config.height;
        file["armor_width_m"] >> config.armor_width_m;
        file["armor_height_m"] >> config.armor_height_m;
        auto matrix = cv::Mat {};
        file["camera_matrix"] >> matrix;
        file["distortion_coefficients"] >> config.distortion;
        if (!file["max_reprojection_error_px"].empty()) {
            file["max_reprojection_error_px"] >> config.max_reprojection_error_px;
        }
        if (config.width <= 0 || config.height <= 0 || matrix.rows != 3 || matrix.cols != 3
            || matrix.channels() != 1 || !cv::checkRange(matrix)
            || !std::isfinite(config.armor_width_m) || config.armor_width_m <= 0.0
            || !std::isfinite(config.armor_height_m) || config.armor_height_m <= 0.0
            || !std::isfinite(config.max_reprojection_error_px)
            || config.max_reprojection_error_px <= 0.0) {
            throw std::invalid_argument("invalid calibration dimensions, matrix or armor size");
        }
        matrix.convertTo(matrix, CV_64F);
        config.matrix = matrix;
        if (config.matrix(0, 0) <= 0 || config.matrix(1, 1) <= 0
            || std::abs(config.matrix(2, 2) - 1.0) > 1e-9
            || std::abs(config.matrix(2, 0)) > 1e-9
            || std::abs(config.matrix(2, 1)) > 1e-9) {
            throw std::invalid_argument("invalid camera intrinsic matrix");
        }
        const auto count = config.distortion.total();
        if (config.distortion.channels() != 1 || !cv::checkRange(config.distortion)
            || !(count == 4 || count == 5 || count == 8 || count == 12 || count == 14)) {
            throw std::invalid_argument("distortion coefficients must contain 4/5/8/12/14 values");
        }
        config.distortion = config.distortion.reshape(1, 1);
        config.distortion.convertTo(config.distortion, CV_64F);
        config.verified = true;
        config.reason = "ready";
        return config;
    }

    auto scaled_matrix(cv::Size image_size) const -> std::optional<cv::Matx33d> {
        if (!verified || width <= 0 || height <= 0
            || image_size.width <= 0 || image_size.height <= 0) return std::nullopt;
        const auto sx = image_size.width / static_cast<double>(width);
        const auto sy = image_size.height / static_cast<double>(height);
        // Only full-frame resizing is supported. Cropped images need their own calibration.
        if (std::abs(sx / sy - 1.0) > 0.001) return std::nullopt;
        auto result = matrix;
        for (int column = 0; column < 3; ++column) {
            result(0, column) *= sx;
            result(1, column) *= sy;
        }
        return result;
    }

    auto object_points() const -> std::vector<cv::Point3d> {
        const auto w = armor_width_m * 0.5;
        const auto h = armor_height_m * 0.5;
        return { { -w, -h, 0 }, { w, -h, 0 }, { w, h, 0 }, { -w, h, 0 } };
    }
};

struct Pose {
    std::size_t detection_index;
    cv::Vec3d position_m;
    cv::Vec3d rotation_vector;
    double reprojection_error_px;
};

inline auto solve_pose(const armor::Armor& detected, std::size_t index,
    const Calibration& calibration, cv::Size image_size) -> std::optional<Pose> {
    if (detected.color != armor::Color::Blue) return std::nullopt;
    const auto matrix = calibration.scaled_matrix(image_size);
    if (!matrix) return std::nullopt;
    const auto object = calibration.object_points();
    auto corners = std::vector<cv::Point2d> {};
    for (const auto& point : detected.corners) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
        corners.emplace_back(point.x, point.y);
    }
    const auto contour = std::vector<cv::Point2f>(detected.corners.begin(), detected.corners.end());
    if (!cv::isContourConvex(contour) || std::abs(cv::contourArea(contour)) < 4.0) {
        return std::nullopt;
    }
    try {
        auto rotations = std::vector<cv::Mat> {};
        auto translations = std::vector<cv::Mat> {};
        if (cv::solvePnPGeneric(object, corners, *matrix, calibration.distortion,
                rotations, translations, false, cv::SOLVEPNP_IPPE) == 0) return std::nullopt;
        auto best = std::optional<Pose> {};
        for (std::size_t i = 0; i < rotations.size(); ++i) {
            const auto& t = translations[i];
            const auto& r = rotations[i];
            if (!cv::checkRange(t) || !cv::checkRange(r) || t.at<double>(2) <= 0.01) continue;
            auto projected = std::vector<cv::Point2d> {};
            cv::projectPoints(object, r, t, *matrix, calibration.distortion, projected);
            auto squared_error = 0.0;
            for (std::size_t j = 0; j < corners.size(); ++j) {
                const auto delta = projected[j] - corners[j];
                squared_error += delta.dot(delta);
            }
            const auto error = std::sqrt(squared_error / corners.size());
            if (!std::isfinite(error) || error > calibration.max_reprojection_error_px
                || (best && error >= best->reprojection_error_px)) continue;
            best = Pose { index,
                { t.at<double>(0), t.at<double>(1), t.at<double>(2) },
                { r.at<double>(0), r.at<double>(1), r.at<double>(2) }, error };
        }
        return best;
    } catch (const cv::Exception&) {
        return std::nullopt;
    }
}

struct TrackingConfig {
    double prediction_seconds = 0.1;
    double timeout_seconds = 0.3;
    double acceleration_stddev = 3.0;
    double observation_stddev = 0.03;
    double association_distance_m = 0.5;
    int confirmation_samples = 3;
};

struct TrackResult {
    int id;
    int detection_index; // -1: no observation in this frame, only a short coasting estimate
    bool observed;
    bool confirmed;
    double observation_age_seconds;
    cv::Vec3d position_m;
    cv::Vec3d velocity_m_s;
    std::optional<cv::Vec3d> predicted_position_m;
};

class Tracker {
    using State = cv::Vec<double, 6>;
    using Matrix = cv::Matx<double, 6, 6>;
    struct Track {
        int id;
        int detection_index;
        int samples;
        double last_observation;
        State state;
        Matrix covariance;
    };
    TrackingConfig config_;
    std::vector<Track> tracks_;
    std::optional<double> previous_time_;
    int next_id_ = 1;

    static auto position(const Track& track) -> cv::Vec3d {
        return { track.state[0], track.state[1], track.state[2] };
    }
    static auto velocity(const Track& track) -> cv::Vec3d {
        return { track.state[3], track.state[4], track.state[5] };
    }
    auto observation_covariance(const Track& track) const -> cv::Matx33d {
        auto result = cv::Matx33d {};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) result(i, j) = track.covariance(i, j);
            result(i, i) += config_.observation_stddev * config_.observation_stddev;
        }
        return result;
    }
    auto correct(Track& track, const Pose& observation) const -> void {
        const auto inverse = observation_covariance(track).inv(cv::DECOMP_CHOLESKY);
        auto gain = cv::Matx<double, 6, 3> {};
        for (int row = 0; row < 6; ++row) {
            for (int column = 0; column < 3; ++column) {
                for (int k = 0; k < 3; ++k) {
                    gain(row, column) += track.covariance(row, k) * inverse(k, column);
                }
            }
        }
        track.state += gain * (observation.position_m - position(track));
        auto ikh = Matrix::eye();
        for (int row = 0; row < 6; ++row) {
            for (int column = 0; column < 3; ++column) ikh(row, column) -= gain(row, column);
        }
        const auto variance = config_.observation_stddev * config_.observation_stddev;
        track.covariance = ikh * track.covariance * ikh.t() + variance * (gain * gain.t());
        track.covariance = (track.covariance + track.covariance.t()) * 0.5;
    }

public:
    explicit Tracker(TrackingConfig config = {}) : config_(config) {
        if (!std::isfinite(config.prediction_seconds) || config.prediction_seconds < 0.0
            || config.prediction_seconds > 0.5 || config.timeout_seconds <= 0
            || config.observation_stddev <= 0 || config.acceleration_stddev <= 0
            || config.association_distance_m <= 0 || config.confirmation_samples < 2) {
            throw std::invalid_argument("invalid short-horizon tracking configuration");
        }
    }

    void reset() {
        tracks_.clear();
        previous_time_.reset();
    }

    auto update(const std::vector<Pose>& observations, double timestamp)
        -> std::vector<TrackResult> {
        if (!std::isfinite(timestamp)) throw std::invalid_argument("timestamp must be finite");
        if (previous_time_ && (timestamp <= *previous_time_
            || timestamp - *previous_time_ > 1.0)) reset();
        const auto dt = previous_time_ ? timestamp - *previous_time_ : 0.0;
        previous_time_ = timestamp;
        std::erase_if(tracks_, [&](const Track& track) {
            return timestamp - track.last_observation > config_.timeout_seconds;
        });
        auto transition = Matrix::eye();
        auto noise = Matrix::zeros();
        for (int i = 0; i < 3; ++i) {
            transition(i, i + 3) = dt;
            const auto a = config_.acceleration_stddev * config_.acceleration_stddev;
            noise(i, i) = a * std::pow(dt, 4) / 4;
            noise(i, i + 3) = noise(i + 3, i) = a * std::pow(dt, 3) / 2;
            noise(i + 3, i + 3) = a * dt * dt;
        }
        for (auto& track : tracks_) {
            track.state = transition * track.state;
            track.covariance = transition * track.covariance * transition.t() + noise;
            track.detection_index = -1;
        }
        struct Edge { double cost; std::size_t track; std::size_t observation; };
        auto edges = std::vector<Edge> {};
        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            const auto inverse = observation_covariance(tracks_[i]).inv(cv::DECOMP_CHOLESKY);
            for (std::size_t j = 0; j < observations.size(); ++j) {
                const auto delta = observations[j].position_m - position(tracks_[i]);
                const auto cost = delta.dot(inverse * delta);
                if (cv::norm(delta) <= config_.association_distance_m && cost < 16.27) {
                    edges.push_back({ cost, i, j });
                }
            }
        }
        std::ranges::sort(edges, {}, &Edge::cost);
        auto used_tracks = std::vector<bool>(tracks_.size(), false);
        auto used_observations = std::vector<bool>(observations.size(), false);
        for (const auto& edge : edges) {
            if (used_tracks[edge.track] || used_observations[edge.observation]) continue;
            auto& track = tracks_[edge.track];
            correct(track, observations[edge.observation]);
            track.last_observation = timestamp;
            track.detection_index = static_cast<int>(observations[edge.observation].detection_index);
            ++track.samples;
            used_tracks[edge.track] = used_observations[edge.observation] = true;
        }
        for (std::size_t i = 0; i < observations.size(); ++i) {
            if (used_observations[i]) continue;
            const auto& point = observations[i].position_m;
            auto covariance = Matrix::eye();
            for (int axis = 0; axis < 3; ++axis) covariance(axis, axis) = 0.01;
            tracks_.push_back({ next_id_++, static_cast<int>(observations[i].detection_index),
                1, timestamp, { point[0], point[1], point[2], 0, 0, 0 }, covariance });
        }
        auto results = std::vector<TrackResult> {};
        for (const auto& track : tracks_) {
            const auto confirmed = track.samples >= config_.confirmation_samples;
            const auto p = position(track);
            const auto v = velocity(track);
            results.push_back({ track.id, track.detection_index, track.detection_index >= 0,
                confirmed, timestamp - track.last_observation, p, v,
                confirmed ? std::optional<cv::Vec3d>(p + config_.prediction_seconds * v)
                          : std::nullopt });
        }
        return results;
    }
};

struct GeometryResult {
    std::string reason;
    std::vector<Pose> poses;
    std::vector<TrackResult> tracks;
};

class GeometryPipeline {
    Calibration calibration_;
    Tracker tracker_;
public:
    GeometryPipeline(Calibration calibration, TrackingConfig tracking)
        : calibration_(std::move(calibration)), tracker_(tracking) {}
    auto process(const std::vector<armor::Armor>& armors, cv::Size size, double timestamp)
        -> GeometryResult {
        auto result = GeometryResult {};
        result.reason = calibration_.reason;
        if (!calibration_.verified) return result;
        if (!calibration_.scaled_matrix(size)) {
            tracker_.reset();
            result.reason = "image_aspect_mismatch";
            return result;
        }
        for (std::size_t i = 0; i < armors.size(); ++i) {
            if (auto pose = solve_pose(armors[i], i, calibration_, size)) {
                result.poses.push_back(*pose);
            }
        }
        result.tracks = tracker_.update(result.poses, timestamp);
        return result;
    }

    auto project(const cv::Vec3d& point, cv::Size size) const -> std::optional<cv::Point> {
        const auto matrix = calibration_.scaled_matrix(size);
        if (!matrix || point[2] <= 0.01) return std::nullopt;
        auto pixels = std::vector<cv::Point2d> {};
        cv::projectPoints(std::vector<cv::Point3d> { { point[0], point[1], point[2] } },
            cv::Vec3d {}, cv::Vec3d {}, *matrix, calibration_.distortion, pixels);
        const auto& pixel = pixels.front();
        if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y)
            || pixel.x < 0 || pixel.y < 0 || pixel.x >= size.width || pixel.y >= size.height) {
            return std::nullopt;
        }
        return cv::Point { cvRound(pixel.x), cvRound(pixel.y) };
    }
};

} // namespace rmcs::vision
