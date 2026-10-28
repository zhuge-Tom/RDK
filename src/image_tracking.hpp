#pragma once

#include "armor_detector.hpp"

#include <optional>

namespace rmcs::vision {

struct ImageTrackResult {
    int id;
    int detection_index;
    bool observed;
    bool confirmed;
    double age_seconds;
    cv::Vec2d center_px;
    cv::Vec2d velocity_px_s;
    std::optional<cv::Vec2d> predicted_center_px;
};

// Image-plane tracking is available without camera intrinsics or physical sizes.
// These states are PIXELS and PIXELS/SECOND, never metric XYZ or world velocity.
class ImageTracker {
    using Matrix = cv::Matx44d;
    struct Track {
        int id;
        int detection_index;
        int samples;
        double last_observation;
        cv::Vec4d state;
        Matrix covariance;
    };
    std::vector<Track> tracks_;
    std::optional<double> previous_time_;
    cv::Size size_;
    int next_id_ = 1;
    double horizon_;

    static auto center(const Track& track) -> cv::Vec2d {
        return { track.state[0], track.state[1] };
    }
    static auto innovation_covariance(const Track& track) -> cv::Matx22d {
        return { track.covariance(0, 0) + 9, track.covariance(0, 1),
            track.covariance(1, 0), track.covariance(1, 1) + 9 };
    }
    static void correct(Track& track, const cv::Vec2d& measured) {
        const auto inverse = innovation_covariance(track).inv(cv::DECOMP_CHOLESKY);
        auto gain = cv::Matx<double, 4, 2> {};
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 2; ++c) {
                for (int k = 0; k < 2; ++k) gain(r, c) += track.covariance(r, k) * inverse(k, c);
            }
        }
        track.state += gain * (measured - center(track));
        auto ikh = Matrix::eye();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 2; ++c) ikh(r, c) -= gain(r, c);
        }
        track.covariance = ikh * track.covariance * ikh.t() + 9.0 * gain * gain.t();
        track.covariance = (track.covariance + track.covariance.t()) * 0.5;
    }

public:
    explicit ImageTracker(double horizon_seconds = 0.1) : horizon_(horizon_seconds) {
        if (!std::isfinite(horizon_) || horizon_ < 0 || horizon_ > 0.5) {
            throw std::invalid_argument("invalid image prediction horizon");
        }
    }

    auto update(const std::vector<armor::Armor>& detections, cv::Size size, double timestamp)
        -> std::vector<ImageTrackResult> {
        if (!std::isfinite(timestamp)) throw std::invalid_argument("invalid image timestamp");
        if (size != size_ || (previous_time_ && (timestamp <= *previous_time_
            || timestamp - *previous_time_ > 1.0))) {
            tracks_.clear();
            previous_time_.reset();
        }
        size_ = size;
        const auto dt = previous_time_ ? timestamp - *previous_time_ : 0;
        previous_time_ = timestamp;
        std::erase_if(tracks_, [&](const Track& track) {
            return timestamp - track.last_observation > 0.3;
        });
        auto transition = Matrix::eye();
        auto noise = Matrix::zeros();
        const auto acceleration_variance = 800.0 * 800.0;
        for (int i = 0; i < 2; ++i) {
            transition(i, i + 2) = dt;
            noise(i, i) = acceleration_variance * std::pow(dt, 4) / 4;
            noise(i, i + 2) = noise(i + 2, i) = acceleration_variance * std::pow(dt, 3) / 2;
            noise(i + 2, i + 2) = acceleration_variance * dt * dt;
        }
        for (auto& track : tracks_) {
            track.state = transition * track.state;
            track.covariance = transition * track.covariance * transition.t() + noise;
            track.detection_index = -1;
        }
        struct Observation { std::size_t index; cv::Vec2d point; };
        auto observations = std::vector<Observation> {};
        for (std::size_t i = 0; i < detections.size(); ++i) {
            if (detections[i].color != armor::Color::Blue) continue;
            auto point = cv::Vec2d {};
            for (const auto& corner : detections[i].corners) point += cv::Vec2d { corner.x, corner.y };
            observations.push_back({ i, point * 0.25 });
        }
        struct Edge { double cost; std::size_t track; std::size_t observation; };
        auto edges = std::vector<Edge> {};
        const auto gate = std::max(50.0, size.width * 0.15);
        for (std::size_t i = 0; i < tracks_.size(); ++i) {
            for (std::size_t j = 0; j < observations.size(); ++j) {
                const auto cost = cv::norm(observations[j].point - center(tracks_[i]));
                if (cost <= gate) edges.push_back({ cost, i, j });
            }
        }
        std::ranges::sort(edges, {}, &Edge::cost);
        auto used_tracks = std::vector<bool>(tracks_.size(), false);
        auto used_observations = std::vector<bool>(observations.size(), false);
        for (const auto& edge : edges) {
            if (used_tracks[edge.track] || used_observations[edge.observation]) continue;
            auto& track = tracks_[edge.track];
            correct(track, observations[edge.observation].point);
            track.detection_index = static_cast<int>(observations[edge.observation].index);
            track.last_observation = timestamp;
            ++track.samples;
            used_tracks[edge.track] = used_observations[edge.observation] = true;
        }
        for (std::size_t i = 0; i < observations.size(); ++i) {
            if (used_observations[i]) continue;
            const auto& observation = observations[i];
            auto covariance = Matrix::eye();
            covariance(0, 0) = covariance(1, 1) = 9;
            covariance(2, 2) = covariance(3, 3) = 500;
            tracks_.push_back({ next_id_++, static_cast<int>(observation.index), 1, timestamp,
                { observation.point[0], observation.point[1], 0, 0 }, covariance });
        }
        auto results = std::vector<ImageTrackResult> {};
        for (const auto& track : tracks_) {
            const auto point = center(track);
            const auto velocity = cv::Vec2d { track.state[2], track.state[3] };
            const auto confirmed = track.samples >= 3;
            results.push_back({ track.id, track.detection_index, track.detection_index >= 0,
                confirmed, timestamp - track.last_observation, point, velocity,
                confirmed ? std::optional<cv::Vec2d>(point + horizon_ * velocity) : std::nullopt });
        }
        return results;
    }
};

} // namespace rmcs::vision
