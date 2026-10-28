#include "image_tracking.hpp"
#include <iostream>

namespace {
void require(bool value, const char* reason) {
    if (!value) throw std::runtime_error(reason);
}
auto armor_at(double x, double y) -> rmcs::armor::Armor {
    auto armor = rmcs::armor::Armor {};
    armor.corners = { cv::Point2f(x - 60, y - 20), cv::Point2f(x + 60, y - 20),
        cv::Point2f(x + 60, y + 20), cv::Point2f(x - 60, y + 20) };
    return armor;
}
}
int main() {
    try {
        auto tracker = rmcs::vision::ImageTracker { 0.1 };
        auto results = std::vector<rmcs::vision::ImageTrackResult> {};
        const auto size = cv::Size { 960, 720 };
        for (int i = 0; i < 80; ++i) {
            const auto t = i / 30.0;
            results = tracker.update({ armor_at(200 + 90 * t, 300) }, size, t);
            require(results.size() == 1 && results[0].id == 1, "image track ID changed");
            if (i < 2) require(!results[0].predicted_center_px, "prediction before confirmation");
        }
        const auto t = 79 / 30.0;
        require(results[0].confirmed && results[0].predicted_center_px.has_value(), "no prediction");
        require(std::abs(results[0].velocity_px_s[0] - 90) < 0.1, "wrong pixel velocity");
        require(std::abs((*results[0].predicted_center_px)[0] - (200 + 90 * (t + 0.1))) < 0.1,
            "wrong image prediction");
        results = tracker.update({}, size, t + 0.1);
        require(results.size() == 1 && !results[0].observed, "missing short image coasting");
        require(tracker.update({}, size, t + 0.5).empty(), "image track did not expire");
        results = tracker.update({ armor_at(300, 300), armor_at(700, 300) }, size, t + 0.6);
        const auto left = results[0].id;
        const auto right = results[1].id;
        results = tracker.update({ armor_at(700, 300), armor_at(300, 300) }, size, t + 0.63);
        require(results[0].id == left && results[0].detection_index == 1, "image left ID swap");
        require(results[1].id == right && results[1].detection_index == 0, "image right ID swap");
        auto red = armor_at(300, 300);
        red.color = rmcs::armor::Color::Red;
        require(tracker.update({ red }, { 720, 540 }, t + 0.7).empty(), "red entered image tracker");
        std::cout << "PASS: uncalibrated image prediction in pixels, velocity, stable IDs, "
                     "confirmation, coasting, expiry, resize reset, blue-only filtering\n";
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
