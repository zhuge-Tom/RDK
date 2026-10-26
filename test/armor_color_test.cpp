#include "armor_detector.hpp"

#include <iostream>
#include <stdexcept>

using namespace rmcs::armor;

namespace {

void require(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}

auto hue(int value) -> cv::Scalar {
    auto hsv = cv::Mat(1, 1, CV_8UC3, cv::Scalar(value, 255, 255));
    auto bgr = cv::Mat {};
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    const auto pixel = bgr.at<cv::Vec3b>(0, 0);
    return { static_cast<double>(pixel[0]), static_cast<double>(pixel[1]),
        static_cast<double>(pixel[2]) };
}

void pair(cv::Mat& frame, int x, const cv::Scalar& color) {
    cv::rectangle(frame, { x, 120, 14, 100 }, color, -1);
    cv::rectangle(frame, { x + 180, 120, 14, 100 }, color, -1);
}

auto blank() -> cv::Mat {
    return cv::Mat::zeros(480, 960, CV_8UC3);
}

} // namespace

int main() {
    try {
        const auto config = Config {};
        // Both sides of the red hue wrap must be accepted, including pure red.
        for (const auto h : { 0, 5, 175, 179 }) {
            auto frame = blank();
            pair(frame, 80, hue(h));
            const auto red = detect(frame, config, Mode::Red);
            require(red.armors.size() == 1, "red hue wrap was rejected");
            require(red.armors[0].color == Color::Red, "red color metadata missing");
            const auto& c = red.armors[0].corners;
            require(c[0].x < c[1].x && c[0].y < c[3].y && c[1].y < c[2].y,
                "corners must be TL, TR, BR, BL");
            require(detect(frame, config, Mode::Blue).armors.empty(),
                "red armor was detected as blue");
        }
        auto blue = blank();
        pair(blue, 80, { 255, 160, 0 });
        require(detect(blue, config, Mode::Blue).armors.size() == 1,
            "blue regression failed");
        require(detect(blue, config, Mode::Red).armors.empty(), "blue detected as red");

        auto both = blank();
        pair(both, 80, hue(0));
        pair(both, 550, { 255, 160, 0 });
        const auto result = detect(both, config, Mode::Both);
        require(result.armors.size() == 2, "simultaneous red/blue detection failed");
        require(result.armors[0].color != result.armors[1].color, "wrong color assignment");

        auto mixed = blank();
        cv::rectangle(mixed, { 80, 120, 14, 100 }, hue(0), -1);
        cv::rectangle(mixed, { 260, 120, 14, 100 }, { 255, 160, 0 }, -1);
        require(detect(mixed, config, Mode::Both).armors.empty(), "cross-color pairing");
        auto red_mask = cv::Mat {};
        auto blue_mask = cv::Mat {};
        auto lights = find_lightbars(mixed, red_mask, config, Color::Red);
        auto blue_lights = find_lightbars(mixed, blue_mask, config, Color::Blue);
        lights.insert(lights.end(), blue_lights.begin(), blue_lights.end());
        require(pair_lightbars(lights, config).empty(), "mixed lightbars paired");

        require(detect(blank(), config, Mode::Both).armors.empty(), "empty scene false positive");
        auto white = blank();
        pair(white, 80, { 255, 255, 255 });
        require(detect(white, config, Mode::Both).armors.empty(), "white false positive");
        auto green = blank();
        pair(green, 80, { 0, 255, 0 });
        require(detect(green, config, Mode::Both).armors.empty(), "green false positive");
        auto single = blank();
        cv::rectangle(single, { 80, 120, 14, 100 }, hue(0), -1);
        require(detect(single, config, Mode::Red).armors.empty(), "single lightbar paired");

        auto broken = blank();
        pair(broken, 80, hue(0));
        cv::rectangle(broken, { 0, 165, 960, 3 }, { 255, 255, 255 }, -1);
        require(detect(broken, config, Mode::Red).armors.size() == 1,
            "red lightbar close failed on clipped core");
        auto invalid_rejected = false;
        try { (void)parse_mode("invalid"); }
        catch (const std::invalid_argument&) { invalid_rejected = true; }
        require(invalid_rejected, "invalid mode accepted");
        std::cout << "PASS: red hue wrap, red/blue isolation, same-color pairing, "
                     "corners, blue regression, clipping, negative scenes, invalid mode\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
