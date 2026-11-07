#include "armor_detector.hpp"
#include <iostream>

using namespace rmcs::armor;

namespace {
void require(bool ok, const char* why) {
    if (!ok) throw std::runtime_error(why);
}

void light(cv::Mat& image, cv::Point2f center, double length, double width,
    double roll, cv::Scalar color, double gap = 0) {
    const auto angle = roll * CV_PI / 180;
    const auto axis = cv::Point2f(std::sin(angle), std::cos(angle));
    const auto normal = cv::Point2f(axis.y, -axis.x);
    const auto pieces = gap > 0 ? 4 : 1;
    const auto part = (length - (pieces - 1) * gap) / pieces;
    for (int i = 0; i < pieces; ++i) {
        const auto start = -length / 2 + i * (part + gap);
        std::vector<cv::Point> polygon;
        for (const auto& p : std::array<cv::Point2f, 4>{
                center + axis * start - normal * (width / 2),
                center + axis * start + normal * (width / 2),
                center + axis * (start + part) + normal * (width / 2),
                center + axis * (start + part) - normal * (width / 2)})
            polygon.emplace_back(cvRound(p.x), cvRound(p.y));
        cv::fillConvexPoly(image, polygon, color);
    }
}

auto scene(double roll, double length, double separation, double width,
    cv::Scalar color, double gap = 0) -> cv::Mat {
    cv::Mat image = cv::Mat::zeros(720, 960, CV_8UC3);
    const auto angle = roll * CV_PI / 180;
    const auto normal = cv::Point2f(std::cos(angle), -std::sin(angle));
    for (const auto sign : {-1, 1})
        light(image, cv::Point2f(480, 360) + normal * (sign * separation / 2),
              length, width, roll, color, gap);
    return image;
}
}

int main() {
    try {
        auto config = Config {};
        config.close_kernel_height = 15;
        config.min_light_area = 45;
        config.min_light_height = 16;
        config.min_light_width = 3;
        config.min_light_aspect_ratio = 1.8;
        for (const auto color : {Color::Red, Color::Blue}) {
            const auto bgr = color == Color::Red ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 160, 0);
            const auto mode = color == Color::Red ? Mode::Red : Mode::Blue;
            for (const auto angle : {-60., -45., -30., 0., 30., 45., 60.}) {
                const auto result = detect(scene(angle, 118, 260, 10, bgr), config, mode);
                require(result.armors.size() == 1, "rotated armor rejected");
                require(result.armors[0].color == color, "rotated armor lost color");
                const auto& c = result.armors[0].corners;
                const auto contour = std::vector<cv::Point2f>(c.begin(), c.end());
                require(cv::isContourConvex(contour), "rotated corner order crossed");
                auto center = cv::Point2f {};
                for (const auto& p : c) center += p * .25F;
                require(cv::norm(center - cv::Point2f(480, 360)) < 2, "rotated center shifted");
                for (int i = 0; i < 4; ++i) {
                    const auto radians = angle * CV_PI / 180;
                    const auto axis = cv::Point2f(std::sin(radians), std::cos(radians));
                    const auto normal = cv::Point2f(axis.y, -axis.x);
                    const auto expected = cv::Point2f(480, 360)
                        + normal * (i == 0 || i == 3 ? -130.F : 130.F)
                        + axis * (i < 2 ? -59.F : 59.F);
                    require(cv::norm(c[i] - expected) < 3, "rotated endpoints shifted");
                }
            }
            for (const auto angle : {-45., 0., 45.}) {
                const auto result = detect(scene(angle, 220, 430, 14, bgr, 35), config, mode);
                require(result.armors.size() == 1, "large separated LED pieces not joined");
                require(result.diagnostics.fragment_merges > 0, "LED merge path not exercised");
                require(cv::norm(result.armors[0].corners[3] - result.armors[0].corners[0]) > 210,
                    "LED merge used only partial lightbar height");
            }
            require(detect(scene(30, 100, 60, 6, bgr), config, mode).armors.size() == 1,
                "foreshortened narrow armor rejected");
            cv::Mat mismatched = cv::Mat::zeros(720, 960, CV_8UC3);
            light(mismatched, {350, 360}, 118, 10, -30, bgr);
            light(mismatched, {610, 360}, 118, 10, 30, bgr);
            const auto rejected = detect(mismatched, config, mode);
            require(rejected.armors.empty() && rejected.diagnostics.rejected_pair_angle > 0,
                "non-parallel lightbars paired");
            cv::Mat shifted = cv::Mat::zeros(720, 960, CV_8UC3);
            light(shifted, {350, 280}, 100, 10, 0, bgr);
            light(shifted, {610, 440}, 100, 10, 0, bgr);
            require(detect(shifted, config, mode).armors.empty(), "diagonally unrelated lights paired");
            cv::Mat clipped = cv::Mat::zeros(720, 960, CV_8UC3);
            light(clipped, {1, 360}, 100, 10, 0, bgr);
            light(clipped, {180, 360}, 100, 10, 0, bgr);
            const auto border = detect(clipped, config, mode);
            require(border.armors.empty() && border.diagnostics.rejected_border > 0,
                "clipped lightbar produced a complete armor");
        }
        std::cout << "PASS: red/blue roll +/-60deg, ordered endpoints, large LED gaps, narrow "
                     "projection, nonparallel/diagonal negatives and clipped borders\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
