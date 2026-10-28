#include "mvs_camera.hpp"
#include "ros_nodes.hpp"
#include "vision_output.hpp"

#include <chrono>
#include <cstring>
#include <deque>
#include <filesystem>
#include <map>
#include <mutex>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rdk_s100_blue_armor_detector/msg/armors.hpp>
#include <rdk_s100_blue_armor_detector/msg/vision_result.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rmcs::ros {
namespace msg = rdk_s100_blue_armor_detector::msg;
using Image = sensor_msgs::msg::Image;
using Result = msg::VisionResult;
using Trigger = std_srvs::srv::Trigger;
using Steady = std::chrono::steady_clock;

// Startup parameters are explicit and read-only; changes take effect on restart.
template <typename T> T parameter(rclcpp::Node &node, const std::string &name, T fallback) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, fallback, descriptor);
}

auto frame_qos() { return rclcpp::SensorDataQoS{}.keep_last(1); }
auto stamp_key(const std_msgs::msg::Header &header) -> int64_t {
    return rclcpp::Time(header.stamp).nanoseconds();
}
auto image_view(const Image &image) -> cv::Mat {
    if (image.encoding != "bgr8" || image.step < image.width * 3 ||
        image.data.size() < static_cast<std::size_t>(image.step) * image.height) {
        throw std::invalid_argument("expected a valid bgr8 image");
    }
    return {static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3,
            const_cast<unsigned char *>(image.data.data()), image.step};
}

template <typename Point> auto vector3(const Point &p) -> cv::Vec3d { return {p.x, p.y, p.z}; }
template <typename Point> void set_vector(Point &p, const cv::Vec3d &v) {
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
}

auto unpack_armors(const std::vector<msg::Armor> &input) -> std::vector<armor::Armor> {
    std::vector<armor::Armor> output;
    for (const auto &source : input) {
        armor::Armor target{};
        for (std::size_t i = 0; i < 4; ++i)
            target.corners[i] = {source.corners[i].x, source.corners[i].y};
        target.bounding_box = cv::boundingRect(
            std::vector<cv::Point2f>(target.corners.begin(), target.corners.end()));
        target.color = source.color == msg::Armor::RED ? armor::Color::Red : armor::Color::Blue;
        target.score = source.score;
        output.push_back(target);
    }
    return output;
}

class HikCameraNode final : public rclcpp::Node {
    std::unique_ptr<mvs::Camera> camera_;
    cv::Mat fixture_, original_;
    std::mutex frame_mutex_;
    Steady::time_point last_capture_{};
    std::string capture_directory_, frame_id_;
    vision::Calibration calibration_;
    int width_ = 960;
    rclcpp::Publisher<Image>::SharedPtr images_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
    rclcpp::Service<Trigger>::SharedPtr capture_service_;
    rclcpp::CallbackGroup::SharedPtr acquisition_group_;
    rclcpp::TimerBase::SharedPtr timer_;

    void capture() {
        auto original = camera_ ? camera_->read(100) : fixture_.clone();
        if (original.empty())
            return;
        const auto stamp = now();
        {
            std::lock_guard lock(frame_mutex_);
            original_ = original;
            last_capture_ = Steady::now();
        }
        cv::Mat image = original;
        if (image.cols > width_) {
            const auto height = cvRound(image.rows * width_ / static_cast<double>(image.cols));
            cv::resize(original, image, {width_, height}, 0, 0, cv::INTER_AREA);
        }
        auto message = std::make_unique<Image>();
        message->header.stamp = stamp;
        message->header.frame_id = frame_id_;
        message->width = image.cols;
        message->height = image.rows;
        message->encoding = "bgr8";
        message->is_bigendian = false;
        message->step = image.cols * 3;
        message->data.resize(static_cast<std::size_t>(message->step) * image.rows);
        for (int row = 0; row < image.rows; ++row) {
            std::memcpy(message->data.data() + row * message->step, image.ptr(row), message->step);
        }
        auto info = sensor_msgs::msg::CameraInfo{};
        info.header = message->header;
        info.width = message->width;
        info.height = message->height;
        info.distortion_model = "plumb_bob";
        info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        if (auto matrix = calibration_.scaled_matrix(image.size())) {
            std::copy(std::begin(matrix->val), std::end(matrix->val), info.k.begin());
            info.d.assign(calibration_.distortion.ptr<double>(),
                          calibration_.distortion.ptr<double>() + calibration_.distortion.total());
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    info.p[r * 4 + c] = (*matrix)(r, c);
        }
        // K[0] remains zero when uncalibrated, following CameraInfo semantics.
        info_->publish(info);
        images_->publish(std::move(message));
    }

  public:
    explicit HikCameraNode(const rclcpp::NodeOptions &options) : Node("hik_camera", options) {
        const auto exposure = parameter(*this, "exposure_us", 3500.0);
        const auto gain = parameter(*this, "gain", 2.0);
        const auto fps = parameter(*this, "fps", 30.0);
        width_ = static_cast<int>(parameter(*this, "processing_width", int64_t{960}));
        capture_directory_ =
            parameter(*this, "capture_directory", std::string{"calibration/captures"});
        frame_id_ = parameter(*this, "camera_frame", std::string{"camera_optical"});
        calibration_ =
            vision::Calibration::load(parameter(*this, "calibration_path", std::string{}));
        const auto fixture = parameter(*this, "test_image", std::string{});
        if (!std::isfinite(fps) || fps <= 0 || fps > 240 || width_ <= 0 ||
            !std::isfinite(exposure) || exposure <= 0 || !std::isfinite(gain) || gain < 0) {
            throw std::invalid_argument("invalid camera startup parameters");
        }
        if (fixture.empty())
            camera_ = std::make_unique<mvs::Camera>(exposure, gain, fps);
        else {
            fixture_ = cv::imread(fixture, cv::IMREAD_COLOR);
            if (fixture_.empty())
                throw std::invalid_argument("cannot read test_image");
            RCLCPP_WARN(get_logger(), "OFFLINE TEST IMAGE: camera is not opened");
        }
        images_ = create_publisher<Image>("/camera/image_raw", frame_qos());
        info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", frame_qos());
        capture_service_ = create_service<Trigger>(
            "/camera/save_calibration_frame",
            [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
                cv::Mat snapshot;
                {
                    std::lock_guard lock(frame_mutex_);
                    if (original_.empty() || Steady::now() - last_capture_ > std::chrono::seconds(1)) {
                        response->message = "No fresh camera frame";
                        return;
                    }
                    // Keep this immutable frame alive while acquisition replaces original_.
                    snapshot = original_;
                }
                try {
                    const auto root = std::filesystem::path{capture_directory_};
                    std::filesystem::create_directories(root);
                    std::ofstream(root / "capture.request").put('\n');
                    output::capture_calibration_frame(snapshot, capture_directory_);
                    std::ifstream report(root / "last_capture.json");
                    response->message = std::string(std::istreambuf_iterator<char>(report), {});
                    response->success =
                        response->message.find("\"saved\":true") != std::string::npos;
                } catch (const std::exception &error) {
                    response->message = error.what();
                }
            });
        // Blocking MVS reads must not monopolize the services' callback group.
        acquisition_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / fps),
                                  [this] { capture(); }, acquisition_group_);
        RCLCPP_INFO(get_logger(), "MVS camera: %.0f FPS, exposure %.0f us, processing width %d",
                    fps, exposure, width_);
    }
};

class ArmorDetectionNode final : public rclcpp::Node {
    armor::Config config_;
    std::string mode_;
    armor::Mode selection_;
    std::deque<double> stamps_;
    rclcpp::Publisher<msg::Armors>::SharedPtr publisher_;
    rclcpp::Subscription<Image>::SharedPtr subscription_;

  public:
    explicit ArmorDetectionNode(const rclcpp::NodeOptions &options)
        : Node("armor_detector", options) {
        mode_ = parameter(*this, "color", std::string{"blue"});
        selection_ = armor::parse_mode(mode_);
#define INT_PARAM(name)                                                                            \
    config_.name = static_cast<int>(parameter(*this, #name, int64_t{config_.name}))
#define DOUBLE_PARAM(name) config_.name = parameter(*this, #name, config_.name)
        INT_PARAM(hue_min);
        INT_PARAM(hue_max);
        INT_PARAM(saturation_min);
        INT_PARAM(value_min);
        INT_PARAM(gray_min);
        INT_PARAM(close_kernel_height);
        INT_PARAM(min_light_height);
        INT_PARAM(min_light_width);
        INT_PARAM(max_light_height);
        INT_PARAM(max_light_width);
        DOUBLE_PARAM(min_light_area);
        DOUBLE_PARAM(max_light_area);
        DOUBLE_PARAM(min_light_aspect_ratio);
        DOUBLE_PARAM(min_blue_red_ratio);
        DOUBLE_PARAM(max_pair_height_ratio);
        DOUBLE_PARAM(max_pair_y_ratio);
        DOUBLE_PARAM(min_pair_x_ratio);
        DOUBLE_PARAM(max_pair_x_ratio);
        DOUBLE_PARAM(min_armor_aspect_ratio);
        DOUBLE_PARAM(max_armor_aspect_ratio);
#undef INT_PARAM
#undef DOUBLE_PARAM
        if (config_.hue_min < 0 || config_.hue_max > 179 || config_.hue_min > config_.hue_max ||
            config_.gray_min < 0 || config_.gray_min > 255 || config_.close_kernel_height <= 0) {
            throw std::invalid_argument("invalid detector thresholds");
        }
        publisher_ = create_publisher<msg::Armors>("/armor/detections", frame_qos());
        subscription_ = create_subscription<Image>(
            "/camera/image_raw", frame_qos(), [this](Image::ConstSharedPtr image) {
                const auto start = Steady::now();
                const auto result = armor::detect(image_view(*image), config_, selection_);
                auto output = std::make_unique<msg::Armors>();
                output->header = image->header;
                output->image_width = image->width;
                output->image_height = image->height;
                output->mode = mode_;
                output->detection_ms =
                    std::chrono::duration<double, std::milli>(Steady::now() - start).count();
                const auto stamp = rclcpp::Time(image->header.stamp).seconds();
                if (!stamps_.empty() && stamp <= stamps_.back())
                    stamps_.clear();
                stamps_.push_back(stamp);
                if (stamps_.size() > 30)
                    stamps_.pop_front();
                if (stamps_.size() > 1 && stamps_.back() > stamps_.front())
                    output->detect_fps = (stamps_.size() - 1) / (stamps_.back() - stamps_.front());
                for (const auto &a : result.armors) {
                    msg::Armor target;
                    target.color =
                        a.color == armor::Color::Blue ? msg::Armor::BLUE : msg::Armor::RED;
                    target.score = a.score;
                    for (std::size_t i = 0; i < 4; ++i) {
                        target.corners[i].x = a.corners[i].x;
                        target.corners[i].y = a.corners[i].y;
                    }
                    output->armors.push_back(target);
                }
                publisher_->publish(std::move(output));
            });
    }
};

class ArmorTrackingNode final : public rclcpp::Node {
    std::unique_ptr<vision::GeometryPipeline> geometry_;
    std::unique_ptr<vision::ImageTracker> image_;
    double horizon_ms_;
    rclcpp::Publisher<Result>::SharedPtr publisher_;
    rclcpp::Subscription<msg::Armors>::SharedPtr subscription_;

  public:
    explicit ArmorTrackingNode(const rclcpp::NodeOptions &options)
        : Node("armor_tracker", options) {
        horizon_ms_ = parameter(*this, "prediction_ms", 100.0);
        auto calibration =
            vision::Calibration::load(parameter(*this, "calibration_path", std::string{}));
        vision::TrackingConfig config;
        config.prediction_seconds = horizon_ms_ / 1000.0;
        image_ = std::make_unique<vision::ImageTracker>(config.prediction_seconds);
        geometry_ = std::make_unique<vision::GeometryPipeline>(calibration, config);
        publisher_ = create_publisher<Result>("/armor/tracks", frame_qos());
        subscription_ = create_subscription<msg::Armors>(
            "/armor/detections", frame_qos(), [this](msg::Armors::ConstSharedPtr detections) {
                const auto armors = unpack_armors(detections->armors);
                const auto size = cv::Size{static_cast<int>(detections->image_width),
                                           static_cast<int>(detections->image_height)};
                const auto timestamp = rclcpp::Time(detections->header.stamp).seconds();
                const auto geometry = geometry_->process(armors, size, timestamp);
                const auto tracks = image_->update(armors, size, timestamp);
                auto result = std::make_unique<Result>();
                result->header = detections->header;
                result->image_width = detections->image_width;
                result->image_height = detections->image_height;
                result->mode = detections->mode;
                result->detect_fps = detections->detect_fps;
                result->detection_ms = detections->detection_ms;
                result->armors = detections->armors;
                result->geometry_reason = geometry.reason;
                result->prediction_horizon_ms = horizon_ms_;
                for (const auto &t : tracks) {
                    msg::ImageTrack out;
                    out.id = t.id;
                    out.detection_index = t.detection_index;
                    out.observed = t.observed;
                    out.confirmed = t.confirmed;
                    out.observation_age_seconds = t.age_seconds;
                    out.center_px.x = t.center_px[0];
                    out.center_px.y = t.center_px[1];
                    out.velocity_px_s.x = t.velocity_px_s[0];
                    out.velocity_px_s.y = t.velocity_px_s[1];
                    out.prediction_valid = t.predicted_center_px.has_value();
                    if (t.predicted_center_px) {
                        out.predicted_center_px.x = (*t.predicted_center_px)[0];
                        out.predicted_center_px.y = (*t.predicted_center_px)[1];
                    }
                    result->image_tracks.push_back(out);
                }
                for (const auto &p : geometry.poses) {
                    msg::ArmorPose out;
                    out.detection_index = p.detection_index;
                    set_vector(out.position_m, p.position_m);
                    set_vector(out.rotation_vector, p.rotation_vector);
                    out.distance_m = cv::norm(p.position_m);
                    out.reprojection_error_px = p.reprojection_error_px;
                    result->poses.push_back(out);
                }
                for (const auto &t : geometry.tracks) {
                    msg::SpatialTrack out;
                    out.id = t.id;
                    out.detection_index = t.detection_index;
                    out.observed = t.observed;
                    out.confirmed = t.confirmed;
                    out.observation_age_seconds = t.observation_age_seconds;
                    set_vector(out.position_m, t.position_m);
                    set_vector(out.velocity_m_s, t.velocity_m_s);
                    out.prediction_valid = t.predicted_position_m.has_value();
                    if (t.predicted_position_m)
                        set_vector(out.predicted_position_m, *t.predicted_position_m);
                    result->tracks.push_back(out);
                }
                publisher_->publish(std::move(result));
            });
        RCLCPP_INFO(get_logger(), "Tracking: %s, prediction %.0f ms", calibration.reason.c_str(),
                    horizon_ms_);
    }
};

class ArmorVisualizationNode final : public rclcpp::Node {
    std::unique_ptr<vision::GeometryPipeline> geometry_;
    std::map<int64_t, Image::ConstSharedPtr> images_;
    std::map<int64_t, Result::ConstSharedPtr> results_;
    int quality_;
    double max_age_ms_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
    rclcpp::Subscription<Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<Result>::SharedPtr result_subscription_;

    void render(int64_t key) {
        while (images_.size() > 8)
            images_.erase(images_.begin());
        while (results_.size() > 8)
            results_.erase(results_.begin());
        if (!images_.contains(key) || !results_.contains(key))
            return;
        auto image = images_.at(key);
        auto result = results_.at(key);
        images_.erase(images_.begin(), images_.upper_bound(key));
        results_.erase(results_.begin(), results_.upper_bound(key));
        const auto age_ms = (now() - rclcpp::Time(result->header.stamp)).seconds() * 1000.0;
        if (age_ms < 0 || age_ms > max_age_ms_)
            return;
        auto frame = image_view(*image).clone();
        const auto armors = unpack_armors(result->armors);
        vision::GeometryResult geometry;
        geometry.reason = result->geometry_reason;
        for (const auto &p : result->poses)
            geometry.poses.push_back({p.detection_index, vector3(p.position_m),
                                      vector3(p.rotation_vector), p.reprojection_error_px});
        for (const auto &t : result->tracks) {
            vision::TrackResult out{t.id,
                                    t.detection_index,
                                    t.observed,
                                    t.confirmed,
                                    t.observation_age_seconds,
                                    vector3(t.position_m),
                                    vector3(t.velocity_m_s),
                                    {}};
            if (t.prediction_valid)
                out.predicted_position_m = vector3(t.predicted_position_m);
            geometry.tracks.push_back(out);
        }
        std::vector<vision::ImageTrackResult> tracks;
        for (const auto &t : result->image_tracks) {
            vision::ImageTrackResult out{t.id,
                                         t.detection_index,
                                         t.observed,
                                         t.confirmed,
                                         t.observation_age_seconds,
                                         {t.center_px.x, t.center_px.y},
                                         {t.velocity_px_s.x, t.velocity_px_s.y},
                                         {}};
            if (t.prediction_valid)
                out.predicted_center_px =
                    cv::Vec2d{t.predicted_center_px.x, t.predicted_center_px.y};
            tracks.push_back(out);
        }
        armor::draw(frame, {}, armors, false);
        if (geometry.reason == "ready" && !geometry.poses.empty())
            output::draw_geometry(frame, geometry, *geometry_);
        else
            output::draw_image_tracking(frame, tracks);
        sensor_msgs::msg::CompressedImage compressed;
        compressed.header = image->header;
        compressed.format = "jpeg";
        if (!cv::imencode(".jpg", frame, compressed.data, {cv::IMWRITE_JPEG_QUALITY, quality_}))
            return;
        auto json = output::status_json(
            armors, result->detect_fps, result->mode, frame.cols, frame.rows, geometry,
            rclcpp::Time(result->header.stamp).seconds(), result->prediction_horizon_ms, tracks);
        json.pop_back();
        json += ",\"runtime\":\"ros2\",\"capture_to_encode_ms\":" +
                std::to_string((now() - rclcpp::Time(result->header.stamp)).seconds() * 1000.0) +
                ",\"detection_ms\":" + std::to_string(result->detection_ms) + '}';
        std_msgs::msg::String status;
        status.data = std::move(json);
        status_->publish(status);
        publisher_->publish(std::move(compressed));
    }

  public:
    explicit ArmorVisualizationNode(const rclcpp::NodeOptions &options)
        : Node("armor_visualizer", options) {
        quality_ = static_cast<int>(parameter(*this, "jpeg_quality", int64_t{75}));
        max_age_ms_ = parameter(*this, "max_frame_age_ms", 250.0);
        if (quality_ < 1 || quality_ > 100 || !std::isfinite(max_age_ms_) || max_age_ms_ <= 0)
            throw std::invalid_argument("invalid visualization parameters");
        geometry_ = std::make_unique<vision::GeometryPipeline>(
            vision::Calibration::load(parameter(*this, "calibration_path", std::string{})),
            vision::TrackingConfig{});
        publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/armor/visualization/compressed", frame_qos());
        status_ = create_publisher<std_msgs::msg::String>("/armor/status", rclcpp::QoS(1));
        image_subscription_ = create_subscription<Image>(
            "/camera/image_raw", frame_qos(), [this](Image::ConstSharedPtr image) {
                const auto key = stamp_key(image->header);
                images_[key] = std::move(image);
                render(key);
            });
        result_subscription_ = create_subscription<Result>(
            "/armor/tracks", frame_qos(), [this](Result::ConstSharedPtr result) {
                const auto key = stamp_key(result->header);
                results_[key] = std::move(result);
                render(key);
            });
    }
};

std::shared_ptr<rclcpp::Node> make_camera(const rclcpp::NodeOptions &options) {
    return std::make_shared<HikCameraNode>(options);
}
std::shared_ptr<rclcpp::Node> make_detector(const rclcpp::NodeOptions &options) {
    return std::make_shared<ArmorDetectionNode>(options);
}
std::shared_ptr<rclcpp::Node> make_tracker(const rclcpp::NodeOptions &options) {
    return std::make_shared<ArmorTrackingNode>(options);
}
std::shared_ptr<rclcpp::Node> make_visualizer(const rclcpp::NodeOptions &options) {
    return std::make_shared<ArmorVisualizationNode>(options);
}
} // namespace rmcs::ros

RCLCPP_COMPONENTS_REGISTER_NODE(rmcs::ros::HikCameraNode)
RCLCPP_COMPONENTS_REGISTER_NODE(rmcs::ros::ArmorDetectionNode)
RCLCPP_COMPONENTS_REGISTER_NODE(rmcs::ros::ArmorTrackingNode)
RCLCPP_COMPONENTS_REGISTER_NODE(rmcs::ros::ArmorVisualizationNode)
