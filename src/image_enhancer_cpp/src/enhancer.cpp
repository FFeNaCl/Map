#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

class ImageEnhancer : public rclcpp::Node
{
public:
    ImageEnhancer() : Node("image_enhancer_cpp")
    {
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/image_raw");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/image_enhanced");
        debug_topic_ = this->declare_parameter<std::string>("debug_topic", "/image_enhanced_debug");
        publish_debug_view_ = this->declare_parameter<bool>("publish_debug_view", false);
        use_red_compensation_ = this->declare_parameter<bool>("use_red_compensation", true);
        use_gray_world_ = this->declare_parameter<bool>("use_gray_world", true);
        use_clahe_ = this->declare_parameter<bool>("use_clahe", true);
        use_unsharp_mask_ = this->declare_parameter<bool>("use_unsharp_mask", false);
        use_undistort_ = this->declare_parameter<bool>("use_undistort", false);
        red_gain_limit_ = this->declare_parameter<double>("red_gain_limit", 2.5);
        clahe_clip_limit_ = this->declare_parameter<double>("clahe_clip_limit", 3.0);
        clahe_tile_size_ = this->declare_parameter<int>("clahe_tile_size", 8);
        unsharp_amount_ = this->declare_parameter<double>("unsharp_amount", 0.35);
        camera_matrix_values_ = this->declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>{});
        distortion_coeffs_values_ = this->declare_parameter<std::vector<double>>("distortion_coeffs", std::vector<double>{});

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10,
            std::bind(&ImageEnhancer::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
        if (publish_debug_view_) {
            debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(debug_topic_, 10);
        }

        RCLCPP_INFO(this->get_logger(), "水下图像预处理节点启动: %s -> %s",
                    input_topic_.c_str(), output_topic_.c_str());
    }

private:
    cv::Mat applyRedCompensation(const cv::Mat &input)
    {
        std::vector<cv::Mat> channels(3);
        cv::split(input, channels);

        const double b_mean = cv::mean(channels[0])[0];
        const double g_mean = cv::mean(channels[1])[0];
        const double r_mean = cv::mean(channels[2])[0];
        const double target_mean = std::max(b_mean, g_mean);
        const double eps = 1e-6;
        const double gain = std::clamp(target_mean / std::max(r_mean, eps), 1.0, std::max(1.0, red_gain_limit_));

        channels[2].convertTo(channels[2], -1, gain);

        cv::Mat compensated;
        cv::merge(channels, compensated);
        return compensated;
    }

    cv::Mat applyGrayWorld(const cv::Mat &input)
    {
        std::vector<cv::Mat> channels(3);
        cv::split(input, channels);

        const double b_mean = cv::mean(channels[0])[0];
        const double g_mean = cv::mean(channels[1])[0];
        const double r_mean = cv::mean(channels[2])[0];
        const double gray_mean = (b_mean + g_mean + r_mean) / 3.0;

        std::vector<cv::Mat> balanced_channels(3);
        const double eps = 1e-6;
        channels[0].convertTo(balanced_channels[0], -1, gray_mean / std::max(b_mean, eps));
        channels[1].convertTo(balanced_channels[1], -1, gray_mean / std::max(g_mean, eps));
        channels[2].convertTo(balanced_channels[2], -1, gray_mean / std::max(r_mean, eps));

        cv::Mat balanced;
        cv::merge(balanced_channels, balanced);
        return balanced;
    }

    cv::Mat applyClahe(const cv::Mat &input)
    {
        cv::Mat lab;
        cv::cvtColor(input, lab, cv::COLOR_BGR2Lab);

        std::vector<cv::Mat> channels(3);
        cv::split(lab, channels);

        const int tile_size = std::max(2, clahe_tile_size_);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clahe_clip_limit_, cv::Size(tile_size, tile_size));
        clahe->apply(channels[0], channels[0]);

        cv::merge(channels, lab);

        cv::Mat enhanced;
        cv::cvtColor(lab, enhanced, cv::COLOR_Lab2BGR);
        return enhanced;
    }

    cv::Mat applyUnsharpMask(const cv::Mat &input)
    {
        cv::Mat blurred;
        cv::GaussianBlur(input, blurred, cv::Size(0, 0), 1.2);

        cv::Mat sharpened;
        const double amount = std::clamp(unsharp_amount_, 0.0, 1.5);
        cv::addWeighted(input, 1.0 + amount, blurred, -amount, 0.0, sharpened);
        return sharpened;
    }

    cv::Mat applyUndistort(const cv::Mat &input)
    {
        if (!use_undistort_) {
            return input;
        }

        if (camera_matrix_values_.size() != 9 || distortion_coeffs_values_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "已请求去畸变，但 camera_matrix 或 distortion_coeffs 参数不完整");
            return input;
        }

        cv::Mat camera_matrix(3, 3, CV_64F, camera_matrix_values_.data());
        cv::Mat distortion_coeffs(1, static_cast<int>(distortion_coeffs_values_.size()), CV_64F,
                                  distortion_coeffs_values_.data());

        cv::Mat undistorted;
        cv::undistort(input, undistorted, camera_matrix.clone(), distortion_coeffs.clone());
        return undistorted;
    }

    cv::Mat makeDebugView(const cv::Mat &input, const cv::Mat &enhanced)
    {
        cv::Mat left = input.clone();
        cv::Mat right = enhanced.clone();
        cv::putText(left, "RAW / CAMERA", cv::Point(18, 38), cv::FONT_HERSHEY_SIMPLEX,
                    1.0, cv::Scalar(30, 220, 255), 2, cv::LINE_AA);
        cv::putText(right, "ENHANCED: red + WB + CLAHE", cv::Point(18, 38), cv::FONT_HERSHEY_SIMPLEX,
                    1.0, cv::Scalar(30, 220, 255), 2, cv::LINE_AA);

        cv::Mat debug;
        cv::hconcat(left, right, debug);
        return debug;
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat input = cv_ptr->image;

            if (input.empty()) {
                RCLCPP_WARN(this->get_logger(), "收到空图像");
                return;
            }

            cv::Mat enhanced = applyUndistort(input);

            if (use_red_compensation_) {
                enhanced = applyRedCompensation(enhanced);
            }

            if (use_gray_world_) {
                enhanced = applyGrayWorld(enhanced);
            }

            if (use_clahe_) {
                enhanced = applyClahe(enhanced);
            }

            if (use_unsharp_mask_) {
                enhanced = applyUnsharpMask(enhanced);
            }

            // 发布增强图像
            auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, enhanced).toImageMsg();
            publisher_->publish(*out_msg);

            if (publish_debug_view_ && debug_publisher_) {
                cv::Mat debug = makeDebugView(input, enhanced);
                auto debug_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, debug).toImageMsg();
                debug_publisher_->publish(*debug_msg);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "图像处理失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
    std::string input_topic_;
    std::string output_topic_;
    std::string debug_topic_;
    bool publish_debug_view_;
    bool use_red_compensation_;
    bool use_gray_world_;
    bool use_clahe_;
    bool use_unsharp_mask_;
    bool use_undistort_;
    double red_gain_limit_;
    double clahe_clip_limit_;
    int clahe_tile_size_;
    double unsharp_amount_;
    std::vector<double> camera_matrix_values_;
    std::vector<double> distortion_coeffs_values_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageEnhancer>());
    rclcpp::shutdown();
    return 0;
}
