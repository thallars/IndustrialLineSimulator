#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class PusherVision : public rclcpp::Node {
public:
    PusherVision() : Node("pusher_vision") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/pusher/camera_raw", 10, std::bind(&PusherVision::image_callback,
            this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        try {
            // Convert image
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::Mat crop = frame(cv::Range(6, 204), cv::Range(96, 230)); // crop
            cv::Mat hsv_image;
            cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV); // convert to hsv
            // Threshold the image
            cv::Mat lower_red_range;
            cv::Mat upper_red_range;
            cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
            cv::Mat red_image;
            cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image); // combine images
            cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);


            // Show image
            cv::imshow("Camera", frame);
            cv::waitKey(10);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PusherVision>());
    rclcpp::shutdown();
    return 0;
}