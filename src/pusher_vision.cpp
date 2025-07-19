#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/trigger.hpp>

class PusherVision : public rclcpp::Node {
public:
    PusherVision() : Node("pusher_vision") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/pusher/camera_raw", 10, std::bind(&PusherVision::image_callback,
            this, std::placeholders::_1));
        client_ = this->create_client<std_srvs::srv::Trigger>("pusher/push");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Server not available, waiting again...");
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
        try {
            // Convert image
            cv::Mat frame = cv_bridge::toCvShare(msg, "rgb8")->image;
            cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

            cv::Mat crop_frame = frame(cv::Range(6, 204), cv::Range(96, 230)); // crop
            cv::medianBlur(crop_frame, crop_frame, 3); // filter
            cv::Mat hsv_frame;
            cv::cvtColor(crop_frame, hsv_frame, cv::COLOR_BGR2HSV); // convert to hsv
            // Threshold the image
            cv::Mat lower_red_range;
            cv::Mat upper_red_range;
            cv::inRange(hsv_frame, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_range);
            cv::inRange(hsv_frame, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_range);
            cv::Mat red_frame;
            cv::addWeighted(lower_red_range, 1.0, upper_red_range, 1.0, 0.0, red_frame); // combine images
            cv::GaussianBlur(red_frame, red_frame, cv::Size(9, 9), 2, 2); // blur

            // Check for red
            int red_pixel = cv::countNonZero(red_frame);
            if (red_pixel > RED_TRESHOLD) {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto result = client_->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Red detected! Triggering push service...");
            }

            // Show image
            cv::imshow("Camera", red_frame);
            cv::waitKey(10);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    static const int RED_TRESHOLD = 8000;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PusherVision>());
    rclcpp::shutdown();
    return 0;
}
