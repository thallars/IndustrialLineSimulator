#include <memory>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::placeholders;
using namespace std::literals::chrono_literals;

class PusherController : public rclcpp::Node {
  public:
    PusherController() : Node("pusher_controller") {
      push_srv = this->create_service<std_srvs::srv::Trigger>(
        "pusher/push",
        std::bind(&PusherController::push_cb, this, _1, _2)
      );
      pusher_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
          "pusher/state",
          rclcpp::QoS(10),
          std::bind(&PusherController::pusher_state_cb, this, _1)
      );
      pusher_pos_pub = this->create_publisher<std_msgs::msg::Float64>(
        "pusher/position",
        rclcpp::QoS(10)
      );
    }

  private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr push_srv;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pusher_state_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pusher_pos_pub;

    void push_cb(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response
    ) {
      std_msgs::msg::Float64 end_pos;
      end_pos.data = 3.;
      pusher_pos_pub->publish(end_pos);
      rclcpp::sleep_for(1s);
      std_msgs::msg::Float64 start_pos;
      start_pos.data = 0.;
      pusher_pos_pub->publish(start_pos);
      response->success = true;
      response->message = "арбуз revenge";
    }

    void pusher_state_cb(
        sensor_msgs::msg::JointState::SharedPtr msg
    ) {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PusherController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
