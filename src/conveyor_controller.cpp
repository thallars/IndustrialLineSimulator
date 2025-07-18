#include <memory>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::placeholders;

class ConveyorController : public rclcpp::Node {
  public:
    ConveyorController() : Node("conveyor_controller") {
      start_srv = this->create_service<std_srvs::srv::Trigger>(
        "conveyor/start",
        std::bind(&ConveyorController::start_cb, this, _1, _2)
      );
      stop_srv = this->create_service<std_srvs::srv::Trigger>(
        "conveyor/stop",
        std::bind(&ConveyorController::stop_cb, this, _1, _2)
      );
      conveyor_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "conveyor/state",
        rclcpp::QoS(10),
        std::bind(&ConveyorController::conveyor_state_cb, this, _1)
      );
      conveyor_vel_pub = this->create_publisher<std_msgs::msg::Float64>(
        "conveyor/velocity",
        rclcpp::QoS(10)
      );
    }

  private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr conveyor_state_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr conveyor_vel_pub;

    void start_cb(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response
    ) {
      std_msgs::msg::Float64 start_vel;
      start_vel.data = 1.;
      conveyor_vel_pub->publish(start_vel);
      response->success = true;
      response->message = "conveyor started";
    }

    void stop_cb(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response
    ) {
      std_msgs::msg::Float64 stop_vel;
      stop_vel.data = 0.;
      conveyor_vel_pub->publish(stop_vel);
      response->success = true;
      response->message = "conveyor stopped";
    }

    void conveyor_state_cb(
        sensor_msgs::msg::JointState::SharedPtr msg
    ) {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConveyorController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
