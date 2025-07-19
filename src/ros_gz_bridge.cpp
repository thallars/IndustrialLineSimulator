#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto ros_gz_bridge = std::make_shared<ros_gz_bridge::RosGzBridge>();
  ros_gz_bridge->add_service_bridge(
      "ros_gz_interfaces/srv/SpawnEntity",
      "gz.msgs.EntityFactory",
      "gz.msgs.Boolean",
      "/world/default/create"
  );

  rclcpp::spin(ros_gz_bridge);
  rclcpp::shutdown();

  return 0;
}
