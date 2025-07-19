#include <memory>
#include <chrono>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include <conveyor_sorter/srv/conveyor_start.hpp>
#include <conveyor_sorter/srv/spawn_prop.hpp>

using namespace std::literals::chrono_literals;

class RandomStart : public rclcpp::Node {
  const int PROP_NUMBER = 10;

  public:
    RandomStart() : Node("random_start") {
      conveyor_clt = this->create_client<conveyor_sorter::srv::ConveyorStart>("/conveyor/start");
      spawner_clt = this->create_client<conveyor_sorter::srv::SpawnProp>("/props/spawn");
    }

    void init() {
      static std::default_random_engine re;
      static std::bernoulli_distribution rand_target(.5);
      if (!wait_for_service<conveyor_sorter::srv::SpawnProp>(spawner_clt)) return;
      auto spawner_req = std::make_shared<conveyor_sorter::srv::SpawnProp::Request>();
      spawner_req->random = false;
      for (int i = 0; i < PROP_NUMBER; ++i) {
        spawner_req->target = rand_target(re);
        auto spawner_res = spawner_clt->async_send_request(spawner_req);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), spawner_res) !=
        rclcpp::FutureReturnCode::SUCCESS || !spawner_res.get()->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed while calling service /props/spawn");
            return;
        }
      }

      if (!wait_for_service<conveyor_sorter::srv::ConveyorStart>(conveyor_clt)) return;
      auto conveyor_req = std::make_shared<conveyor_sorter::srv::ConveyorStart::Request>();
      conveyor_req->speed = 2.;
      auto conveyor_res = conveyor_clt->async_send_request(conveyor_req);
      if (rclcpp::spin_until_future_complete(this->shared_from_this(), conveyor_res) !=
      rclcpp::FutureReturnCode::SUCCESS || !conveyor_res.get()->success) {
          RCLCPP_ERROR(this->get_logger(), "Failed while calling service /conveyor/start");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Random setup ready");
    }

    template<typename T>
    bool wait_for_service(typename rclcpp::Client<T>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        return true;
    }

  private:
    rclcpp::Client<conveyor_sorter::srv::ConveyorStart>::SharedPtr conveyor_clt;
    rclcpp::Client<conveyor_sorter::srv::SpawnProp>::SharedPtr spawner_clt;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomStart>();
  node->init();
  rclcpp::shutdown();

  return 0;
}
