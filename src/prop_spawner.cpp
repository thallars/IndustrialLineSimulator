#include <memory>
#include <chrono>
#include <functional>
#include <random>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>

#include <conveyor_sorter/srv/spawn_prop.hpp>

using namespace std::placeholders;
using namespace std::literals::chrono_literals;

class PropSpawner : public rclcpp::Node {
  const double INIT_MARGIN = 3.;
  const double PROP_MARGIN = 2.;

  public:
    PropSpawner() : Node("prop_spawner") {
      spawn_clt = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/default/create");

      spawn_prop_srv = this->create_service<conveyor_sorter::srv::SpawnProp>(
        "props/spawn",
        std::bind(&PropSpawner::spawn_prop_cb, this, _1, _2, _3)
      );
    }

    template<typename T>
    bool wait_for_service(typename rclcpp::Client<T>::SharedPtr client) {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }
        return true;
    }

  private:
    int prop_counter = 0;
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_clt;
    rclcpp::Service<conveyor_sorter::srv::SpawnProp>::SharedPtr spawn_prop_srv;

    void spawn_prop_cb(
      rclcpp::Service<conveyor_sorter::srv::SpawnProp>::SharedPtr service,
      const std::shared_ptr<rmw_request_id_t> request_header,
      const conveyor_sorter::srv::SpawnProp::Request::SharedPtr request
    ) {
      if (!wait_for_service<ros_gz_interfaces::srv::SpawnEntity>(spawn_clt)) return;
      std::string package_share_directory = ament_index_cpp::get_package_share_directory("conveyor_sorter");

      static std::default_random_engine re;
      static std::uniform_real_distribution<double> rand_angle(0., 2 * M_PI);

      auto spawn_req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
      auto *ef = &spawn_req->entity_factory;
      ef->name = std::string("prop") + std::to_string(prop_counter + 1);
      ef->pose.position.x = - INIT_MARGIN - PROP_MARGIN * prop_counter++;
      ef->pose.position.z = 2.;
      ef->pose.orientation.z = rand_angle(re);
      ef->relative_to = "belt";

      if (request->random) {
        const std::string vegs[] = { "bad_veg.sdf", "good_veg.sdf", "normal_veg.sdf" };
        static std::uniform_int_distribution<int> rand_veg(0, 2);
        ef->sdf_filename = package_share_directory + "/models/vegetables/" + vegs[rand_veg(re)];
      } else {
        const std::string good_vegs[] = { "good_veg.sdf", "normal_veg.sdf" };
        static std::uniform_int_distribution<int> rand_good_veg(0, 1);
        if (request->target) {
          ef->sdf_filename = package_share_directory + "/models/vegetables/bad_veg.sdf";
        } else {
          ef->sdf_filename = package_share_directory + "/models/vegetables/" + good_vegs[rand_good_veg(re)];
        }
      }

      auto async_cb = [service, request_header, request](
          rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future
      ) {
        (void)request;
        conveyor_sorter::srv::SpawnProp::Response response;
        auto spawn_answer = future.get();
        if (spawn_answer->success) {
          response.success = true;
          response.message = std::string(request->target ? "target " : "regular ") + "prop has spawned";
        } else {
          response.success = false;
          response.message = "failed to spawn prop";
        }
        service->send_response(*request_header, response);
      };
      spawn_clt->async_send_request(spawn_req, async_cb);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PropSpawner>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
