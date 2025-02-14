#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

class SpawnTurtleClient : public rclcpp::Node {
public:
    SpawnTurtleClient() : Node("spawn_turtle_client") {
        client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        callSpawnService();
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;

    void callSpawnService() {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 1.0;
        request->y = 1.0;
        request->theta = 0.0;
        request->name = "turtle";

        auto future = client_->async_send_request(request);
        future.wait();

        if (future.get()) {
            RCLCPP_INFO(this->get_logger(), "Successfully spawned turtle :)");
        } 
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle!!!!");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpawnTurtleClient>());
    rclcpp::shutdown();
    return 0;
}
