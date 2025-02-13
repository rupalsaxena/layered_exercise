#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"

class KillTurtleClient : public rclcpp::Node {
public:
    KillTurtleClient() : Node("kill_turtle_client") {
        client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        call_kill_service("turtle1");
    }

private:
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_;

    void call_kill_service(const std::string &turtle_name) {
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name;

        auto future = client_->async_send_request(request);
        future.wait();

        if (future.get()) {
            RCLCPP_INFO(this->get_logger(), "Successfully killed turtle: %s", turtle_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to kill turtle");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KillTurtleClient>());
    rclcpp::shutdown();
    return 0;
}
