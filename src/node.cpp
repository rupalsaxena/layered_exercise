#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <vector>
#include <iostream>

// Waypoints
const std::vector<std::vector<double>> POINTS = {
    {1.0, 8.0}, {3.0, 8.0}, {3.0, 5.0}, {1.0, 5.0}, {3.0, 3.0}, {5.0, 3.0}, {5.0, 8.0}, {7.0, 8.0},
    {7.0, 3.0}, {5.0, 3.0}, {9.0, 8.0}, {7.0, 8.0}, {7.0, 6.0}, {9.0, 6.0}, {9.0, 4.0}, {7.0, 4.0}};

class TurtleController : public rclcpp::Node {
public:
  TurtleController() : Node("turtle_controller") {
    // Subscriber and Publisher
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle/pose", 10, std::bind(&TurtleController::poseCallback, this, std::placeholders::_1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle/cmd_vel", 10);

    // Start the control loop
    controlLoop();
  }

private:
  // Member variables
  turtlesim::msg::Pose current_pose_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
  }

  void controlLoop() {
    rclcpp::Rate rate(10); // 10 Hz loop rate

    for (const auto& point : POINTS) {
        double goal_x = point[0];
        double goal_y = point[1];
        double goal_theta = (point.size() > 2) ? point[2] : current_pose_.theta; // Optional goal orientation

        // Step 1: Align with the goal
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            geometry_msgs::msg::Twist cmd;

            std::cout << "current pose x " << current_pose_.x << "\n"  ;
            std::cout << "current pose y " << current_pose_.y << "\n"  ;
            std::cout << "current pose z " << current_pose_.theta << "\n"  ;

            double angle_to_goal = std::atan2(goal_y - current_pose_.y, goal_x - current_pose_.x);
            double angle_error = angle_to_goal - current_pose_.theta;

            // Normalize angle error to range [-pi, pi]
            angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

            std::cout << "[Step 1] Aligning to goal. Angle error: " << angle_error << "\n";

            if (std::abs(angle_error) < 0.01) {
                std::cout << "[Step 1] Alignment complete.\n";
                break;
            }

            cmd.angular.z = 10.0 * angle_error;
            std::cout << "[Step 1] Publishing angular.z: " << cmd.angular.z << "\n";
            vel_pub_->publish(cmd);
            rate.sleep();
        }

        // Step 2: Move to the goal
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            geometry_msgs::msg::Twist cmd;
            double dx = goal_x - current_pose_.x;
            double dy = goal_y - current_pose_.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            std::cout << "[Step 2] Moving to goal. Distance to goal: " << distance << "\n";

            if (distance < 0.1) {
                std::cout << "[Step 2] Reached goal position.\n";
                break;
            }

            cmd.linear.x = std::min(2.0 * distance, 1.0); // Limit linear speed to 1.0
            std::cout << "[Step 2] Publishing linear.x: " << cmd.linear.x << "\n";
            vel_pub_->publish(cmd);
            rate.sleep();
        }

        // Step 3: Align with the goal orientation
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            geometry_msgs::msg::Twist cmd;
            double theta_error = goal_theta - current_pose_.theta;

            // Normalize theta_error to range [-pi, pi]
            theta_error = std::atan2(std::sin(theta_error), std::cos(theta_error));

            std::cout << "[Step 3] Aligning orientation. Theta error: " << theta_error << "\n";

            if (std::abs(theta_error) < 0.1) {
                std::cout << "[Step 3] Orientation alignment complete.\n";
                break;
            }

            cmd.angular.z = 2.0 * theta_error; // Slower angular velocity for precision
            std::cout << "[Step 3] Publishing angular.z: " << cmd.angular.z << "\n";
            vel_pub_->publish(cmd);
            rate.sleep();
        }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleController>());
  rclcpp::shutdown();
  return 0;
}