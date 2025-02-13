#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <iostream>
#include <vector>

class TurtleController : public rclcpp::Node {
public:
  TurtleController() : Node("turtle_controller") {
    // Create publisher and subscriber
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle/cmd_vel", 10);
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle/pose", 10,
        std::bind(&TurtleController::poseCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), 
        std::bind(&TurtleController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Turtle Controller Node Initialized .......................................");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  turtlesim::msg::Pose current_pose_;
  size_t point_index_ = 0;

  const std::vector<std::vector<double>> POINTS = {
      {1.0, 8.0}, {3.0, 8.0}, {3.0, 5.0}, {1.0, 5.0}, {3.0, 3.0}, {5.0, 3.0},
      {5.0, 8.0}, {7.0, 8.0}, {7.0, 3.0}, {5.0, 3.0}, {9.0, 8.0}, {7.0, 8.0},
      {7.0, 6.0}, {9.0, 6.0}, {9.0, 4.0}, {7.0, 4.0}};

  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
  }

  void controlLoop() {
    if (point_index_ >= POINTS.size()) {
      RCLCPP_INFO(this->get_logger(), "All points reached. Stopping.");
      return;
    }

    double goal_x = POINTS[point_index_][0];
    double goal_y = POINTS[point_index_][1];
    double goal_theta = current_pose_.theta; //POINTS[point_index_][2]; // DISCUSS THIS

    geometry_msgs::msg::Twist cmd;
    

    // Step 1: Align with goal
    double angle_to_goal = std::atan2(goal_y - current_pose_.y, goal_x - current_pose_.x);
    double angle_error = angle_to_goal - current_pose_.theta;

    RCLCPP_INFO(this->get_logger(), "STEP 1 Angle to goal: %.2f, Current angle: %.2f, Angle error: %.2f", 
            angle_to_goal, current_pose_.theta, angle_error);

    if (std::abs(angle_error) > 0.01) {
      cmd.angular.z = 10.0 * angle_error;
      vel_pub_->publish(cmd);
      return;
    }

    // Step 2: Move to goal
    double dx = goal_x - current_pose_.x;
    double dy = goal_y - current_pose_.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double direction = std::atan2(dy, dx);

    RCLCPP_INFO(this->get_logger(), "STEP 2 Distance to goal: %.2f, Current x: %.2f, Current y: %.2f", distance, current_pose_.x, current_pose_.y);

    if (distance > 0.1) {
      cmd.linear.x = 2.0 * distance * std::cos(direction - current_pose_.theta);
      vel_pub_->publish(cmd);
      return;
    }

    // Step 3: Align with the goal orientation
    double theta_error = goal_theta - current_pose_.theta;
    RCLCPP_INFO(this->get_logger(), "[Step 3] Aligning orientation. Theta error: %.2f", theta_error);

    if (theta_error > M_PI)
        theta_error -= 2 * M_PI;
    if (theta_error < -M_PI)
      theta_error += 2 * M_PI;

    if (std::abs(theta_error) > 0.1) {
      cmd.angular.z = 2.0 * theta_error;

      RCLCPP_INFO(this->get_logger(), "[Step 3] Publishing angular.z: %.2f", cmd.angular.z);
      vel_pub_->publish(cmd);
      return;
    }
        
    point_index_++;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleController>());
  rclcpp::shutdown();
  return 0;
}



