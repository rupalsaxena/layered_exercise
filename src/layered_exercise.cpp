#include <cmath>
#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>


// Constants
constexpr double ANGLE_TOLERANCE = 0.01;  // Tolerance for angle alignment
constexpr double ANGULAR_GAIN = 10.0;  // Angular speed gain
constexpr double DISTANCE_TOLERANCE = 0.1;  // Tolerance for reaching goal
constexpr double LINEAR_GAIN = 2.0;  // Linear speed gain
constexpr double THETA_TOLERANCE = 0.1;  // Tolerance for reaching goal
constexpr double THETA_GAIN = 2.0;  // Linear speed gain

const std::vector<std::vector<double>> POINTS = {
  {1.0, 8.0}, {3.0, 8.0}, {3.0, 5.0}, {1.0, 5.0}, {3.0, 3.0}, {5.0, 3.0}, {5.0, 8.0}, {7.0, 8.0},
  {7.0, 3.0}, {5.0, 3.0}, {9.0, 8.0}, {7.0, 8.0}, {7.0, 6.0}, {9.0, 6.0}, {9.0, 4.0}, {7.0, 4.0}
}; // points for turtle to follow


class TurtleController : public rclcpp::Node {
public:
  TurtleController() : Node("turtle_controller") {
    // Create publisher and subscriber
    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle/cmd_vel", 10);
    pose_sub = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle/pose", 10,
        std::bind(&TurtleController::poseCallback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&TurtleController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Turtle Controller Node Initialized");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
  rclcpp::TimerBase::SharedPtr timer;

  turtlesim::msg::Pose current_pose;
  size_t point_index = 0;

  // Callback to receive ROS2 msgs
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_pose = *msg;
  }

  void controlLoop() {
    if (point_index >= POINTS.size()) {
      RCLCPP_INFO(this->get_logger(), "All points reached. Stopping.");
      rclcpp::shutdown();
      return;
    }

    double goal_x = POINTS[point_index][0];
    double goal_y = POINTS[point_index][1];
    double goal_theta = current_pose.theta; // DISCUSS THIS

    geometry_msgs::msg::Twist cmd;

    // Step 1: Align with goal
    double angle_to_goal = std::atan2(goal_y - current_pose.y, goal_x - current_pose.x);
    double angle_error = angle_to_goal - current_pose.theta;

    RCLCPP_INFO(this->get_logger(), "Angle to goal: %.2f, Current angle: %.2f, Angle error: %.2f", angle_to_goal, 
    goal_theta, angle_error);

    if (std::abs(angle_error) > ANGLE_TOLERANCE) {
      cmd.angular.z = ANGULAR_GAIN * angle_error;
      vel_pub->publish(cmd);
      return;
    }

    // Step 2: Move to goal
    double dx = goal_x - current_pose.x;
    double dy = goal_y - current_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double direction = std::atan2(dy, dx);

    RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f, Current x: %.2f, Current y: %.2f, Direction: %.2f", 
    distance, current_pose.x, current_pose.y, direction);

    if (distance > DISTANCE_TOLERANCE) {
      cmd.linear.x = LINEAR_GAIN * distance * std::cos(direction - current_pose.theta);
      vel_pub->publish(cmd);
      return;
    }

    // Step 3: Align with the goal orientation
    double theta_error = goal_theta - current_pose.theta;
    RCLCPP_INFO(this->get_logger(), "Theta error: %.2f", theta_error);

    if (theta_error > M_PI)
        theta_error -= 2 * M_PI;
    if (theta_error < -M_PI)
      theta_error += 2 * M_PI;

    if (std::abs(theta_error) > THETA_TOLERANCE) {
      cmd.angular.z = THETA_GAIN * theta_error;

      RCLCPP_INFO(this->get_logger(), "Publishing angular.z: %.2f", cmd.angular.z);
      vel_pub->publish(cmd);
      return;
    }
        
    point_index++;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleController>());
  rclcpp::shutdown();
  return 0;
}
