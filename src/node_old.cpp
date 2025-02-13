// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <turtlesim/msg/pose.hpp>
// #include <cmath>
// #include <iostream>
// #include <vector>

// class TurtleController : public rclcpp::Node {
// public:
//   TurtleController() : Node("turtle_controller") {
//     // Create publisher and subscriber
//     vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
//     pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
//         "/turtle1/pose", 10,
//         std::bind(&TurtleController::poseCallback, this, std::placeholders::_1));

//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(100), 
//         std::bind(&TurtleController::controlLoop, this));

//     RCLCPP_INFO(this->get_logger(), "Turtle Controller Node Initialized .......................................");
//   }

// private:
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
//   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   turtlesim::msg::Pose current_pose_;
//   size_t point_index_ = 0;

//   const std::vector<std::vector<double>> POINTS = {
//       {1.0, 8.0}, {3.0, 8.0}, {3.0, 5.0}, {1.0, 5.0}, {3.0, 3.0}, {5.0, 3.0},
//       {5.0, 8.0}, {7.0, 8.0}, {7.0, 3.0}, {5.0, 3.0}, {9.0, 8.0}, {7.0, 8.0},
//       {7.0, 6.0}, {9.0, 6.0}, {9.0, 4.0}, {7.0, 4.0}};

//   void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
//     current_pose_ = *msg;
//   }

//   void controlLoop() {
//     if (point_index_ >= POINTS.size()) {
//       RCLCPP_INFO(this->get_logger(), "All points reached. Stopping.");
//       return;
//     }

//     double goal_x = POINTS[point_index_][0];
//     double goal_y = POINTS[point_index_][1];

//     geometry_msgs::msg::Twist cmd;
    
//     // Step 1: Align with goal
//     double angle_to_goal = std::atan2(goal_y - current_pose_.y, goal_x - current_pose_.x);
//     double angle_error = angle_to_goal - current_pose_.theta;

//     std::cout << "Angle to goal: " << angle_to_goal << std::endl;
//     std::cout << "Current angle: " << current_pose_.theta << std::endl;
//     std::cout << "Angle error: " << angle_error << std::endl;

//     if (std::abs(angle_error) > 0.01) {
//       cmd.angular.z = 10.0 * angle_error;
//       vel_pub_->publish(cmd);
//       return;
//     }

//     // Step 2: Move to goal
//     double dx = goal_x - current_pose_.x;
//     double dy = goal_y - current_pose_.y;
//     double distance = std::sqrt(dx * dx + dy * dy);
//     double direction = std::atan2(dy, dx);

//     std::cout << "Distance to goal: " << distance << std::endl;
//     std::cout << "Current x: " << current_pose_.x << std::endl;
//     std::cout << "Current y: " << current_pose_.y << std::endl;
//     std::cout << "Direction: " << direction << std::endl;

//     if (distance > 0.1) {
//       cmd.linear.x = 2.0 * distance * std::cos(direction - current_pose_.theta);
//       vel_pub_->publish(cmd);
//       return;
//     }

//     // Step 3: Move to next point
//     point_index_++;
//   }
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<TurtleController>());
//   rclcpp::shutdown();
//   return 0;
// }



