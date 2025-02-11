#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <cmath>
#include <iostream>
#include <vector>
const std::vector<std::vector<double>> POINTS = {
    {1.0, 8.0}, {3.0, 8.0}, {3.0, 5.0}, {1.0, 5.0}, {3.0, 3.0}, {5.0, 3.0}, {5.0, 8.0}, {7.0, 8.0},
    {7.0, 3.0}, {5.0, 3.0}, {9.0, 8.0}, {7.0, 8.0}, {7.0, 6.0}, {9.0, 6.0}, {9.0, 4.0}, {7.0, 4.0}};
turtlesim::Pose current_pose;

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
  current_pose = *msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "node");
  ros::NodeHandle nh("~");
  ros::Subscriber pose_sub = nh.subscribe("/turtle/pose", 10, poseCallback);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle/cmd_vel", 10);
  ros::Rate rate(10);

  for (const auto& point : POINTS) {
    double goal_x = point[0];
    double goal_y = point[1];
    double goal_theta = point[2];

    // Step 1: Align with goal
    while (ros::ok()) {
      ros::spinOnce();
      geometry_msgs::Twist cmd;
      double angle_to_goal = std::atan2(goal_y - current_pose.y, goal_x - current_pose.x);
      double angle_error = angle_to_goal - current_pose.theta;

      std::cout << "Angle to goal: " << angle_to_goal << std::endl;
      std::cout << "Current angle: " << current_pose.theta << std::endl;
      std::cout << "Angle error: " << angle_error << std::endl;

      if (std::abs(angle_error) < 0.01)
        break;
      cmd.angular.z = 10.0 * angle_error;
      vel_pub.publish(cmd);
      rate.sleep();
    }

    // Step 2: Move to goal
    while (ros::ok()) {
      ros::spinOnce();
      geometry_msgs::Twist cmd;
      double dx = goal_x - current_pose.x;
      double dy = goal_y - current_pose.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      double direction = std::atan2(dy, dx);

      std::cout << "Distance to goal: " << distance << std::endl;
      std::cout << "Current x: " << current_pose.x << std::endl;
      std::cout << "Current y: " << current_pose.y << std::endl;
      std::cout << "Direction: " << direction << std::endl;

      if (distance < 0.1)
        break;
      cmd.linear.x = 2.0 * distance * std::cos(direction - current_pose.theta);
      vel_pub.publish(cmd);
      rate.sleep();
    }

    // Step 3: Align with goal orientation
    while (ros::ok()) {
      ros::spinOnce();
      geometry_msgs::Twist cmd;
      double theta_error = goal_theta - current_pose.theta;

      if (theta_error > M_PI)
        theta_error -= 2 * M_PI;
      if (theta_error < -M_PI)
        theta_error += 2 * M_PI;

      if (std::abs(theta_error) < 0.1)
        break;
      cmd.angular.z = 2.0 * theta_error;
      vel_pub.publish(cmd);
      rate.sleep();
    }
  }

  return 0;
}