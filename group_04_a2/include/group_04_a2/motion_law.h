#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <group_04_a2/tiago_goal.h>

/// @brief Callback function for the laser scanner, it stores the data in laser_scan
/// @param laser_msg read the data from the laser scanner
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg);

/// @brief Check if the narrow passage is crossed
bool passageCrossed();

/// @brief Function that receives odometry data and computes the velocities to move the robot
/// @param odom_msg odometry data
void odomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& odom_msg);

/// @brief Function that moves the robot in the corridor
/// @param pos goal pose
void motion (geometry_msgs::PoseStamped pos);

/// @brief Function that rotates the robot when it can't move forward
void recovery_rotation();
