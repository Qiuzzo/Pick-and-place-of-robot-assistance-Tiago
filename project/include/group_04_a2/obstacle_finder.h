#ifndef OBSTACLE_FINDER_H
#define OBSTACLE_FINDER_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * @brief Function that finds the obstacles in the laser scan
 * 
 * @param scan_msg laser scan message
 * @return vector of poses of the obstacles
 */
std::vector<geometry_msgs::PoseStamped> obstacle_finder_function(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

#endif