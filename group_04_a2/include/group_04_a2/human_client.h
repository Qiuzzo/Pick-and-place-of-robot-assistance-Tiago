#ifndef HUMAN_CLIENT_H
#define HUMAN_CLIENT_H

//*** Includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tiago_iaslab_simulation/Objs.h>

//*** Functions

/// @brief Get the order of the objects from human_node
/// @return std::vector<int> The order of the objects
std::vector<int> get_order();

#endif