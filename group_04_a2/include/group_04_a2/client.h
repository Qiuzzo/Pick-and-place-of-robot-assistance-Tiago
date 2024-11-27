#ifndef CLIENT_H
#define CLIENT_H

//*** Includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <group_04_a2/TiagoAction.h>
#include <group_04_a2/human_client.h>
#include <group_04_a2/camera.h>
#include <group_04_a2/arm.h>

//*** Movement functions declaration

/// @brief Function that is called when the goal completes
/// @param state  State of the goal 
/// @param result Result of the laser reading
void doneCb(const actionlib::SimpleClientGoalState& state,
            const group_04_a2::TiagoResultConstPtr& result);

/// @brief Function that is called when the goal becomes active
void activeCb();

/// @brief Function that is called when feedback is received
/// @param feedback pointer to the feedback section of the action
void feedbackCb(const group_04_a2::TiagoFeedbackConstPtr& feedback);

/// @brief Function that creates a goal message, if the goal is not valid it ends with an error message
/// @param x x coordinate of the goal
/// @param y y coordinate of the goal
/// @param z z coordinate of the goal
/// @param ox first quaternion of the goal
/// @param oy second quaternion of the goal
/// @param oz third quaternion of the goal
/// @param ow fourth quaternion of the goal
/// @param linear_init if true the robot will move in the corridor
group_04_a2::TiagoGoal createGoal(double px, double py, double pz, double ox, double oy, double oz, double ow, bool linear_init=false);

/// @brief Function that moves the robot to a goal
/// @param pose_1 goal pose
/// @param ac action client for the movement
/// @param corridor if true the robot will move in the corridor using Motion Law
/// @return Pointer to the result of the movement
group_04_a2::TiagoResultConstPtr move_to(geometry_msgs::PoseStamped pose_1, actionlib::SimpleActionClient<group_04_a2::TiagoAction> &ac, bool corridor=false);

/// @brief Returns a map with waypoints to move around the table
/// @return std::map<int, geometry_msgs::PoseStamped>, the key is the number of the waypoint
std::map<int, geometry_msgs::PoseStamped> getPositionMap();

/// @brief Returns the path to reach the barrel
/// @param barrel_wrt_robot pose of the barrel wrt the robot
/// @param wp current pose of the robot
/// @param direction retrieved from the camera, 0 is left, 1 is straight, 2 is right
std::vector<geometry_msgs::PoseStamped> computeBarrelPose(geometry_msgs::PoseStamped barrel_wrt_robot, geometry_msgs::PoseStamped wp, int direction);

//*** Camera functions declaration

/// @brief Contact the camera server and return the result of the detection
/// @param color_recognition if true the camera will detect the color of the objects otherwise will use the AprilTags
/// @return Pointer to the result of CameraAction
group_04_a2::CameraResultConstPtr cameraDetection(bool color_recognition=false);

/// @brief Function that is called when the goal of the camera completes
/// @param state 
/// @param result Result of the camera action
void doneCbCamera(const actionlib::SimpleClientGoalState &state, const group_04_a2::CameraResultConstPtr &result);

/// @brief Function that is called when the goal of the camera becomes active
void activeCbCamera();

/// @brief Function that is called when feedback of the camera is received
/// @param feedback pointer to the feedback section of the action
void feedbackCbCamera(const group_04_a2::CameraFeedbackConstPtr &feedback);

//*** Arm functions declaration

/// @brief Contact the arm server and return the result
/// @param objects vector of poses of the objects
/// @param ids vector of ids of the objects
/// @param pick if true the arm will pick the objects otherwise will place them
 group_04_a2::ArmResultConstPtr pick_place(std::vector<geometry_msgs::Pose> objects, std::vector<int> ids, bool pick);

/// @brief Contact the arm server and return the result
/// @param objects vector of poses of the objects
/// @param ids vector of ids of the objects
/// @param pick if true the arm will pick the objects otherwise will place them
void arm_doneCb(const actionlib::SimpleClientGoalState& state, const group_04_a2::ArmResultConstPtr& result);

/// @brief Function that is called when the goal of the arm becomes active
void arm_activeCb();

/// @brief Function that is called when feedback of the arm is received
/// @param feedback pointer to the feedback section of the action
void arm_feedbackCb(const group_04_a2::ArmFeedbackConstPtr& feedback);

#endif