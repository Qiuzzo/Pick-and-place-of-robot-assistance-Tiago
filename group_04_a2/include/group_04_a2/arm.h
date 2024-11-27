#ifndef ARM_H
#define ARM_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <group_04_a2/ArmAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_srvs/Empty.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>

class Arm
{
public:
    /// @brief Constructor for the action server
    /// @param name name of the node
    Arm(std::string name):
    as_(nh_, name, boost::bind(&Arm::goalCB, this, _1), false),
    action_name_(name)
    {
        as_.start();
    }

    ~Arm(){}

    /// @brief Callback function for the action server
    /// @param goal contains the ids of the objects on the table and a bool to pick or place
    void goalCB(const group_04_a2::ArmGoalConstPtr &goal);

    /// @brief Start the pick sequence
    /// @param goal contains the ids of the objects on the table
    void pickObject(const group_04_a2::ArmGoalConstPtr &goal);

    /// @brief Start the place sequence
    /// @param goal contains the ids of the objects on the table
    void placeObject(const group_04_a2::ArmGoalConstPtr &goal);


protected:
    /// @brief Node handle
    ros::NodeHandle nh_;
    /// @brief Action server
    actionlib::SimpleActionServer<group_04_a2::ArmAction> as_;
    /// @brief Name of the action
    std::string action_name_;
    /// @brief Feedback and result of the action
    group_04_a2::ArmFeedback feedback_;
    /// @brief Result of the action
    group_04_a2::ArmResult result_;

private:
    /// @brief Move group interface, used to move the arm
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    /// @brief Client to attach the object to the gripper
    ros::ServiceClient attachClient_;
    /// @brief Client to detach the object from the gripper
    ros::ServiceClient detachClient_;
    
    /// @brief Add the objects to the collision objects, if pick is false the added objects will be the place tables,
    ///         if adjust is true the place tables will be enlarged
    /// @param objects vector of poses of the objects
    /// @param ids  vector of ids of the objects
    /// @param pick, true if pick, false if place
    /// @param adjust, true if we need to adjust the object
    /// @return vector of names of the added objects
    std::vector<std::string> addCollisionObjects(std::vector<geometry_msgs::Pose>& objects, std::vector<int>& ids, bool pick, bool adjust=false);

    /// @brief Subroutine to pick the object using intermediate poses
    /// @param object, pose of the object to pick
    /// @param id, id of the object to pick
    /// @return path of poses to go up from the object
    std::vector<geometry_msgs::Pose> pickObj(const geometry_msgs::Pose& object, int id);

    /// @brief Subroutine to place the object using intermediate poses
    /// @param object, pose of the object to place
    /// @param id, id of the object to place
    /// @return path of poses to go up from the object
    std::vector<geometry_msgs::Pose> placeObj(const geometry_msgs::Pose& object, int id);

    /// @brief Plans and executes the path given in input
    /// @param path, path of the motions the robot arm has to do to reach the target position
    void moveArmPath(const std::vector<geometry_msgs::Pose>& path);

    /// @brief Return the dimensions of each collision object, based on the id
    /// @param id of the object to return its dimensions
    /// @return vector of dimensions of the collision object
    std::vector<double> returnDimesions(int id);

    /// @brief Open or close the gripper
    /// @param open, true if open, false if close
    /// @param id of the object to attach or detach
    void gripper(bool open, int id);

    /// @brief Attach the object to the gripper using gazebo_ros_link_attacher
    /// @param id of the object
    void attachObjectToGripper(int id);

    /// @brief Detach the object from the gripper using gazebo_ros_link_attacher
    /// @param id of the object
    void detachObjectFromGripper(int id);

    /// @brief Move the arm to a safe pose and tuck it if requested
    /// @param tuck, true if tuck, false if safe pose
    void safePose(bool tuck);
};

#endif // ARM_H