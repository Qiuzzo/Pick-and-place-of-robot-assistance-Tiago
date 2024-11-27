#ifndef SERVER_H
#define SERVER_H

//*** Includes
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <group_04_a2/TiagoAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <sensor_msgs/LaserScan.h>

//*** Class definition
class Tiago{
    public:
        /// @brief Constructor for the Tiago class
        /// @param name Name of the action server
        Tiago(std::string name):
            as_(nh_, name, false),
            action_name_(name){
                // Setup all the subscribers, publishers and callbacks
                as_.registerGoalCallback(boost::bind(&Tiago::goalCB, this));
                as_.registerPreemptCallback(boost::bind(&Tiago::preemptCB, this));
                as_.start();
                sub_ = nh_.subscribe("tiago_pose", 1, &Tiago::feedCB, this);
                sub_result_ = nh_.subscribe("move_base/result", 1, &Tiago::poseCB, this);
                sub_pose_ = nh_.subscribe("robot_pose", 1, &Tiago::updateCB, this);
                sub_laser_ = nh_.subscribe("scan", 1, &Tiago::laserCB, this);
            }
        
    /// @brief Destructor for the Tiago class
        ~Tiago(void){}

    /// @brief Function that is called when the goal is received
    void goalCB();

    /// @brief Function that is called when the goal is preempted
    void preemptCB();

    /// @brief Function that is called when the node move_base/result publishes
    void poseCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

    /// @brief Function that provides additional feedback to the client
    void feedCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /// @brief Function that updates the data about the robot position
    void updateCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    /// @brief Function that updates the data about the laser scan
    void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg);

    /// @brief Function that computes the position of the objects and sets the success of the goal
    void computeObjectPosition();

    protected:
        // Variables for the action server
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<group_04_a2::TiagoAction> as_;
        std::string action_name_;
        group_04_a2::TiagoFeedback feedback_;
        group_04_a2::TiagoResult result_;
        geometry_msgs::PoseStamped goal_;
        
        // Publisher to move_base_simple/goal
        ros::Publisher pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        // Subscriber to tiago_pose
        ros::Subscriber sub_;
        // Subscriber to move_base/result
        ros::Subscriber sub_result_;
        // Subscriber to move_base/feedback
        ros::Subscriber sub_pose_;
        // Subscriber to laser_scan
        ros::Subscriber sub_laser_;
        // Variable that stores the pointer of the laser scan message
        sensor_msgs::LaserScan::ConstPtr laser_msg_; 
        // Variable to see if we reached the goal position       
        bool success_ = false;
        // Variables to store the robot position
        geometry_msgs::Pose pose_actual_;
        geometry_msgs::Pose pose_previous_;
};

#endif