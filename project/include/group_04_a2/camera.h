#ifndef CAMERA_H
#define CAMERA_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <group_04_a2/CameraAction.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Create a service client for the camera, it will use apriltags to detect the objects and return their position
class Camera
{
public:
    /// @brief Constructor for the camera
    /// @param name Name of the server
    Camera(std::string name):
    as_(nh_, name, boost::bind(&Camera::goalCB, this, _1), false),
    action_name_(name),
    tf_buffer_(),
    tf_listener_(tf_buffer_)
  {
    // Subscribe to tag_detections topic
    sub_det_ = nh_.subscribe("/tag_detections", 1, &Camera::tagDetectionsCB, this);
    sub_camera_ = nh_.subscribe("/xtion/rgb/image_raw", 1, &Camera::imageCallback, this);

    // Allow the buffer to use a dedicated thread
    tf_buffer_.setUsingDedicatedThread(true);
    as_.start();
  }
    
    /// @brief Destructor for the camera
    ~Camera(void){}

    /// @brief Function that is called when the goal is received
    /// @param goal Goal containing a bool to decide if we need detection or color recognition
    void goalCB(const group_04_a2::CameraGoalConstPtr &goal);

    /// @brief Function that is called when color recognition is required
    /// @param goal Goal containing the order of the objects
    std::vector<int> colorGoalCB(const group_04_a2::CameraGoalConstPtr &goal);

    /// @brief Function that is called when the camera topic is received, it stores the image as a cv::Mat
    /// @param msg Message containing the camera image
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /// @brief Function that is called when the tag_detections topic is received, it stores the detections
    ///        after transforming them in the base frame in a map  
    /// @param msg Message containing the tag detections
    void tagDetectionsCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

    /// @brief Function that sets the head position using the trajectory action server
    /// @param pan Pan position
    /// @param tilt Tilt position
    void setHeadPosition(double pan, double tilt);


protected:
    // Node handle
    ros::NodeHandle nh_;
    // Variables for the action server
    actionlib::SimpleActionServer<group_04_a2::CameraAction> as_;
    std::string action_name_;
    group_04_a2::CameraFeedback feedback_;
    group_04_a2::CameraResult result_;

    //Subscriber to tag_detections topic
    ros::Subscriber sub_det_;
    //Subscriber to tf topic
    ros::Subscriber sub_tf_;
    //Subscriber to camera topic
    ros::Subscriber sub_camera_;

    //Variables to store the tag detections, a map with the id and the pose
    std::map<int, std::vector<geometry_msgs::PoseStamped>> tag_detections_;
    bool accumulate = false;

    //Variables to store the camera image 
    cv::Mat image_;
    
    //Transform listener
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;

};

#endif // CAMERA_H