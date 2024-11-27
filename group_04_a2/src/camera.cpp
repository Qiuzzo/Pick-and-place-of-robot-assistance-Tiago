#include <group_04_a2/camera.h>

//*** Shrink the namespace
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

//*** Function implementation

/// @brief Function that is called when the goal is received
/// @param goal Goal containing a bool to decide if we need detection or color recognition
void Camera::goalCB(const group_04_a2::CameraGoalConstPtr &goal){
    // Clear the result
    result_.poses.clear();
    result_.ids.clear();

    // If color recognition is required, call the colorGoalCB function
    if(goal->color_recognition){
        setHeadPosition(0.15, -0.52);
        ros::Time start = ros::Time::now();
        while(ros::Time::now() - start < ros::Duration(2.0)){
            ros::spinOnce();
        }
        result_.ids = colorGoalCB(goal);
        result_.poses = std::vector<geometry_msgs::Pose>(0);
        as_.setSucceeded(result_);
        return;
    }

    // Otherwise lower the head and start accumulating the detections
    ROS_INFO("Moving the head to recognize the objects");
    setHeadPosition(0.0, -0.57);
    accumulate = true;
    // Sleep for a little to store the detections
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1.5)){
        ros::spinOnce();
    }
    accumulate = false;
    setHeadPosition(0.1, -0.50);
    accumulate = true;
    // Move to the right
    start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1.5)){
        ros::spinOnce();
    }
    accumulate = false;

    setHeadPosition(-0.1, -0.50);
    accumulate = true;
    // Move to the left
    start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1.5)){
        ros::spinOnce();
    }
    accumulate = false;

    // Create a vector of poses and a vector of ids
    std::vector<geometry_msgs::Pose> poses;
    std::vector<int> index;
    
    // Iterate on the map and compute the average pose for each object
    for(auto element: tag_detections_){
        // Compute the average pose
        geometry_msgs::Pose pose;
        for(auto pose_stamped: element.second){
            pose.position.x += pose_stamped.pose.position.x;
            pose.position.y += pose_stamped.pose.position.y;
            pose.position.z += pose_stamped.pose.position.z;
            pose.orientation.x += pose_stamped.pose.orientation.x;
            pose.orientation.y += pose_stamped.pose.orientation.y;
            pose.orientation.z += pose_stamped.pose.orientation.z;
            pose.orientation.w += pose_stamped.pose.orientation.w;
        }
        pose.position.x /= element.second.size();
        pose.position.y /= element.second.size();
        pose.position.z /= element.second.size();
        pose.orientation.x /= element.second.size();
        pose.orientation.y /= element.second.size();
        pose.orientation.z /= element.second.size();
        pose.orientation.w /= element.second.size();

        // Store the pose and the index
        poses.push_back(pose);
        index.push_back(element.first);
        
    }
    // Store the poses and the index in the goal
    result_.poses = poses;
    result_.ids = index;


    // Raise the head
    setHeadPosition(0.0, 0.0);

    // Clear the map
    tag_detections_.clear();
    
    // Publish the result
    as_.setSucceeded(result_);
}

/// @brief Function that is called when the tag_detections topic is received, it stores the detections
///        after transforming them in the base frame in a map  
/// @param msg Message containing the tag detections
void Camera::tagDetectionsCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{   
    if (!accumulate) {
        return;
    }

    // For each detection, store the pose transformed in base_footprint in the map
    for (const auto& detection : msg->detections) {
        // Create a PoseStamped for the tag pose in the camera frame
        geometry_msgs::PoseStamped tag_pose_camera;
        tag_pose_camera.header = detection.pose.header;
        tag_pose_camera.pose = detection.pose.pose.pose;

        // Create a PoseStamped for the transformed pose in the base frame
        geometry_msgs::PoseStamped tag_pose_base;

        try {
            // Transform the tag pose from the camera frame to the base frame
            tf_buffer_.transform(tag_pose_camera, tag_pose_base, "base_footprint");
            
            // Store the pose in the map
            tag_detections_[detection.id[0]].push_back(tag_pose_base);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Failed to transform tag pose: %s", ex.what());
        }
    }

}

/// @brief Function that sets the head position using the trajectory action server
/// @param pan Pan position
/// @param tilt Tilt position
void Camera::setHeadPosition(double pan, double tilt){
    // Initialize ROS node
    ros::NodeHandle nh;

    // Create a trajectory action client for the head controller
    TrajectoryClient headClient("/head_controller/follow_joint_trajectory", true);

    // Wait for the action server to come up
    headClient.waitForServer();

    // Create a goal for the head movement
    control_msgs::FollowJointTrajectoryGoal goal;

    // Set the joint names for the head controller
    goal.trajectory.joint_names.push_back("head_1_joint");
    goal.trajectory.joint_names.push_back("head_2_joint");

    // Create a trajectory point
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(pan);  // Pan position
    point.positions.push_back(tilt); // Tilt position
    point.time_from_start = ros::Duration(2.0);  // Time to reach the goal

    // Add the trajectory point to the goal
    goal.trajectory.points.push_back(point);

    // Send the goal to the action server
    headClient.sendGoal(goal);

    // Wait for the action to finish
    headClient.waitForResult();

    // Check if the action was successful
    if (headClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Head movement succeeded");
    } else {
        ROS_ERROR("Head movement failed");
    }

    //Close the nodehandle
    nh.shutdown();

    return;
}

/// @brief Function that is called when the camera topic is received, it stores the image as a cv::Mat
/// @param msg Message containing the camera image
void Camera::imageCallback(const sensor_msgs::ImageConstPtr& msg){
    // Convert the image to a cv::Mat and store it
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_ = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

/// @brief Function that is called when color recognition is required
/// @param goal Goal containing the order of the objects
std::vector<int> Camera::colorGoalCB(const group_04_a2::CameraGoalConstPtr &goal){

    ROS_INFO("Recognizing the colors");
    // Take the image and extract three vertical subimages
    cv::Mat image = image_;
    // Convert the image to HSV, we will use the H channel to recognize the colors
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);

    cv::Mat sub_image_1 = image(cv::Rect(0, 0, image.cols/3, image.rows));
    cv::Mat sub_image_2 = image(cv::Rect(image.cols/3, 0, image.cols/3, image.rows));
    cv::Mat sub_image_3 = image(cv::Rect(2*image.cols/3, 0, image.cols/3, image.rows));
    // Split the image in three channels
    std::vector<cv::Mat> sub_image_1_channels;
    std::vector<cv::Mat> sub_image_2_channels;
    std::vector<cv::Mat> sub_image_3_channels;
    cv::split(sub_image_1, sub_image_1_channels);
    cv::split(sub_image_2, sub_image_2_channels);
    cv::split(sub_image_3, sub_image_3_channels);

    // Assign blue, green and red to the three subimages based on the histogram
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    bool uniform = true;
    bool accumulate = false;
    cv::Mat hist_1, hist_2, hist_3;
    cv::calcHist(&sub_image_1_channels[0], 1, 0, cv::Mat(), hist_1, 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&sub_image_2_channels[1], 1, 0, cv::Mat(), hist_2, 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&sub_image_3_channels[2], 1, 0, cv::Mat(), hist_3, 1, &histSize, &histRange, uniform, accumulate);
    //compute histograms
    std::vector<int> color_codes;
    int max_1 = 0, max_2 = 0, max_3 = 0;
    int max_index_1 = 0, max_index_2 = 0, max_index_3 = 0;
    for(int i = 0; i < histSize; i++){
        if(hist_1.at<float>(i) > max_1){
            max_1 = hist_1.at<float>(i);
            max_index_1 = i;
        }
        if(hist_2.at<float>(i) > max_2){
            max_2 = hist_2.at<float>(i);
            max_index_2 = i;
        }
        if(hist_3.at<float>(i) > max_3){
            max_3 = hist_3.at<float>(i);
            max_index_3 = i;
        }
    }
    // 1 blue, 2 green, 3 red
    cv::Mat blue, green, red;
    if(max_index_1 < max_index_2 && max_index_1 < max_index_3){
        color_codes.push_back(1);
        if(max_index_2 < max_index_3){
            color_codes.push_back(2);
            color_codes.push_back(3);
        } else {
            color_codes.push_back(3);
            color_codes.push_back(2);
        }
    } else if(max_index_2 < max_index_1 && max_index_2 < max_index_3){
        color_codes.push_back(2);
        if(max_index_1 < max_index_3){
            color_codes.push_back(1);
            color_codes.push_back(3);
        } else {
            color_codes.push_back(3);
            color_codes.push_back(1);
        }
    } else {
        color_codes.push_back(3);
        if(max_index_1 < max_index_2){
            color_codes.push_back(1);
            color_codes.push_back(2);
        } else {
            color_codes.push_back(2);
            color_codes.push_back(1);
        }
    }

    return color_codes;
}