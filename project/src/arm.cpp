#include <group_04_a2/arm.h>

/// @brief Start the pick sequence
/// @param goal contains the ids of the objects on the table
void Arm::pickObject(const group_04_a2::ArmGoalConstPtr &goal){
    //Pick the vector of objects and ids
    std::vector<geometry_msgs::Pose> objects = goal->poses;
    std::vector<int> ids = goal->ids;

    // Add the objects to the collision objects
    std::vector<std::string> obj_names = addCollisionObjects(objects, ids, true);
    // Move the arm to the object
    std::vector<geometry_msgs::Pose> up_path = pickObj(objects[0], ids[0]);

    // Grip and attach the object
    gripper(false, ids[0]);

    // Raise the object
    up_path[0].position.z += 0.2;
    moveArmPath(up_path);

    // Tuck again the arm as in the beginning
    safePose(true);

    // After placing, clear the planning scene
    planning_scene_interface_.removeCollisionObjects(obj_names);

    as_.setSucceeded();
}

/// @brief Start the place sequence
/// @param goal contains the ids of the objects on the table
void Arm::placeObject(const group_04_a2::ArmGoalConstPtr &goal){   
    std::vector<geometry_msgs::Pose> objects = goal->poses;
    std::vector<int> ids = goal->ids;

    // Add the objects to the collision objects
    std::vector<std::string> obj_names = addCollisionObjects(objects, ids, false);

    // Move the arm to the safe pose
    safePose(false);

    // Place the object
    std::vector<geometry_msgs::Pose> up_path = placeObj(objects[0], ids[0]);

    // Detach the object
    gripper(true, goal->ids[0]);

    // Raise the arm
    up_path[0].position.z += 0.2;
    moveArmPath(up_path);

    // Adjust the object now that it is on the table
    addCollisionObjects(objects, ids, false, true);

    // Tuck again the arm as in the beginning
    safePose(true);

    // After placing, clear the planning scene
    planning_scene_interface_.removeCollisionObjects(obj_names);

    as_.setSucceeded();
}

/// @brief Subroutine to place the object using intermediate poses
/// @param object, pose of the object to place
/// @param id, id of the object to place
/// @return path of poses to go up from the object
std::vector<geometry_msgs::Pose> Arm::placeObj(const geometry_msgs::Pose& object, int id){
    
    // Angle to place the object
    tf2::Quaternion q; q.setRPY(0, +M_PI/2, 0);

    // Vectors for the poses and the return path
    std::vector<geometry_msgs::Pose> waypoints;
    std::vector<geometry_msgs::Pose> up_path;
    geometry_msgs::Pose pose_1;
    pose_1.position = object.position;
    pose_1.position.x += 0.17;
    pose_1.position.y += 0.05;
    pose_1.orientation.x = q.x();
    pose_1.orientation.y = q.y();
    pose_1.orientation.z = q.z();
    pose_1.orientation.w = q.w();
    pose_1.position.z += 1.0;

    waypoints.push_back(pose_1);
    waypoints.push_back(pose_1);
    up_path.push_back(pose_1);

    // Move the arm
    moveArmPath(waypoints);

    return up_path;
}   


/// @brief Plans and executes the path given in input
/// @param path, path of the motions the robot arm has to do to reach the target position
void Arm::moveArmPath(const std::vector<geometry_msgs::Pose>& path)
{
    ROS_INFO("-----STARTING PATH------");

    moveit::planning_interface::MoveGroupInterface move_group_interface("arm_torso");
    move_group_interface.setPoseReferenceFrame("base_footprint"); //or base_footprint
    move_group_interface.setNumPlanningAttempts(16);
    move_group_interface.setPlanningTime(7);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    for(int i=0; i<path.size(); i++){

        // If its the last pose, do a cartesian path
        //*** THIS WORKS ONLY IN LOCAL SIMULATION ***

       /* if(i == path.size()-1){
            geometry_msgs::Pose current_pose = move_group_interface.getCurrentPose().pose;
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(current_pose);
            waypoints.push_back(path[i]);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            my_plan.trajectory_ = trajectory;

            bool success = (fraction >=0.5);

            if (success){
                move_group_interface.execute(my_plan);
                ROS_INFO("ONE STEP DONE");
            }
            else{
                ROS_ERROR("FAILED TO PLAN NEXT MOVE --- ABORT");
                break;
            }
        }
        else{*/ //Otherwise, do a normal plan

            geometry_msgs::Pose step = path[i];
            move_group_interface.setPoseTarget(step);

            bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
            if (success){
                move_group_interface.execute(my_plan); 
            }
            else{
                ROS_ERROR("FAILED TO PLAN NEXT MOVE --- ABORT");
                break;
            }
        //}
    }

    ROS_INFO("-----ENDING PATH------");

    return;
}


/// @brief Subroutine to pick the object using intermediate poses
/// @param object, pose of the object to pick
/// @param id, id of the object to pick
/// @return path of poses to go up from the object
std::vector<geometry_msgs::Pose> Arm::pickObj(const geometry_msgs::Pose& object, int id){
    // Move to a safe pose
    safePose(false);
    std::vector<geometry_msgs::Pose> up_path;

    if(id == 1){
    //BLUE EXAGON ARM POSE - OK
    std::vector<geometry_msgs::Pose> path_blue;

    tf2::Quaternion q;
    geometry_msgs::Pose pose_0;
    pose_0.position = object.position;
    pose_0.position.z = pose_0.position.z - returnDimesions(id)[1] / 2 +0.35;
    q.setRPY(0, +M_PI/2, 0);
    pose_0.orientation.x = q.x();
    pose_0.orientation.y = q.y();
    pose_0.orientation.z = q.z();
    pose_0.orientation.w = q.w();
    path_blue.push_back(pose_0);
    up_path.push_back(pose_0);

    geometry_msgs::Pose pose_1;
    pose_1.position = object.position;
    pose_1.position.z = pose_1.position.z - returnDimesions(id)[1] / 2 +0.19;
    q.setRPY(0, +M_PI/2, 0);
    pose_1.orientation.x = q.x();
    pose_1.orientation.y = q.y();
    pose_1.orientation.z = q.z();
    pose_1.orientation.w = q.w();
    path_blue.push_back(pose_1); 
    
    // Move the arm
    moveArmPath(path_blue);
    
   }
   else if (id == 3){
    //RED CUBE ARM POSES - OK

    std::vector<geometry_msgs::Pose> path_red;

    tf2::Quaternion q;
    geometry_msgs::Pose pose_0;
    pose_0.position = object.position;
    pose_0.position.z = pose_0.position.z - returnDimesions(id)[1] / 2 +0.35;
    q.setRPY(0, +M_PI/2, 0);
    pose_0.orientation.x = q.x();
    pose_0.orientation.y = q.y();
    pose_0.orientation.z = q.z();
    pose_0.orientation.w = q.w();
    path_red.push_back(pose_0);
    up_path.push_back(pose_0);

    geometry_msgs::Pose pose_1;
    pose_1.position = object.position;
    pose_1.position.z = pose_1.position.z - returnDimesions(id)[1] / 2 +0.22;
    q.setRPY(0, +M_PI/2, 0);
    pose_1.orientation.x = q.x();
    pose_1.orientation.y = q.y();
    pose_1.orientation.z = q.z();
    pose_1.orientation.w = q.w();
    path_red.push_back(pose_1); 
    
    // Move the arm
    moveArmPath(path_red);
   }
    else {
    //GREEN PYRAMID ARM POSES

    std::vector<geometry_msgs::Pose> path_green;

    tf2::Quaternion q;

    geometry_msgs::Pose pose_2;
    pose_2.position = object.position;
    pose_2.position.z += 0.30;
    q.setRPY(0, +M_PI/2, +M_PI/3);
    pose_2.orientation.x = q.x();
    pose_2.orientation.y = q.y();
    pose_2.orientation.z = q.z();
    pose_2.orientation.w = q.w();
    path_green.push_back(pose_2);
    up_path.push_back(pose_2);

    geometry_msgs::Pose pose_3;
    pose_3.position = object.position;
    pose_3.position.z += 0.30-0.08;
    q.setRPY(0, +M_PI/2, +M_PI/3);
    pose_3.orientation.x = q.x();
    pose_3.orientation.y = q.y();
    pose_3.orientation.z = q.z();
    pose_3.orientation.w = q.w();
    path_green.push_back(pose_3);

    // Move the arm
    moveArmPath(path_green);
    }

    return up_path;

}

/// @brief Callback function for the action server
/// @param goal contains the ids of the objects on the table and a bool to pick or place
void Arm::goalCB(const group_04_a2::ArmGoalConstPtr &goal){
    auto goal_ = as_.acceptNewGoal();

    //See if we need to pick or place
    bool pick = goal->pick;
    if(pick) pickObject(goal);
    else placeObject(goal);
}

/// @brief Open or close the gripper
/// @param open, true if open, false if close
/// @param id of the object to attach or detach
void Arm::gripper(bool open, int id){

    // Remove the object from the collision objects
    std::vector<std::string> object_ids;
    object_ids.push_back("object" + std::to_string(id));
    planning_scene_interface_.removeCollisionObjects(object_ids);

    // Attach or detach the object
    if (open) detachObjectFromGripper(id);
    else attachObjectToGripper(id);
    
    // Create a publisher for the gripper command
    ros::Publisher gripperPub = nh_.advertise<trajectory_msgs::JointTrajectory>("/parallel_gripper_controller/command", 10);
    ros::Duration(1.0).sleep();
    // Create a gripper command message
    trajectory_msgs::JointTrajectory gripperMsg;
    gripperMsg.joint_names.push_back("gripper_left_finger_joint");
    gripperMsg.joint_names.push_back("gripper_right_finger_joint");
    gripperMsg.points.resize(1);
    gripperMsg.points[0].positions.resize(2);
    gripperMsg.points[0].positions[0] = open ? 0.5 : 0.0;
    gripperMsg.points[0].positions[1] = open ? 0.5 : 0.0;
    gripperMsg.points[0].time_from_start = ros::Duration(1.0);

    // Publish the gripper command
    gripperPub.publish(gripperMsg);

    ros::spinOnce();

    // Wait for the gripper to finish
    ros::Duration(1.5).sleep();

    // Close the publisher
    gripperPub.shutdown();
}