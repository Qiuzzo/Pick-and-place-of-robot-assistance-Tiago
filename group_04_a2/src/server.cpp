#include <group_04_a2/server.h>
#include <group_04_a2/obstacle_finder.h>
#include <group_04_a2/motion_law.h>

    /// @brief Function that is called when the goal is received
    void Tiago::goalCB(){
        auto goal = as_.acceptNewGoal();
        goal_ = goal->goal_pose;
        bool linear_init = goal->linear_init;

        if (linear_init){ // Check if we need to move in the corridor
        //Wait some seconds for the arm to tuck
        // Send feedback of tucking
        // Set the feedback header
        feedback_.head_feedback.seq++;
        feedback_.head_feedback.stamp = ros::Time::now();
        feedback_.head_feedback.frame_id = "Goal feedback";
        // Set the feedback message
        feedback_.feedback_message = "Waiting for the arm to tuck";
        as_.publishFeedback(feedback_);
        // Motion Law
        // Send feedback of motion law
        // Set the feedback header
        feedback_.head_feedback.seq++;
        feedback_.head_feedback.stamp = ros::Time::now();
        feedback_.head_feedback.frame_id = "Goal feedback";
        // Set the feedback message
        feedback_.feedback_message = "Motion Control Law activated";
        as_.publishFeedback(feedback_);
        motion(goal_);
        }
        // Send the goal to move_base_simple/goal
        pub_.publish(goal_);

        // Send feedback to the client
         // Set the feedback header
        feedback_.head_feedback.seq++;
        feedback_.head_feedback.stamp = ros::Time::now();
        feedback_.head_feedback.frame_id = "Goal feedback";
         // Set the feedback message
        feedback_.feedback_message = "Goal sent to move_base";
        as_.publishFeedback(feedback_);
    }

    /// @brief Function that is called when the goal is preempted
    void Tiago::preemptCB(){
        as_.setPreempted();
    }

    /// @brief Function that is called when the node move_base/result publishes
    void Tiago::poseCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
        // Check if the robot has reached the goal in /move_base/result
        if(msg->status.status == 3){
            success_ = true;
            // Return feedback to the client
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "The robot has reached the goal";
            // Set the status
            feedback_.status = msg->status.status;
            as_.publishFeedback(feedback_);
            // Call the object position function
            computeObjectPosition();
        }
            // If it failed becuase the point is outside the map
        else if (msg->status.status == 4 && 
                (msg->status.text.find("oscillating") == std::string::npos)){
            success_ = false;
            // Return feedback to the client
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "The robot has not reached the goal: " + msg->status.text;
            // Set the status
            feedback_.status = msg->status.status;
            as_.publishFeedback(feedback_);
        }
        else if (msg->status.status == 4 &&         // Else it failed because of an obstacle
                                                    // => Motion Control Law takes control
                (msg->status.text.find("oscillating") != std::string::npos)){
            success_ = false;
            // Return feedback to the client
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "Narrow passage, switching to Motion Control Law";
            // Set the status
            feedback_.status = 0;
            as_.publishFeedback(feedback_);

            recovery_rotation();
            // Send the goal to move_base_simple/goal
            pub_.publish(goal_);
        }


    }

    /// @brief Function that provides additional feedback to the client
    void Tiago::feedCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
        if(!as_.isActive())
            return; 
        
        if (success_){
        }
        else{
            // Compare the actual position with the previous one
            // We truncate at 3 decimals to deal with noise
            if((int)(pose_actual_.position.x*1000) == (int)(pose_previous_.position.x*1000) &&
                (int)(pose_actual_.position.y*1000) == (int)(pose_previous_.position.y*1000)){
                // Return feedback to the client
                // Set the feedback header
                feedback_.head_feedback.seq++;
                feedback_.head_feedback.stamp = ros::Time::now();
                feedback_.head_feedback.frame_id = "Goal feedback";
                // Set the feedback message
                feedback_.feedback_message = "The robot is stopped";
                as_.publishFeedback(feedback_);
            }
            else{
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "The robot is moving to the goal";
            as_.publishFeedback(feedback_);
            }
        }
    }

    /// @brief Function that updates the data about the robot position
    void Tiago::updateCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        // Save the old position
        pose_previous_= pose_actual_;
        pose_actual_ = msg->pose.pose;
    }

    /// @brief Function that updates the data about the laser scan
    void Tiago::laserCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        laser_msg_ = msg;
    }

    /// @brief Function that computes the position of the objects and sets the success of the goal
    void Tiago::computeObjectPosition(){
        // Return feedback to the client
            // Set the feedback header
            feedback_.head_feedback.seq++;
            feedback_.head_feedback.stamp = ros::Time::now();
            feedback_.head_feedback.frame_id = "Goal feedback";
            // Set the feedback message
            feedback_.feedback_message = "Computing the position of the objects";
            as_.publishFeedback(feedback_);

        // Call the object position function, it wants as input sensor_msgs::LaserScan::ConstPtr&
        std::vector<geometry_msgs::PoseStamped> obstacle_poses = obstacle_finder_function(laser_msg_);
        // Return the result to the client
        result_.result_points = obstacle_poses;
        as_.setSucceeded(result_);
        return;
    }