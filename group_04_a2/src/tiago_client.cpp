#include <group_04_a2/client.h>
#include <math.h>

//*** Main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "tiago_management");
    ROS_INFO("Starting tiago_management node");
    // Initialize the action client
    actionlib::SimpleActionClient<group_04_a2::TiagoAction> ac("tiago_pose", true);

    // Ask human_node for the order of the objects
    std::vector<int> objects = get_order();

    // Wait for the arm to tuck and go to a starting position
    ros::Duration(13.0).sleep();
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started");
    tf2::Quaternion q; q.setRPY(0,0,0);
    group_04_a2::TiagoGoal goal = createGoal(8.5, 0, 0, q.x(), q.y(), q.z(), q.w(), true);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    ac.waitForResult();

    // Get the map with the waypoints
    std::map<int, geometry_msgs::PoseStamped> position_map = getPositionMap(); 

    // For each object
    for(int actual_color : objects){
        ROS_INFO("Object to pick: %d", actual_color);
        // Container for the camera results
        group_04_a2::CameraResultConstPtr camera_pointer;

        // Move through the waypoints until the object is found
        for(int ix = 1; ix <= 5; ix++){
            geometry_msgs::PoseStamped pose_move = position_map[ix];
            move_to(pose_move, ac);

            // Get the camera results if you're in the table positions
            if(ix == 3 || ix == 4) continue;
            camera_pointer = cameraDetection();

            // If actual_color is not in the camera results, go to the next position
            if(std::find(camera_pointer->ids.begin(), camera_pointer->ids.end(), actual_color) == camera_pointer->ids.end()){
                continue;
            }

            //Send pick goal to arm
            pick_place(camera_pointer->poses, camera_pointer->ids, true);

            // Go to the nearest waypoint
            if(ix == 5) move_to(position_map[4], ac);
            else {  move_to(position_map[3], ac);
                    break;}
        }
            // Go to waypoint place
            group_04_a2::TiagoResultConstPtr result = move_to(position_map[6], ac);

            // Extract the barrel poses from the result
            std::vector<geometry_msgs::PoseStamped> barrel_poses;
            barrel_poses.push_back(result->result_points[2]);   //left barrel pose wrt robot
            barrel_poses.push_back(result->result_points[1]);   //straight barrel pose wrt robot
            barrel_poses.push_back(result->result_points[0]);   //right barrel pose wrt robot

            // Activate the camera to detect the color of the objects and choose the right place
            group_04_a2::CameraResultConstPtr camera_res = cameraDetection(true);
            group_04_a2::TiagoResultConstPtr barrel_pose_place;
            // Fuse the informations together to find the right place
            int i = 0;
            for(i = 0; i < camera_res->ids.size(); i++){
                if (camera_res->ids[i] == actual_color) 
                {
                    ROS_INFO("putting object to DIRECTION %d", camera_res->ids[i]);
                    ROS_INFO("BARREL TO GO at position (%f, %f, %f)", barrel_poses[i].pose.position.x, barrel_poses[i].pose.position.y, barrel_poses[i].pose.position.z);
                    std::vector<geometry_msgs::PoseStamped> pathBarrel = computeBarrelPose(barrel_poses[i], position_map[6], i);
                    for(int j=0; j<pathBarrel.size(); j++)
                        barrel_pose_place = move_to(pathBarrel[j], ac);
                    break;
                }
            }

            // Extract the pose of the closest barrel
            geometry_msgs::Pose barrel_pose;
            for (int j = 0; j < barrel_pose_place.get()->result_points.size(); j++)
            {
                if (j==0) barrel_pose = barrel_pose_place.get()->result_points[j].pose;
                else if (fabs(barrel_pose_place.get()->result_points[j].pose.position.y) < fabs(barrel_pose.position.y))
                    barrel_pose = barrel_pose_place.get()->result_points[j].pose;
               
            }

            //Print the pose of the barrel
            ROS_INFO("Place position (%f, %f, %f)", barrel_pose.position.x, barrel_pose.position.y, barrel_pose.position.z);

            // Set the pose for the place
            std::vector<geometry_msgs::Pose> barrel_pose_vector;
            barrel_pose_vector.push_back(barrel_pose);

            std::vector<int> barrel_id;
            barrel_id.push_back(actual_color);
            //Place the object
            pick_place(barrel_pose_vector, barrel_id, false);

            // Return home
            move_to(position_map[4], ac);
            move_to(position_map[3], ac);

        }
    return 0;
}