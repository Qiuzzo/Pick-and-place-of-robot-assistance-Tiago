#include <group_04_a2/arm.h>

/// @brief Return the dimensions of each collision object, based on the id
/// @param id of the object to return its dimensions
/// @return vector of dimensions of the collision object
std::vector<double> Arm::returnDimesions(int id){
    //if object is BLUE
    if(id == 1) { return std::vector<double> {0.050301,0.054000,0.1}; }

    //if object is GREEN
    else if(id == 2) { return std::vector<double> {0.06650,0.05500,0.03250};}

    //if object is RED
    else if (id == 3) { return std::vector<double> {0.06800,0.06800,0.05600}; }

    //if object is Gold
    else { return std::vector<double> {0.086602,0.10000, 0.22500}; }
}

/// @brief Move the arm to a safe pose and tuck it if requested
/// @param tuck, true if tuck, false if safe pose
void Arm::safePose(bool tuck){
        
    // Go up again before tucking

    // Variables for MoveIt
    moveit::planning_interface::MoveGroupInterface move_group_interface("arm_torso");
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arm_torso");
    
    move_group_interface.setNumPlanningAttempts(15);
    move_group_interface.setPlanningTime(5);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Set the initial position
    std::vector<double> initial_position = {0.300, 4 * (M_PI / 180), 45 * (M_PI / 180), -80 * (M_PI / 180), 33 * (M_PI / 180), -90 * (M_PI / 180), 78 * (M_PI / 180), 0};
    move_group_interface.setJointValueTarget(initial_position);

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success){
        move_group_interface.execute(my_plan);
    }
    else{
        ROS_ERROR("FAILED TO PLAN INITIAL POSITION --- ABORT");
        return;
    }

    // If we need to tuck the arm
    if(tuck){
        // Tuck again the arm as in the beginning
        moveit::planning_interface::MoveGroupInterface move_group_interface("arm_torso");
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("arm_torso");
        
        move_group_interface.setNumPlanningAttempts(15);
        move_group_interface.setPlanningTime(5);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // Set the initial position
        std::vector<double> initial_position = {0.195, 11 * (M_PI / 180), -84 * (M_PI / 180), -20 * (M_PI / 180), 103 * (M_PI / 180), -90 * (M_PI / 180), 80 * (M_PI / 180), 0};

        move_group_interface.setJointValueTarget(initial_position);

        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success){
            move_group_interface.execute(my_plan);
        }
        else{
            ROS_ERROR("FAILED TO TUCK ARM --- ABORT");
            return;
        }
    }
}


/// @brief Add the objects to the collision objects, if pick is false the added objects will be the place tables,
///         if adjust is true the place tables will be enlarged
/// @param objects vector of poses of the objects
/// @param ids  vector of ids of the objects
/// @param pick, true if pick, false if place
/// @param adjust, true if we need to adjust the object
/// @return vector of names of the added objects
std::vector<std::string> Arm::addCollisionObjects(std::vector<geometry_msgs::Pose>& objects, std::vector<int>& ids, bool pick, bool adjust){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<std::string> collision_names;

    // If we're picking
    if (pick){

        // Add the objects to the collision objects
        for(int i = 0; i < objects.size(); i++)
        {
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = "base_footprint";
            collision_object.id = "object" + std::to_string(ids[i]);
            collision_names.push_back(collision_object.id);
            // Use a box as the collision object
            shape_msgs::SolidPrimitive object_primitive;
            object_primitive.type = shape_msgs::SolidPrimitive::BOX;
            std::vector<double> dim_s = returnDimesions(ids[i]);
            object_primitive.dimensions.resize(3);

            object_primitive.dimensions[0] = dim_s[0];
            object_primitive.dimensions[1] = dim_s[1];
            object_primitive.dimensions[2] = dim_s[2];

            // Set the pose of the collision object
            geometry_msgs::Pose pose = objects[i];
            if(ids[i] == 2) { pose.position.z -= dim_s[1]/2;}
            else { pose.position.z -= dim_s[1]; }

            pose.orientation = objects[i].orientation;

            collision_object.primitive_poses.push_back(pose);
            collision_object.primitives.push_back(object_primitive);
            collision_object.operation = moveit_msgs::CollisionObject::ADD;

            collision_objects.push_back(collision_object);
        }

        // Add the table to the collision object
        moveit_msgs::CollisionObject table_coll_object;
        table_coll_object.header.frame_id = "map";
        table_coll_object.id = "table";
        collision_names.push_back(table_coll_object.id);
        shape_msgs::SolidPrimitive primitive;
        primitive.type = shape_msgs::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.944400; // x dimension
        primitive.dimensions[1] = 0.944400;  // y dimension
        primitive.dimensions[2] = 0.774700;  // z dimension

        geometry_msgs::Pose pose;
        pose.position.x = 1.2451+6.55;
        pose.position.y = -1.6131-1.35;
        pose.position.z = primitive.dimensions[2] / 2; //NOT 0; IDK, MY TABLE SEEMS HALF IN THE GROUND

        table_coll_object.primitives.push_back(primitive);
        table_coll_object.primitive_poses.push_back(pose);
        table_coll_object.operation = moveit_msgs::CollisionObject::ADD;

        collision_objects.push_back(table_coll_object);
    }
    else{ // If we're placing
        // Tables for the place task
       moveit_msgs::CollisionObject red_pill;
        red_pill.header.frame_id = "map";
        red_pill.id = "red_pill";
        collision_names.push_back(red_pill.id);
        shape_msgs::SolidPrimitive primitive;
        primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = 0.72; // height
        primitive.dimensions[1] = 0.235;  // radius
        if(adjust) primitive.dimensions[0] = 0.72+0.2; // height

        geometry_msgs::Pose pose;
        pose.position.x = 4.0073+6.55;
        pose.position.y = 1.0159-1.35;
        pose.position.z = 0.345;

        red_pill.primitives.push_back(primitive);
        red_pill.primitive_poses.push_back(pose);
        red_pill.operation = moveit_msgs::CollisionObject::ADD;

        collision_objects.push_back(red_pill);
        // Green table
        moveit_msgs::CollisionObject green_pill;
        green_pill.header.frame_id = "map";
        green_pill.id = "green_pill";
        collision_names.push_back(green_pill.id);
        shape_msgs::SolidPrimitive primitive2;
        primitive2.type = shape_msgs::SolidPrimitive::CYLINDER;
        primitive2.dimensions.resize(2);
        primitive2.dimensions[0] = 0.72; // height
        primitive2.dimensions[1] = 0.235;  // radius
        if (adjust) primitive2.dimensions[0] = 0.72+0.2; // height

        geometry_msgs::Pose pose2;
        pose2.position.x = 5.0074+6.55;
        pose2.position.y = 1.0159-1.35;
        pose2.position.z = 0.345;

        green_pill.primitives.push_back(primitive2);
        green_pill.primitive_poses.push_back(pose2);
        green_pill.operation = moveit_msgs::CollisionObject::ADD;

        collision_objects.push_back(green_pill);
        // Blue table
        moveit_msgs::CollisionObject blue_pill;
        blue_pill.header.frame_id = "map";
        blue_pill.id = "blue_pill";
        collision_names.push_back(blue_pill.id);
        shape_msgs::SolidPrimitive primitive3;
        primitive3.type = shape_msgs::SolidPrimitive::CYLINDER;
        primitive3.dimensions.resize(2);
        primitive3.dimensions[0] = 0.72; // height
        primitive3.dimensions[1] = 0.235;  // radius
        if (adjust) primitive3.dimensions[0] = 0.72+0.2; // height

        geometry_msgs::Pose pose3;
        pose3.position.x = 6.00714+6.55;
        pose3.position.y = 1.0159-1.35;
        pose3.position.z = 0.345;

        blue_pill.primitives.push_back(primitive3);
        blue_pill.primitive_poses.push_back(pose3);
        blue_pill.operation = moveit_msgs::CollisionObject::ADD;

        collision_objects.push_back(blue_pill);
        
    }
    // Add the collision object to the planning scene
    planning_scene_interface_.applyCollisionObjects(collision_objects);
    // return the names of the collision objects to be removed later
    return collision_names;
}

/// @brief Attach the object to the gripper using gazebo_ros_link_attacher
/// @param id of the object
void Arm::attachObjectToGripper(int id){
    
    // Create a service client for attaching objects
    attachClient_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");

    // Create an Attach service message
    gazebo_ros_link_attacher::Attach attachSrv;
    // Robot model
    attachSrv.request.model_name_1 = "tiago"; 
    attachSrv.request.link_name_1 = "arm_7_link"; 
    // Object model
    if (id == 1){
        attachSrv.request.model_name_2 = "Hexagon";
        attachSrv.request.link_name_2 = "Hexagon_link";
    }
    else if (id == 2){
        attachSrv.request.model_name_2 = "Triangle";
        attachSrv.request.link_name_2 = "Triangle_link";
    }
    else if (id == 3){
        attachSrv.request.model_name_2 = "cube";
        attachSrv.request.link_name_2 = "cube_link";
    }

    // Call the Attach service
    if (attachClient_.call(attachSrv))
    {
    }
    else
    {
        ROS_ERROR("Failed to attach object to arm_7_link");
    }
}

/// @brief Detach the object from the gripper using gazebo_ros_link_attacher 
/// @param id of the object
void Arm::detachObjectFromGripper(int id){

    // Create a service client for detaching objects
    detachClient_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    // Create a Detach service message
    gazebo_ros_link_attacher::Attach detachSrv;
    // Robot model
    detachSrv.request.model_name_1 = "tiago"; 
    detachSrv.request.link_name_1 = "arm_7_link"; 
    // Object model
    if (id == 1){
        detachSrv.request.model_name_2 = "Hexagon";
        detachSrv.request.link_name_2 = "Hexagon_link";
    }
    else if (id == 2){
        detachSrv.request.model_name_2 = "Triangle";
        detachSrv.request.link_name_2 = "Triangle_link";
    }
    else if (id == 3){
        detachSrv.request.model_name_2 = "cube";
        detachSrv.request.link_name_2 = "cube_link";
    }

    // Call the Detach service
    if (detachClient_.call(detachSrv)) ;
    else ROS_ERROR("Failed to detach object from arm_7_link");
}

