#include <group_04_a2/server.h>

//*** Main
int main(int argc, char** argv){
    // Initialize the tiago server for the movement and spin
    ros::init(argc, argv, "tiago_pose");

    Tiago tiago(ros::this_node::getName());
    ros::spin();

    return 0;
}

