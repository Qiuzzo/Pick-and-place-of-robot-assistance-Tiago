#include <group_04_a2/arm.h>

//*** Main
int main(int argc, char** argv){
  // Initialize the arm server and spin
  ros::init(argc, argv, "tiago_arm");

  Arm object_mover(ros::this_node::getName());
  ros::spin();

  return 0;
}