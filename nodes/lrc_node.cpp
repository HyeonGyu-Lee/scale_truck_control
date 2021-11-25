#include "lrc/lrc.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lrc_node");
  ros::NodeHandle nodeHandle("~");
  LocalResiliencyCoordinator::LocalRC LocalRC(nodeHandle);  
  ros::spin();
  return 0;
}
