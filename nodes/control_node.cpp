#include <scale_truck_control/ScaleTruckController.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "scale_truck_control_node");
  ros::NodeHandle nodeHandle("~");
  scale_truck_control::ScaleTruckController STC(nodeHandle);  
  ros::spin();
  return 0;
}
