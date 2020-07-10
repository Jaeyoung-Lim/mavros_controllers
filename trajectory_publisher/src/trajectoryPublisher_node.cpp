
#include "trajectory_publisher/trajectoryPublisher.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_publisher");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  trajectoryPublisher referencePublisher(nh, nh_private);
  ros::spin();
  return 0;
}
