//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "trajectory_controller/trajectory_controller.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"trajectory_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  trajectoryCtrl trajTrackingController(nh, nh_private);
  ros::spin();
  return 0;
}
