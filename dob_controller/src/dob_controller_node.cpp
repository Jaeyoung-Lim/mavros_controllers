//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "dob_controller/dob_controller.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"dob_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  DisturbanceObserverCtrl dobcontroller(nh, nh_private);
  ros::spin();
  return 0;
}
