//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "controller_benchmark/controller_benchmark.h"

int main(int argc, char** argv) {
  ros::init(argc,argv,"geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ControllerEvaluator geometricController(nh, nh_private);
  ros::spin();
  return 0;
}
