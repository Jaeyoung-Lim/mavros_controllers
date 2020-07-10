//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "geometric_controller/geometric_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  geometricCtrl* geometricController = new geometricCtrl(nh, nh_private);

  dynamic_reconfigure::Server<geometric_controller::GeometricControllerConfig> srv;
  dynamic_reconfigure::Server<geometric_controller::GeometricControllerConfig>::CallbackType f;
  f = boost::bind(&geometricCtrl::dynamicReconfigureCallback, geometricController, _1, _2);
  srv.setCallback(f);

  ros::spin();
  return 0;
}
