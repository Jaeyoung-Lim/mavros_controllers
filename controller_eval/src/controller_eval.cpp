//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "controller_eval/controller_eval.h"

//Constructor
ControllerEvaluator::ControllerEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

}
ControllerEvaluator::~ControllerEvaluator() {
  //Destructor
}