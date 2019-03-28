//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "dob_controller/dob_controller.h"

using namespace Eigen;
using namespace std;
//Constructor
DisturbanceObserverCtrl::DisturbanceObserverCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &DisturbanceObserverCtrl::CmdLoopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &DisturbanceObserverCtrl::StatusLoopCallback, this); // Define timer for constant loop rate

  nh_.param<double>("/dob_controller/dob/a0_x", a0_x, 10.0);
  nh_.param<double>("/dob_controller/dob/a0_y", a0_y, 10.0);
  nh_.param<double>("/dob_controller/dob/a0_z", a0_z, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_x", a1_x, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_y", a1_y, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_z", a1_z, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_y", a1_y, 10.0);
  nh_.param<double>("/dob_controller/dob/a1_z", a1_z, 10.0);
  nh_.param<double>("/dob_controller/dob/tau_x", tau_x, 10.0);
  nh_.param<double>("/dob_controller/dob/tau_y", tau_y, 10.0);
  nh_.param<double>("/dob_controller/dob/tau_z", tau_z, 10.0);
  nh_.param<double>("/dob_controller/dob/max_dhat", dhat_max, 10.0);
  nh_.param<double>("/dob_controller/dob/min_dhat", dhat_min, -10.0);

  q_.resize(3);
  p_.resize(3);
  for (int i = 0; i < 3; ++i) {
    q_.at(i) << 0.0, 0.0;
    p_.at(i) << 0.0, 0.0;
  }
  a0 << a0_x, a0_y, a0_z;
  a1 << a1_x, a1_y, a1_z;

}
DisturbanceObserverCtrl::~DisturbanceObserverCtrl() {
  //Destructor
}

void DisturbanceObserverCtrl::CmdLoopCallback(const ros::TimerEvent& event){
 
}

void DisturbanceObserverCtrl::StatusLoopCallback(const ros::TimerEvent& event){

}