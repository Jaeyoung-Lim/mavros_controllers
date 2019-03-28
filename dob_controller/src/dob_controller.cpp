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
    // /// Compute BodyRate commands using disturbance observer
  // /// From Hyuntae Kim
  // /// Compute BodyRate commands using differential flatness
  // /// Controller based on Faessler 2017
  // q_ref = acc2quaternion(a_ref - g_, mavYaw_);
  // a_fb = Kpos_.asDiagonal() * errorPos_ + Kvel_.asDiagonal() * errorVel_; //feedforward term for trajectory error
  // a_dob = disturbanceobserver(errorPos_, a_ref + a_fb - a_dob);
  // a_des = a_ref + a_fb - a_dob - g_;
  // q_des = acc2quaternion(a_des, mavYaw_);

 
}

void DisturbanceObserverCtrl::StatusLoopCallback(const ros::TimerEvent& event){

}

Eigen::Vector3d DisturbanceObserverCtrl::disturbanceobserver(Eigen::Vector3d pos_error, Eigen::Vector3d acc_setpoint){

  Eigen::Vector3d acc_input, yq, yp, d_hat;
  double control_dt = 0.01;

  for(int i = 0; i < acc_input.size(); i++){
    //Update dob states
    p_.at(i)(0) = p_.at(i)(0) + p_.at(i)(1) * control_dt;
    p_.at(i)(1) = (-a0(i) * control_dt / std::pow(tau(i),2)) * p_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) *p_.at(i)(1) + control_dt * acc_setpoint(i);
    q_.at(i)(0) = q_.at(i)(0) + control_dt * q_.at(i)(1);
    q_.at(i)(1) = (-a0(i)/std::pow(tau(i), 2)) * control_dt * q_.at(i)(0) + (1 - a1(i) * control_dt / tau(i)) * q_.at(i)(1) + control_dt * pos_error(i);

    //Calculate outputs
    yp(i) = (a0(i) / pow(tau(i), 2)) * p_.at(i)(0);
    yq(i) = (-a1(i)*a0(i) / std::pow(tau(i), 3))*q_.at(i)(0) - (std::pow(a0(i),2) / std::pow(tau(i), 4)) * q_.at(i)(1) + a0(i) / pow(tau(i),2) * pos_error(i);
    d_hat(i) = yq(i) - yp(i);
    d_hat(i) = std::max( std::min( d_hat(i), dhat_max), dhat_min);
    acc_input(i) = d_hat(i);
  }

  return acc_input;
}