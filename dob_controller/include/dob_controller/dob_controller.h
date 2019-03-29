//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef DISTURBANCEOBSERVERCTRL_H
#define DISTURBANCEOBSERVERCTRL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "geometric_controller/geometric_controller.h"

using namespace std;
using namespace Eigen;
class DisturbanceObserverCtrl
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_, statusloop_timer_;


    Eigen::Vector3d a_input, a_dob, pos_error, vel_error;
    Eigen::Vector3d g_;
    Eigen::Vector3d a0, a1, tau;
    Eigen::Vector3d Kpos_, Kvel_, D_;
    Eigen::Vector3d a_fb, a_des;

    double a0_x, a0_y, a0_z, a1_x, a1_y, a1_z, tau_x, tau_y, tau_z;
    double dhat_max, dhat_min;
 
    std::vector<Eigen::Vector2d> q_, p_;

    void CmdLoopCallback(const ros::TimerEvent& event);
    void StatusLoopCallback(const ros::TimerEvent& event);
    Eigen::Vector3d DisturbanceObserver(Eigen::Vector3d pos_error, Eigen::Vector3d acc_setpoint);

    geometricCtrl geometric_controller_;

  public:
    DisturbanceObserverCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ DisturbanceObserverCtrl();
};


#endif
