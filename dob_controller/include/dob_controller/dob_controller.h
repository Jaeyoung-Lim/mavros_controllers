//  March/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef DISTURBANCEOBSERVERCTRL_H
#define DISTURBANCEOBSERVERCTRL_H

#include <ros/ros.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
class DisturbanceObserverCtrl
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_, statusloop_timer_;


    Eigen::Vector3d a0, a1, tau;
    double a0_x, a0_y, a0_z, a1_x, a1_y, a1_z, tau_x, tau_y, tau_z;
    double dhat_max, dhat_min;
    std::vector<Eigen::Vector2d> q_, p_;

    void CmdLoopCallback(const ros::TimerEvent& event);
    void StatusLoopCallback(const ros::TimerEvent& event);
    Eigen::Vector3d disturbanceobserver(Eigen::Vector3d pos_error, Eigen::Vector3d acc_setpoint);

  public:
    DisturbanceObserverCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ DisturbanceObserverCtrl();
};


#endif
