//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef CONTROLLER_EVAL_H
#define CONTROLLER_EVAL_H

#include <ros/ros.h>

class ControllerEvaluator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

  public:
    ControllerEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ ControllerEvaluator();
};


#endif
