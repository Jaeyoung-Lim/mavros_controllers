//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef CONTROLLER_EVAL_H
#define CONTROLLER_EVAL_H

#include <Eigen/Dense>

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class ControllerEvaluator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber mavposeSub_;
    ros::Subscriber mavtwistSub_;
    ros::Subscriber gzmavposeSub_;

    std::string mav_name_;

    Eigen::Vector3d mavPos_;
    Eigen::Vector3d mavVel_;
    Eigen::Vector3d mavRate_;
    Eigen::Vector4d mavAtt_;
    Eigen::Vector3d gt_mavPos_;
    Eigen::Vector3d gt_mavVel_;
    Eigen::Vector3d gt_mavRate_;
    Eigen::Vector4d gt_mavAtt_;
    Eigen::Vector3d error_pos_;
    Eigen::Vector3d error_vel_;

    void mavposeCallback(const geometry_msgs::PoseStamped& msg);
    void mavtwistCallback(const geometry_msgs::TwistStamped& msg);
    void gzmavposeCallback(const gazebo_msgs::ModelStates& msg);

  public:
    ControllerEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ ControllerEvaluator();
};


#endif
