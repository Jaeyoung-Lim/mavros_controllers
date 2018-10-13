//
// Created by jalim on 29.08.18.
//

#include "trajectory_publisher/trajectory.h"

trajectory::trajectory() {

  N = 0;
  dt_ = 0.1;
  T_ = 1.0;

  traj_axis_ << 0.0, 0.0, 1.0;
  traj_origin_ << 0.0, 0.0, 0.0;

  c_x_ << 0.0, 0.0, 0.0, 0.0;
  c_y_ << 0.0, 0.0, 0.0, 0.0;
  c_z_ << 0.0, 0.0, 0.0, 0.0;

};

trajectory::~trajectory(){

};


Eigen::Vector3d trajectory::getVelocity(double time){

}

nav_msgs::Path trajectory::getSegment(){

  Eigen::Vector3d targetPosition;
  Eigen::Vector4d targetOrientation;
  nav_msgs::Path segment;

  int N = T_/dt_; //Resolution of the trajectory to be published

  targetOrientation << 1.0, 0.0, 0.0, 0.0;
  geometry_msgs::PoseStamped targetPoseStamped;

  for(int i = 0 ; i < N ; i++){
    targetPosition = this->getPosition(i*dt_);
    targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
    segment.poses.push_back(targetPoseStamped);
  }
  return segment;
}

geometry_msgs::PoseStamped trajectory::vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation){
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}
