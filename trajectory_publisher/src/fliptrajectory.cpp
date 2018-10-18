//
// Created by jalim on 11.10.18.
//

#include <Eigen/Geometry>

#include "trajectory_publisher/fliptrajectory.h"

fliptrajectory::fliptrajectory() :
  trajectory(),
  N(0),
  dt_(0.1),
  T_(1.0)  {

  traj_omega_ = 6.0;
  traj_axis_ << 1.0, 0.0, 0.0;
  thrust = 30.0;

};

fliptrajectory::~fliptrajectory(){

};

void fliptrajectory::initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega){
  //Generate primitives based on current state for smooth trajectory
  Eigen::Vector3d offset;
  offset << 0.0, 0.0, 0.5;
  traj_origin_ = pos;
  traj_omega_ = omega;
  T_ = 2*3.14 / traj_omega_;
  traj_axis_ = axis;
}

void fliptrajectory::generatePrimitives(Eigen::Vector3d pos){

}

void fliptrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel){

}

void fliptrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk){

}

void fliptrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk){

}

Eigen::Vector3d fliptrajectory::getPosition(double time){

  return traj_origin_;
}

Eigen::Vector3d fliptrajectory::getVelocity(double time){

  Eigen::Vector3d velocity;

  velocity = Eigen::Vector3d::Zero();

  return velocity;

}

Eigen::Vector3d fliptrajectory::getAcceleration(double time){

  Eigen::Vector3d acceleration;
  Eigen::Matrix3d R;

  if (time < 0.5*T_) acceleration = thrust* Eigen::Vector3d::UnitZ();
  else if(time < 2.3*T_){ //Turn the acceleration vector for onlythe duration
    R = Eigen::AngleAxisd(traj_omega_*time, traj_axis_);
    acceleration = thrust* R * Eigen::Vector3d::UnitZ();
  }
  else if(time < 3.5*T_){
    acceleration = 2*thrust * Eigen::Vector3d::UnitZ();
  }
  else acceleration = 0.1 * thrust * Eigen::Vector3d::UnitZ();
  return acceleration;

}

nav_msgs::Path fliptrajectory::getSegment(){

  Eigen::Vector3d targetPosition;
  Eigen::Vector4d targetOrientation;
  nav_msgs::Path segment;

  targetOrientation << 1.0, 0.0, 0.0, 0.0;
  geometry_msgs::PoseStamped targetPoseStamped;

  for(double t = 0 ; t < this->getDuration() ; t+=this->getsamplingTime()){
    targetPosition = this->getPosition(t);
    targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
    segment.poses.push_back(targetPoseStamped);
  }
  return segment;
}

geometry_msgs::PoseStamped fliptrajectory::vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation){
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
