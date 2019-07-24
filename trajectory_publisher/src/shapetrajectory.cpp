//
// Created by jalim on 11.10.18.
//

#include "trajectory_publisher/shapetrajectory.h"

shapetrajectory::shapetrajectory(int type) :
  trajectory(),
  N(0),
  dt_(0.1),
  T_(10.0),
  type_(type) {

  traj_omega_ = 2.0;
  traj_axis_ << 0.0, 0.0, 1.0;
  traj_radial_ << 1.0, 0.0, 0.0;
  traj_origin_ << 0.0, 0.0, 1.0;


};

shapetrajectory::~shapetrajectory(){

};

void shapetrajectory::initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega){
  //Generate primitives based on current state for smooth trajectory
  traj_origin_ = pos;
  traj_omega_ = omega;
  T_ = 2*3.14 / traj_omega_;
  traj_axis_ = axis;
  traj_radial_ << 2.0, 0.0, 0.0;

}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos){

}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel){

}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk){

}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk){

}


Eigen::Vector3d shapetrajectory::getPosition(double time){

  Eigen::Vector3d position;
  double theta;

  switch(type_) {
    case TRAJ_ZERO :

      position << 0.0, 0.0, 0.0;
      break;

    case TRAJ_CIRCLE :

      theta = traj_omega_* time;
      position = std::cos(theta) * traj_radial_
                 + std::sin(theta) * traj_axis_.cross(traj_radial_)
                 + (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_
                 + traj_origin_;
      break;

    case TRAJ_LAMNISCATE : //Lemniscate of Genero

      theta = traj_omega_* time;
      position = std::cos(theta) * traj_radial_
                 + std::sin(theta) * std::cos(theta) * traj_axis_.cross(traj_radial_)
                 + (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_
                 + traj_origin_;
      break;
    case TRAJ_STATIONARY : //Lemniscate of Genero

      position = traj_origin_;
      break;


  }
  return position;
}

Eigen::Vector3d shapetrajectory::getVelocity(double time){

  Eigen::Vector3d velocity;
  double theta;

  switch(type_) {
    case TRAJ_CIRCLE :

      velocity = traj_omega_ * traj_axis_.cross(getPosition(time));
      break;
    case TRAJ_STATIONARY :

      velocity << 0.0, 0.0, 0.0;
      break;

    case TRAJ_LAMNISCATE : //Lemniscate of Genero

      theta = traj_omega_* time;
      velocity = traj_omega_ * (-std::sin(theta) * traj_radial_
                 + (std::pow(std::cos(theta), 2) - std::pow(std::sin(theta), 2)) * traj_axis_.cross(traj_radial_)
                 + (std::sin(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_);
      break;

    default :
      velocity << 0.0, 0.0, 0.0;
      break;
  }
  return velocity;

}

Eigen::Vector3d shapetrajectory::getAcceleration(double time){

  Eigen::Vector3d acceleration;

  switch(type_) {
    case TRAJ_CIRCLE :

      acceleration = traj_omega_ * traj_axis_.cross(getVelocity(time));
      break;
    case TRAJ_STATIONARY :

      acceleration << 0.0, 0.0, 0.0;
      break;
    default :
      acceleration << 0.0, 0.0, 0.0;
      break;

  }
  return acceleration;

}

nav_msgs::Path shapetrajectory::getSegment(){

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

geometry_msgs::PoseStamped shapetrajectory::vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation){
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
