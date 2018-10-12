//
// Created by jalim on 11.10.18.
//

#include "trajectory_publisher/shapetrajectory.h"

shapetrajectory::shapetrajectory(int type) :
  N(0),
  dt_(0.1),
  T_(1.0),
  type_(type) {

  traj_axis_ << 0.0, 0.0, 1.0;
  target_initpos << 0.0, 0.0, 0.0;

};

shapetrajectory::~shapetrajectory(){

};

Eigen::Vector3d shapetrajectory::getPosition(double time){

  Eigen::Vector3d position;
  double theta;

  switch(type_) {
    case TRAJ_ZERO :
      position << 0.0, 0.0, 0.0;
      break;
    case TRAJ_CIRCLE :

      theta = traj_omega_* time;
      position = std::cos(theta) * target_initpos
                 + std::sin(theta) * traj_axis_.cross(target_initpos)
                 + (1 - std::cos(theta)) * traj_axis_.dot(target_initpos) * traj_axis_;
      break;
    case TRAJ_LAMNISCATE : //Lemniscate of Genero

      position = std::cos(theta) * target_initpos
                 + std::sin(theta) * std::cos(theta) * traj_axis_.cross(target_initpos)
                 + (1 - std::cos(theta)) * traj_axis_.dot(target_initpos) * traj_axis_;
      break;
  }
  return position;
}

Eigen::Vector3d shapetrajectory::getVelocity(double time){

  Eigen::Vector3d velocity;

  switch(type_) {
    case TRAJ_CIRCLE :
      velocity = traj_omega_ * traj_axis_.cross(getPosition(time));
  }
  return velocity;

}

nav_msgs::Path shapetrajectory::getSegment(){

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

void shapetrajectory::setTrajectory(int ID, double omega, Eigen::Vector3d axis, double radius,
                               Eigen::Vector3d initpos) {
  target_trajectoryID_ = ID;
  traj_axis_ = axis;
  traj_omega_ = omega;
  traj_radius_ = radius;
  target_initpos = initpos;
}

void shapetrajectory::setTrajectory(int ID) {
  double radius = 0, omega = 0;
  Eigen::Vector3d axis, initpos;

  switch (ID) {//TODO: Move standard trajectories to the trajectory class
    case TRAJ_ZERO: //stationary trajectory
      omega = 0.0;
      radius = 0.0;
      axis << 0.0, 0.0, 1.0;
      initpos << 0.0, 0.0, 1.0;
      break;
    case TRAJ_CIRCLE: //circular trajectory
      omega = 1.0;
      radius = 2.0;
      axis << 0.0, 0.0, 1.0;
      initpos << 0.0, radius, 0.0;
      break;
    case TRAJ_LAMNISCATE: //Lemniscate of Genoro
      omega = 1.0;
      radius = 2.0;
      axis << 0.0, 0.0, 1.0;
      initpos << 0.0, radius, 0.0;
      break;
  }
  setTrajectory(ID, omega, axis, radius, initpos);
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