//
// Created by jalim on 11.10.18.
//

#include "trajectory_publisher/shapetrajectory.h"

shapetrajectory::shapetrajectory(int type) :
  N(0),
  dt_(0.1),
  T_(1.0),
  type_(type) {

  traj_omega_ = 1.0;
  traj_axis_ << 0.0, 0.0, 1.0;
  traj_radial_ << 1.0, 0.0, 0.0;
  traj_origin_ << 0.0, 0.0, 1.0;

};

shapetrajectory::~shapetrajectory(){

};

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos){

  traj_origin_ = pos;

}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel){

  traj_origin_ = pos;

}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk){

  traj_origin_ = pos;

}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk){

  traj_origin_ = pos;

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

void shapetrajectory::setTrajectory(int ID, double omega, Eigen::Vector3d axis, double radius,
                               Eigen::Vector3d origin) {
  type_ = ID;
  traj_axis_ = axis;
  traj_omega_ = omega;
  traj_radius_ = radius;
  traj_origin_ = origin;

}

void shapetrajectory::setTrajectory(int ID) {
  double radius, omega;
  Eigen::Vector3d axis,origin;

  switch (ID) {
    case TRAJ_ZERO: //stationary trajectory
      omega = 0.0;
      radius = 0.0;
      axis << 0.0, 0.0, 1.0;
      origin << 0.0, 0.0, 1.0;
      break;
    case TRAJ_CIRCLE: //circular trajectory
      omega = 1.0;
      radius = 2.0;
      axis << 0.0, 0.0, 1.0;
      origin << 0.0, radius, 0.0;
      break;
    case TRAJ_LAMNISCATE: //Lemniscate of Genoro
      omega = 1.0;
      radius = 2.0;
      axis << 0.0, 0.0, 1.0;
      origin << 0.0, radius, 0.0;
      break;
  }
  setTrajectory(ID, omega, axis, radius, origin);
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