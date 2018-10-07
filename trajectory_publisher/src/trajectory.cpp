//
// Created by jalim on 29.08.18.
//

#include "trajectory_publisher/trajectory.h"

trajectory::trajectory(int type) :
  N(0),
  dt_(0.1),
  T_(10.0),
  type_(type) {

  traj_axis_ << 0.0, 0.0, 1.0;
  target_initpos << 0.0, 0.0, 0.0;

  c_x_ << 0.0, 0.0, 0.0;
  c_y_ << 0.0, 0.0, 0.0;
  c_z_ << 0.0, 0.0, 0.0;

}

trajectory::~trajectory(){

};

void trajectory::setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients, Eigen::VectorXd &z_coefficients){

  c_x_ = x_coefficients;
  c_y_ = y_coefficients;
  c_z_ = z_coefficients;

}

void trajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel){
  //Generate primitives based on current state for smooth trajectory
  c_x_(0) = pos(0);
  c_y_(0) = pos(1);
  c_z_(0) = pos(2);

  c_x_(1) = vel(0);
  c_y_(1) = vel(1);
  c_z_(1) = vel(2);

}

void trajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc){
  //Generate primitives based on current state for smooth trajectory
  c_x_(0) = pos(0);
  c_y_(0) = pos(1);
  c_z_(0) = pos(2);

  c_x_(1) = vel(0);
  c_y_(1) = vel(1);
  c_z_(1) = vel(2);

  c_x_(2) = acc(0);
  c_y_(2) = acc(1);
  c_z_(2) = acc(2);

}

Eigen::VectorXd trajectory::getCoefficients(int dim){

  switch(dim) {
    case 0 :
      return c_x_;
    case 1 :
      return c_y_;
    case 2 :
      return c_z_;
  }
}

Eigen::Vector3d trajectory::getPosition(double time){

  Eigen::Vector3d position;
  double theta;

  switch(type_) {
    case TRAJ_ZERO :
      position << 0.0, 0.0, 0.0;
      break;
    case TRAJ_POLYNOMIAL :
      position << c_x_(0) + c_x_(1) * (double) time + c_x_(2) * pow((double) time, 2) + c_x_(3) * pow((double) time, 3),
              c_y_(0) + c_y_(1) * (double) time + c_y_(2) * pow((double) time, 2) + c_y_(3) * pow((double) time, 3),
              c_z_(0) + c_z_(1) * (double) time + c_z_(2) * pow((double) time, 2) + c_z_(3) * pow((double) time, 3);
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

Eigen::Vector3d trajectory::getVelocity(double time){

  Eigen::Vector3d velocity;

  switch(type_) {
    case TRAJ_POLYNOMIAL :
      velocity << c_x_(1) + c_x_(2) * time * 2 + c_x_(3) * pow(time, 2) * 3,
              c_y_(1) + c_y_(2) * time * 2 + c_y_(3) * pow(time, 2) * 3,
              c_z_(1) + c_z_(2) * time * 2 + c_z_(3) * pow(time, 2) * 3;

    case TRAJ_CIRCLE :
      velocity = traj_omega_ * traj_axis_.cross(getPosition(time));
  }
  return velocity;

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

void trajectory::setTrajectory(int ID, double omega, Eigen::Vector3d axis, double radius,
                                        Eigen::Vector3d initpos) {
  target_trajectoryID_ = ID;
  traj_axis_ = axis;
  traj_omega_ = omega;
  traj_radius_ = radius;
  target_initpos = initpos;
}

void trajectory::setTrajectory(int ID) {
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
