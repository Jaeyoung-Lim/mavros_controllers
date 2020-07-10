//
// Created by jalim on 11.10.18.
//

#include "trajectory_publisher/polynomialtrajectory.h"

polynomialtrajectory::polynomialtrajectory() : N(0), dt_(0.1), T_(1.0) {
  c_x_ << 0.0, 0.0, 0.0, 0.0;
  c_y_ << 0.0, 0.0, 0.0, 0.0;
  c_z_ << 0.0, 0.0, 0.0, 0.0;
};

polynomialtrajectory::~polynomialtrajectory() {}

void polynomialtrajectory::setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients,
                                           Eigen::VectorXd &z_coefficients) {
  c_x_ = x_coefficients;
  c_y_ = y_coefficients;
  c_z_ = z_coefficients;
}

void polynomialtrajectory::initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega) {
  // Generate primitives based on current state for smooth trajectory
  c_x_(0) = pos(0);
  c_y_(0) = pos(1);
  c_z_(0) = pos(2);
}

void polynomialtrajectory::generatePrimitives(Eigen::Vector3d pos) {
  // Generate primitives based on current state for smooth trajectory
  c_x_(0) = pos(0);
  c_y_(0) = pos(1);
  c_z_(0) = pos(2);
}

void polynomialtrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel) {
  // Generate primitives based on current state for smooth trajectory
  c_x_(0) = pos(0);
  c_y_(0) = pos(1);
  c_z_(0) = pos(2);

  c_x_(1) = vel(0);
  c_y_(1) = vel(1);
  c_z_(1) = vel(2);
}

void polynomialtrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk) {
  // Generate primitives based on current state for smooth trajectory
  c_x_(0) = pos(0);
  c_y_(0) = pos(1);
  c_z_(0) = pos(2);

  c_x_(1) = vel(0);
  c_y_(1) = vel(1);
  c_z_(1) = vel(2);

  c_x_(2) = 0.0;  // Acceleration is neglected
  c_y_(2) = 0.0;
  c_z_(2) = 0.0;

  c_x_(3) = jerk(0);
  c_y_(3) = jerk(1);
  c_z_(3) = jerk(2);
}

void polynomialtrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc,
                                              Eigen::Vector3d jerk) {
  // Generate primitives based on current state for smooth trajectory
  c_x_(0) = pos(0);
  c_y_(0) = pos(1);
  c_z_(0) = pos(2);

  c_x_(1) = vel(0);
  c_y_(1) = vel(1);
  c_z_(1) = vel(2);

  c_x_(2) = acc(0);
  c_y_(2) = acc(1);
  c_z_(2) = acc(2);

  c_x_(3) = jerk(0);
  c_y_(3) = jerk(1);
  c_z_(3) = jerk(2);
}

Eigen::VectorXd polynomialtrajectory::getCoefficients(int dim) {
  switch (dim) {
    case 0:
      return c_x_;
    case 1:
      return c_y_;
    case 2:
      return c_z_;
  }
}

Eigen::Vector3d polynomialtrajectory::getPosition(double time) {
  Eigen::Vector3d position;
  position << c_x_(0) + c_x_(1) * time + c_x_(2) * pow(time, 2) + c_x_(3) * pow(time, 3),
      c_y_(0) + c_y_(1) * time + c_y_(2) * pow(time, 2) + c_y_(3) * pow(time, 3),
      c_z_(0) + c_z_(1) * time + c_z_(2) * pow(time, 2) + c_z_(3) * pow(time, 3);

  return position;
}

Eigen::Vector3d polynomialtrajectory::getVelocity(double time) {
  Eigen::Vector3d velocity;
  velocity << c_x_(1) + c_x_(2) * time * 2 + c_x_(3) * pow(time, 2) * 3,
      c_y_(1) + c_y_(2) * time * 2 + c_y_(3) * pow(time, 2) * 3,
      c_z_(1) + c_z_(2) * time * 2 + c_z_(3) * pow(time, 2) * 3;

  return velocity;
}

Eigen::Vector3d polynomialtrajectory::getAcceleration(double time) {
  Eigen::Vector3d acceleration;
  acceleration << c_x_(2) * 2 + c_x_(3) * time * 6, c_y_(2) * 2 + c_y_(3) * time * 6, c_z_(2) * 2 + c_z_(3) * time * 6;

  return acceleration;
}

nav_msgs::Path polynomialtrajectory::getSegment() {
  Eigen::Vector3d targetPosition;
  Eigen::Vector4d targetOrientation;
  nav_msgs::Path segment;

  targetOrientation << 1.0, 0.0, 0.0, 0.0;
  geometry_msgs::PoseStamped targetPoseStamped;

  for (double t = 0; t < this->getDuration(); t += this->getsamplingTime()) {
    targetPosition = this->getPosition(t);
    targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
    segment.poses.push_back(targetPoseStamped);
  }
  return segment;
}

geometry_msgs::PoseStamped polynomialtrajectory::vector3d2PoseStampedMsg(Eigen::Vector3d position,
                                                                         Eigen::Vector4d orientation) {
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