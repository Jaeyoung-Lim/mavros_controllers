//
// Created by jalim on 11.10.18.
//

#ifndef TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H
#define TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H

#include "trajectory_publisher/trajectory.h"

#define TRAJ_ZERO 0
#define TRAJ_CIRCLE 1
#define TRAJ_LAMNISCATE 2
#define TRAJ_STATIONARY 3

class shapetrajectory : public trajectory {
private:
  int type_;
  int N;
  double dt_;
  double T_;
  Eigen::Vector3d traj_axis_;
  Eigen::Vector3d traj_origin_;
  Eigen::Vector3d traj_radial_;
  double traj_radius_, traj_omega_;

public:
  shapetrajectory(int type);
  virtual ~shapetrajectory();
  void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega);
  void generatePrimitives(Eigen::Vector3d pos);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk);
  Eigen::Vector3d getPosition(double time);
  Eigen::Vector3d getVelocity(double time);
  Eigen::Vector3d getAcceleration(double time);
  double getsamplingTime(){return dt_;};
  double getDuration(){ return T_;};
  nav_msgs::Path getSegment();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);
};
#endif //TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H
