//
// Created by jalim on 11.10.18.
//

#ifndef TRAJECTORY_PUBLISHER_FLIPTRAJECTORY_H
#define TRAJECTORY_PUBLISHER_FLIPTRAJECTORY_H

#include "trajectory_publisher/trajectory.h"

#define TRAJ_FLIP 3

class fliptrajectory : public trajectory {
private:
  int type_;
  int N;
  double dt_;
  double T_;
  Eigen::Vector3d traj_axis_;
  Eigen::Vector3d traj_origin_;

public:
  fliptrajectory(int type);
  virtual ~fliptrajectory();
  void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega);
  void generatePrimitives(Eigen::Vector3d pos);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk);
  Eigen::Vector3d getPosition(double time);
  Eigen::Vector3d getVelocity(double time);
  double getsamplingTime(){return dt_;};
  double getDuration(){ return T_;};
  nav_msgs::Path getSegment();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);
};
#endif //TRAJECTORY_PUBLISHER_FLIPTRAJECTORY_H
