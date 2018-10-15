//
// Created by jalim on 11.10.18.
//

#ifndef TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H
#define TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H

#include "trajectory_publisher/trajectory.h"


class polynomialtrajectory : public trajectory {
private:
  int N; //Degree of polynomial
  double dt_; //Sampling time
  double T_;
  Eigen::Vector4d c_x_, c_y_, c_z_; //Coefficients for polynomial representation

public:
  polynomialtrajectory();
  virtual ~polynomialtrajectory();
  void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega);
  void generatePrimitives(Eigen::Vector3d pos);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk);
  void setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients, Eigen::VectorXd &z_coefficients);
  Eigen::VectorXd getCoefficients(int dim);
  Eigen::Vector3d getPosition(double time);
  Eigen::Vector3d getVelocity(double time);
  Eigen::Vector3d getAcceleration(double time);
  double getsamplingTime(){return dt_;};
  double getDuration(){ return T_;};
  nav_msgs::Path getSegment();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);

};


#endif //TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H
