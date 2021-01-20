/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Polynomial Trajectory
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H
#define TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H

#include "trajectory_publisher/trajectory.h"

class polynomialtrajectory : public trajectory {
 private:
  int N;       // Degree of polynomial
  double dt_;  // Sampling time
  double T_;
  Eigen::Vector4d c_x_, c_y_, c_z_;  // Coefficients for polynomial representation

 public:
  polynomialtrajectory();
  virtual ~polynomialtrajectory();
  void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega);
  void generatePrimitives(Eigen::Vector3d pos);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk);
  void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk);
  void setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients,
                       Eigen::VectorXd &z_coefficients);
  Eigen::VectorXd getCoefficients(int dim);
  Eigen::Vector3d getPosition(double time);
  Eigen::Vector3d getVelocity(double time);
  Eigen::Vector3d getAcceleration(double time);
  double getsamplingTime() { return dt_; };
  double getDuration() { return T_; };
  nav_msgs::Path getSegment();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);
};

#endif  // TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H
