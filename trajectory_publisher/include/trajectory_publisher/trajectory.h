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
 * @brief Trajectory Base Class
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef TRAJECTORY_PUBLISHER_TRAJECTORY_H
#define TRAJECTORY_PUBLISHER_TRAJECTORY_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>

class trajectory {
 private:
  int N;       // Degree of polynomial
  double dt_;  // Sampling time
  double T_;
  int type_;
  int target_trajectoryID_;
  Eigen::Vector4d c_x_, c_y_, c_z_;  // Coefficients for polynomial representation
  Eigen::Vector3d traj_axis_;
  Eigen::Vector3d traj_origin_;
  double traj_radius_, traj_omega_;

 public:
  trajectory();
  ~trajectory();
  virtual void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega) = 0;
  virtual void generatePrimitives(Eigen::Vector3d pos) = 0;
  virtual void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel) = 0;
  virtual void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk) = 0;
  virtual void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc,
                                  Eigen::Vector3d jerk) = 0;
  virtual Eigen::Vector3d getPosition(double time) = 0;
  virtual Eigen::Vector3d getVelocity(double time) = 0;
  virtual Eigen::Vector3d getAcceleration(double time) = 0;
  virtual double getsamplingTime() { return dt_; };
  virtual double getDuration() { return T_; };
  virtual nav_msgs::Path getSegment() = 0;
  virtual geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation) = 0;
};

#endif  // TRAJECTORY_PUBLISHER_TRAJECTORY_H
