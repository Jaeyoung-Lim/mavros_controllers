/****************************************************************************
 *
 *   Copyright (c) 2018-2022 Jaeyoung Lim. All rights reserved.
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
 * @brief Controller base class
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "geometric_controller/jerk_tracking_control.h"

JerkTrackingControl::JerkTrackingControl() : Control() {}

JerkTrackingControl::~JerkTrackingControl() {}

void JerkTrackingControl::Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                 const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
  // Jerk feedforward control
  // Based on: Lopez, Brett Thomas. Low-latency trajectory planning for high-speed navigation in unknown environments.
  // Diss. Massachusetts Institute of Technology, 2016.
  // Feedforward control from Lopez(2016)

  double dt_ = 0.01;
  // Numerical differentiation to calculate jerk_fb
  const Eigen::Vector3d jerk_fb = (ref_acc - last_ref_acc_) / dt_;
  const Eigen::Vector3d jerk_des = ref_jerk + jerk_fb;
  const Eigen::Matrix3d R = quat2RotMatrix(curr_att);
  const Eigen::Vector3d zb = R.col(2);

  const Eigen::Vector3d jerk_vector =
      jerk_des / ref_acc.norm() - ref_acc * ref_acc.dot(jerk_des) / std::pow(ref_acc.norm(), 3);
  const Eigen::Vector4d jerk_vector4d(0.0, jerk_vector(0), jerk_vector(1), jerk_vector(2));

  Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
  const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
  const Eigen::Vector4d qd = quatMultiplication(q_inv, ref_att);

  const Eigen::Vector4d qd_star(qd(0), -qd(1), -qd(2), -qd(3));

  const Eigen::Vector4d ratecmd_pre = quatMultiplication(quatMultiplication(qd_star, jerk_vector4d), qd);

  Eigen::Vector4d ratecmd;
  desired_rate_(0) = ratecmd_pre(2);  // TODO: Are the coordinate systems consistent?
  desired_rate_(1) = (-1.0) * ratecmd_pre(1);
  desired_rate_(2) = 0.0;
  desired_thrust_(0) = 0.0;
  desired_thrust_(1) = 0.0;
  desired_thrust_(2) = ref_acc.dot(zb);  // Calculate thrust
  last_ref_acc_ = ref_acc;
  return;
}
