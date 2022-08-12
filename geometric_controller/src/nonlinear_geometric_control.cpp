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

#include "geometric_controller/nonlinear_geometric_control.h"

NonlinearGeometricControl::NonlinearGeometricControl(double attctrl_tau) : Control() { attctrl_tau_ = attctrl_tau; }

NonlinearGeometricControl::~NonlinearGeometricControl() {}

void NonlinearGeometricControl::Update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att,
                                       const Eigen::Vector3d &ref_acc, const Eigen::Vector3d &ref_jerk) {
  // Geometric attitude controller
  // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
  // of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
  // The original paper inputs moment commands, but for offboard control, angular rate commands are sent

  Eigen::Vector4d ratecmd;
  Eigen::Matrix3d rotmat;    // Rotation matrix of current attitude
  Eigen::Matrix3d rotmat_d;  // Rotation matrix of desired attitude
  Eigen::Vector3d error_att;

  rotmat = quat2RotMatrix(curr_att);
  rotmat_d = quat2RotMatrix(ref_att);

  error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
  desired_rate_ = (2.0 / attctrl_tau_) * error_att;
  const Eigen::Vector3d zb = rotmat.col(2);
  desired_thrust_(0) = 0.0;
  desired_thrust_(1) = 0.0;
  desired_thrust_(2) = ref_acc.dot(zb);
}
