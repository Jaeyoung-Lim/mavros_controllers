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
 * @brief Common library
 *
 * Common library for geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>

const size_t nStates = 10;
const size_t nControls = 4;

typedef Eigen::Matrix<double, nStates, 1> state_vector_t;
typedef Eigen::Matrix<double, nControls, 1> control_vector_t;

typedef Eigen::Matrix<double, nStates, nStates> state_matrix_t;
typedef Eigen::Matrix<double, nControls, nControls> control_matrix_t;
typedef Eigen::Matrix<double, nStates, nControls> control_gain_matrix_t;


static Eigen::Matrix3d matrix_hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  // Sanity checks on M
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}

static Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m) {
  Eigen::Vector3d v;
  // TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
  Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
  return ev3;
}

Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
      2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

state_matrix_t A_quadrotor(const state_vector_t& x, const control_vector_t& u);
control_gain_matrix_t B_quadrotor(const state_vector_t& x, const control_vector_t& u);

state_matrix_t A_quadrotor(const state_vector_t& x, const control_vector_t& u)
{
    double wx = u(0);
    double wy = u(1);
    double wz = u(2);
    double norm_thrust  = u(3);
    Eigen::Quaternion<double> q(x(3),x(4),x(5),x(6));
    Eigen::Matrix<double,4,4> q_partial_correction;
    Eigen::Matrix<double,4,4> dqdot_dq;
    Eigen::Matrix<double,3,4> dvdot_dq;
    Eigen::Matrix<double,4,1> q_vec;

    q_vec(0) = q.w();
    q_vec(1) = q.x();
    q_vec(2) = q.y();
    q_vec(3) = q.z();

    state_matrix_t A;
    A.setZero();

    //Position
    A(0,7) = 1;
    A(1,8)= 1;
    A(2,9)= 1;
    Eigen::Matrix<double,4,4> Identity;

    //Orientation
    q_partial_correction = pow(q.norm(),-1.0)*(Identity.Identity() - pow(q.norm(),-2.0)*(q_vec * q_vec.transpose()));

    dqdot_dq << 0, -wx, -wy, -wz,
                wx, 0, wz, -wy,
                wy, -wz, 0, wx,
                wz, wy, -wx, 0;
    dqdot_dq = 0.5*dqdot_dq*q_partial_correction;

    A(3,3) = dqdot_dq(0,0);
    A(3,4) = dqdot_dq(0,1);
    A(3,5) = dqdot_dq(0,2);
    A(3,6) = dqdot_dq(0,3);

    A(4,3) = dqdot_dq(1,0);
    A(4,4) = dqdot_dq(1,1);
    A(4,5) = dqdot_dq(1,2);
    A(4,6) = dqdot_dq(1,3);

    A(5,3) = dqdot_dq(2,0);
    A(5,4) = dqdot_dq(2,1);
    A(5,5) = dqdot_dq(2,2);
    A(5,6) = dqdot_dq(2,3);

    A(6,3) = dqdot_dq(3,0);
    A(6,4) = dqdot_dq(3,1);
    A(6,5) = dqdot_dq(3,2);
    A(6,6) = dqdot_dq(3,3);


    //Velocity
    dvdot_dq << q.y(),  q.z(),  q.w(), q.x(),
              -q.x(), -q.w(),  q.z(), q.y(),
               q.w(), -q.x(), -q.y(), q.z();

    dvdot_dq = 2*norm_thrust*dvdot_dq*q_partial_correction;

    A(7,3) = dvdot_dq(0,0);
    A(7,4) = dvdot_dq(0,1);
    A(7,5) = dvdot_dq(0,2);
    A(7,6) = dvdot_dq(0,3);

    A(8,3) = dvdot_dq(1,0);
    A(8,4) = dvdot_dq(1,1);
    A(8,5) = dvdot_dq(1,2);
    A(8,6) = dvdot_dq(1,3);

    A(9,3) = dvdot_dq(2,0);
    A(9,4) = dvdot_dq(2,1);
    A(9,5) = dvdot_dq(2,2);
    A(9,6) = dvdot_dq(2,3);

    return A;
}

control_gain_matrix_t B_quadrotor(const state_vector_t& x, const control_vector_t& u)
{
   double wx = u(0);
   double wy = u(1);
   double wz = u(2);
   double norm_thrust  = u(3);
   Eigen::Quaternion<double> q(x(3),x(4),x(5),x(6));
   Eigen::Matrix<double,3,1> dvdot_dc;
   Eigen::Matrix<double,4,3> dqdot_dw;

   control_gain_matrix_t B;
   B.setZero();

   dvdot_dc << 2*(q.w()*q.y() + q.x()*q.z()),
               2*(q.y()*q.z() - q.w()*q.x()),
               pow(q.w(),2) - pow(q.x(),2) - pow(q.y(),2) + pow(q.z(),2);

   B(7,3) = dvdot_dc(0);
   B(8,3) = dvdot_dc(1);
   B(9,3) = dvdot_dc(2);

   dqdot_dw << -q.x(), -q.y(), -q.z(),
                q.w(), -q.z(),  q.y(),
                q.z(),  q.w(), -q.x(),
               -q.y(),  q.x(),  q.w();

   dqdot_dw = 0.5*dqdot_dw;

   B(3,0) = dqdot_dw(0,0);
   B(3,1) = dqdot_dw(0,1);
   B(3,2) = dqdot_dw(0,2);

   B(4,0) = dqdot_dw(1,0);
   B(4,1) = dqdot_dw(1,1);
   B(4,2) = dqdot_dw(1,2);

   B(5,0) = dqdot_dw(2,0);
   B(5,1) = dqdot_dw(2,1);
   B(5,2) = dqdot_dw(2,2);

   B(6,0) = dqdot_dw(3,0);
   B(6,1) = dqdot_dw(3,1);
   B(6,2) = dqdot_dw(3,2);

   return B;
}

/* Itereation method for discrete model */
bool solveRiccatiIterationC(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                            const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                            Eigen::MatrixXd &P, const double dt = 0.001,
                            const double &tolerance = 1.E-5,
                            const uint iter_max = 100000);

bool solveRiccatiIterationC(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                            const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                            Eigen::MatrixXd &P, const double dt,
                            const double &tolerance,
                            const uint iter_max) {
  P = Q; // initialize

  Eigen::MatrixXd P_next;

  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd Rinv = R.inverse();

  double diff;
  for (uint i = 0; i < iter_max; ++i) {
    P_next = P + (P * A + AT * P - P * B * Rinv * BT * P + Q) * dt;
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if (diff < tolerance) {
      std::cout << "iteration mumber = " << i << std::endl;
      return true;
    }
  
  }
  
  return false; // over iteration limit

  }

#endif
