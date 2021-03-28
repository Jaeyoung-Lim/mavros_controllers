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
 * @brief Parameter Estimator Class
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "parameter_estimator/estimator.h"

using namespace Eigen;
using namespace std;
//Constructor
Estimator::Estimator() {

  F_ = Eigen::Matrix4d::Identity();   //Process Dynamics

  //TODO: Read noise configurations from parameters
  R_ = 0.01;
  Eigen::Vector4d Q_vector;
  Q_vector << 1.0, 1.0, 1.0, 1.0;
  Q_ = Q_vector.asDiagonal();

  thermal_state_ << 100.0, 10.0, 0.0, 0.0;
}

Estimator::~Estimator() {
  //Destructor
}

void Estimator::UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector4d attitude) {
  //Update States
  Eigen::Vector3d current_position = position;
  Eigen::Vector3d current_velocity = velocity;

  // PriorUpdate(thermal_state_, thermal_state_covariance_, predicted_thermal_state, predicted_thermal_covariance);

  //MeaurementUpdate
  // MeasurementUpdate(predicted_thermal_state, predicted_thermal_covariance, thermal_state_, thermal_state_covariance_, netto_vario);
}

void Estimator::reset() {
  thermal_state_ = Eigen::Vector4d::Zero();


}

void Estimator::PriorUpdate(const Eigen::Vector4d &state, const Eigen::Matrix4d &covariance, Eigen::Vector4d &predicted_state, Eigen::Matrix4d &predicted_covariance) {
  predicted_state = state;
  predicted_covariance = covariance + Q_;
}

void Estimator::MeasurementUpdate(const Eigen::Vector4d &state, const Eigen::Matrix4d &covariance, Eigen::Vector4d &updated_state, Eigen::Matrix4d &updated_covariance, double measurement) {
  ///Compute Kalman gains
  double predicted_measurement;
  Eigen::Vector4d H;
  ObservationProcess(thermal_state_, H, predicted_measurement);
  double den = H.transpose() * covariance * H + R_;
  Eigen::Vector4d K_kalman_ = covariance * H / den;

  //Update
  //TODO: Publish innovations
  updated_state = state + K_kalman_ * (measurement - predicted_measurement);
  updated_covariance = (Eigen::Matrix4d::Identity() - K_kalman_ * H.transpose())*covariance + Q_;
}

void Estimator::ObservationProcess(Eigen::Vector4d state, Eigen::Vector4d &H, double &predicted_measurement){
  const double W_th = state(0);
  const double R_th = state(1);
  const double x = state(2);
  const double y = state(3);

  predicted_measurement = W_th * std::exp( - (x*x + y*y)/(R_th*R_th));

  H(0) = std::exp(- (x*x + y*y)/(R_th*R_th));
  H(1) = 2 * W_th * (x*x + y*y) * H(0) / (std::pow(R_th, 3));
  H(2) = 2 * W_th * x * H(0) / (std::pow(R_th, 2));
  H(3) = 2 * W_th * y * H(0) / (std::pow(R_th, 2));
}
