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

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <math.h>

using namespace std;
using namespace Eigen;

class Estimator
{
  private:
    //Parameters
    double R_;

    // thermal state vector is defined as below
    // W_th : Thermal Strength
    // R_th : Thermal Radius
    // x    : Thermal Center x
    // y    : Thermal Cetner y
    Eigen::Vector4d thermal_state_;
    Eigen::Matrix4d thermal_state_covariance_;

    Eigen::Matrix4d F_;
    Eigen::Matrix4d Q_;

  protected:
    void ObservationProcess(Eigen::Vector4d state, Eigen::Vector4d &H, double &predicted_measurement);
    void PriorUpdate(const Eigen::Vector4d &state, const Eigen::Matrix4d &covariance, Eigen::Vector4d &predicted_state, Eigen::Matrix4d &predicted_covariance);
    void MeasurementUpdate(const Eigen::Vector4d &state, const Eigen::Matrix4d &covariance, Eigen::Vector4d &updated_state, Eigen::Matrix4d &updated_covariance, double measurement);

  public:
    Estimator();
    virtual ~Estimator();
    void UpdateState(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector4d attitude);
    void reset();
};


#endif