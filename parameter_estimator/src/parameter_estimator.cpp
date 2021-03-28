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
 * @brief Parameter Estimator Node Class
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "parameter_estimator/parameter_estimator.h"


using namespace Eigen;
using namespace std;
//Constructor
ParameterEstimator::ParameterEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &ParameterEstimator::cmdloopCallback, this); // Define timer for constant loop rate
  mavpose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, &ParameterEstimator::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  mavtwist_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 1, &ParameterEstimator::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  //TODO: Add subscription to rotor velocities

  estimator_ = std::make_shared<Estimator>();
}
ParameterEstimator::~ParameterEstimator() {
  //Destructor
}

void ParameterEstimator::cmdloopCallback(const ros::TimerEvent& event){
  estimator_->UpdateState(mav_pos_, mav_vel_, mav_att_);
}
