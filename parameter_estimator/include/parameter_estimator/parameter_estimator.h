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

#ifndef PARAMETER_ESTIMATOR_H
#define PARAMETER_ESTIMATOR_H

#include "parameter_estimator/estimator.h"

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>


using namespace std;
using namespace Eigen;

class ParameterEstimator
{
  public:
    ParameterEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ ParameterEstimator();
    
    std::shared_ptr<Estimator> estimator_;

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber mavpose_sub_;
    ros::Subscriber mavtwist_sub_;

    ros::Timer cmdloop_timer_;

    Eigen::Vector3d mav_pos_;
    Eigen::Vector4d mav_att_;
    Eigen::Vector3d mav_vel_;
    Eigen::Vector3d mav_rate_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void mavposeCallback(const geometry_msgs::PoseStamped& msg) {
      mav_pos_(0) = msg.pose.position.x;
      mav_pos_(1) = msg.pose.position.y;
      mav_pos_(2) = msg.pose.position.z;
      mav_att_(0) = msg.pose.orientation.w;
      mav_att_(1) = msg.pose.orientation.x;
      mav_att_(2) = msg.pose.orientation.y;
      mav_att_(3) = msg.pose.orientation.z;
    };
    void mavtwistCallback(const geometry_msgs::TwistStamped& msg) {
      mav_vel_(0) = msg.twist.linear.x;
      mav_vel_(1) = msg.twist.linear.y;
      mav_vel_(2) = msg.twist.linear.z;
      mav_rate_(0) = msg.twist.angular.x;
      mav_rate_(1) = msg.twist.angular.y;
      mav_rate_(2) = msg.twist.angular.z;
    };
};


#endif
