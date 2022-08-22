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
 * @brief Trajectory Publisher
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#include <stdio.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavconn/mavlink_dialect.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include "controller_msgs/FlatTarget.h"
#include "trajectory_publisher/polynomialtrajectory.h"
#include "trajectory_publisher/shapetrajectory.h"
#include "trajectory_publisher/trajectory.h"

#define REF_TWIST 8
#define REF_SETPOINTRAW 16

using namespace std;
using namespace Eigen;
class trajectoryPublisher {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher trajectoryPub_;
  ros::Publisher referencePub_;
  ros::Publisher flatreferencePub_;
  ros::Publisher rawreferencePub_;
  ros::Publisher global_rawreferencePub_;
  std::vector<ros::Publisher> primitivePub_;
  ros::Subscriber motionselectorSub_;
  ros::Subscriber mavposeSub_;
  ros::Subscriber mavtwistSub_;
  ros::Subscriber mavstate_sub_;
  ros::ServiceServer trajtriggerServ_;
  ros::Timer trajloop_timer_;
  ros::Timer refloop_timer_;
  ros::Time start_time_, curr_time_;

  nav_msgs::Path refTrajectory_;
  nav_msgs::Path primTrajectory_;
  mavros_msgs::State current_state_;

  int trajectory_type_;
  Eigen::Vector3d p_targ, v_targ, a_targ;
  Eigen::Vector3d p_mav_, v_mav_;
  Eigen::Vector3d shape_origin_, shape_axis_;
  double shape_omega_ = 0;
  double theta_ = 0.0;
  double controlUpdate_dt_;
  double primitive_duration_;
  double trigger_time_;
  double init_pos_x_, init_pos_y_, init_pos_z_;
  double max_jerk_;
  int pubreference_type_;
  int num_primitives_;
  int motion_selector_;

  std::vector<std::shared_ptr<trajectory>> motionPrimitives_;
  std::vector<Eigen::Vector3d> inputs_;

 public:
  trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void updateReference();
  void pubrefTrajectory(int selector);
  void pubprimitiveTrajectory();
  void pubrefState();
  void pubflatrefState();
  void pubrefSetpointRaw();
  void pubrefSetpointRawGlobal();
  void initializePrimitives(int type);
  void updatePrimitives();
  void loopCallback(const ros::TimerEvent& event);
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  void motionselectorCallback(const std_msgs::Int32& selector);
  void mavposeCallback(const geometry_msgs::PoseStamped& msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped& msg);
  void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
};

#endif
