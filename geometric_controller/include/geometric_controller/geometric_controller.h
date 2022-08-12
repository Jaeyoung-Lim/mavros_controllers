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
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

#include <controller_msgs/FlatTarget.h>
#include <dynamic_reconfigure/server.h>
#include <geometric_controller/GeometricControllerConfig.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include "geometric_controller/common.h"
#include "geometric_controller/control.h"

#define ERROR_QUATERNION 1
#define ERROR_GEOMETRIC 2

using namespace std;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class geometricCtrl {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber referenceSub_;
  ros::Subscriber flatreferenceSub_;
  ros::Subscriber multiDOFJointSub_;
  ros::Subscriber mavstateSub_;
  ros::Subscriber mavposeSub_, gzmavposeSub_;
  ros::Subscriber mavtwistSub_;
  ros::Subscriber yawreferenceSub_;
  ros::Publisher rotorVelPub_, angularVelPub_, target_pose_pub_;
  ros::Publisher referencePosePub_;
  ros::Publisher posehistoryPub_;
  ros::Publisher systemstatusPub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceServer ctrltriggerServ_;
  ros::ServiceServer land_service_;
  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::Time last_request_, reference_request_now_, reference_request_last_;

  string mav_name_;
  bool fail_detec_{false};
  bool feedthrough_enable_{false};
  bool ctrl_enable_{true};
  int ctrl_mode_;
  bool landing_commanded_{false};
  bool sim_enable_;
  bool velocity_yaw_;
  double kp_rot_, kd_rot_;
  double reference_request_dt_;
  double norm_thrust_const_, norm_thrust_offset_;
  double max_fb_acc_;

  mavros_msgs::State current_state_;
  mavros_msgs::CommandBool arm_cmd_;
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

  double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
  Eigen::Vector3d mavPos_, mavVel_, mavRate_;
  double mavYaw_;
  Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.8)};
  Eigen::Vector4d mavAtt_, q_des;
  Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}
  Eigen::Vector3d Kpos_, Kvel_, D_;
  double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
  int posehistory_window_;

  void pubMotorCommands();
  void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
  void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);
  void pubPoseHistory();
  void pubSystemStatus();
  void appendPoseHistory();
  void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  void targetCallback(const geometry_msgs::TwistStamped &msg);
  void flattargetCallback(const controller_msgs::FlatTarget &msg);
  void yawtargetCallback(const std_msgs::Float32 &msg);
  void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);
  void keyboardCallback(const geometry_msgs::Twist &msg);
  void cmdloopCallback(const ros::TimerEvent &event);
  void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void statusloopCallback(const ros::TimerEvent &event);
  bool ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);
  void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_acc);
  Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                  const Eigen::Vector3d &target_acc);
  Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);
  Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                Eigen::Vector4d &curr_att);

  enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state;

  template <class T>
  void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
    ros::Rate pause(hz);
    ROS_INFO_STREAM(msg);
    while (ros::ok() && !(*pred)) {
      ros::spinOnce();
      pause.sleep();
    }
  };
  geometry_msgs::Pose home_pose_;
  bool received_home_pose;
  std::shared_ptr<Control> controller_;

 public:
  void dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config, uint32_t level);
  geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~geometricCtrl();
  void getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel) {
    pos = mavPos_;
    att = mavAtt_;
    vel = mavVel_;
    angvel = mavRate_;
  };
  void getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel) {
    pos = mavPos_ - targetPos_;
    vel = mavVel_ - targetVel_;
  };
  void setBodyRateCommand(Eigen::Vector4d bodyrate_command) { cmdBodyRate_ = bodyrate_command; };
  void setFeedthrough(bool feed_through) { feedthrough_enable_ = feed_through; };
  void setDesiredAcceleration(Eigen::Vector3d &acceleration) { targetAcc_ = acceleration; };
  static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
  static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };
};

#endif
