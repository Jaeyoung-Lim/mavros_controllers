//  May/2018, ETHZ, Jaeyoung Lim, jalim@ethz.ch

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>

#include "trajectory_publisher/trajectory.h"

using namespace std;
using namespace Eigen;
class trajectoryPublisher
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher trajectoryPub_;
  ros::Publisher referencePub_;
  ros::Publisher primitivePub_;
  ros::Subscriber motionselectorSub_;
  ros::Subscriber mavposeSub_;
  ros::Subscriber mavtwistSub_;
  ros::ServiceServer trajtriggerServ_;
  ros::Timer trajloop_timer_;
  ros::Timer refloop_timer_;
  ros::Time start_time_, curr_time_;

  nav_msgs::Path refTrajectory_, primTrajectory_;
  geometry_msgs::TwistStamped refState_;

  int counter;
  int mode_;
  Eigen::Vector3d p_targ, v_targ;
  Eigen::Vector3d p_mav_, v_mav_;
  double theta_ = 0.0;
  double controlUpdate_dt_;
  double primitive_duration_;
  double trigger_time_;
  double init_pos_x_, init_pos_y_, init_pos_z_;
  int num_primitives_;
  int motion_selector_;

  std::vector<trajectory> motionPrimitives_;
  std::vector<Eigen::Vector3d> inputs_;


public:
  trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void setTrajectory(int ID);
  void setTrajectory(int ID, double omega, Eigen::Vector3d axis, double radius, Eigen::Vector3d initpos);
  void setTrajectoryTheta(double in);
  double getTrajectoryOmega();
  double getTrajectoryUpdateRate();
  void updateReference();
  void pubrefTrajectory(int selector);
  void pubprimitiveTrajectory();
  void pubrefState();
  void initializePrimitives();
  void updatePrimitives();
  Eigen::Vector3d getTargetPosition();
  void loopCallback(const ros::TimerEvent& event);
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_message);
  void motionselectorCallback(const std_msgs::Int32& selector);
  void mavposeCallback(const geometry_msgs::PoseStamped& msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped& msg);

  };


#endif
