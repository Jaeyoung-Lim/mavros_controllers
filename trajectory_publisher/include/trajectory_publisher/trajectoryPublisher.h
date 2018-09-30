//  May/2018, ETHZ, Jaeyoung Lim, jalim@ethz.ch

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#define TRAJ_STATIONARY 0
#define TRAJ_CIRCLE 1
#define TRAJ_LAMNISCATE 2

#define MODE_PRIMITIVES 1
#define MODE_REFERENCE 2

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
  ros::Subscriber motionselectorSub_;
  ros::ServiceServer trajtriggerServ_;
  ros::Timer trajloop_timer_;
  ros::Timer refloop_timer_;
  ros::Time start_time_, curr_time_;

  nav_msgs::Path refTrajectory_;
  geometry_msgs::TwistStamped refState_;

  int counter;
  int mode_;
  Eigen::Vector3d target_initpos;
  Eigen::Vector3d traj_axis_;
  Eigen::Vector3d p_targ, v_targ;
  double traj_radius_, traj_omega_;
  double theta_ = 0.0;
  double controlUpdate_dt_;
  double trigger_time_;
  double init_pos_x_, init_pos_y_, init_pos_z_;
  int target_trajectoryID_;
  int num_primitives_;
  int motion_selector_;

  std::vector<trajectory> motionPrimitives_;


public:
  trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void setTrajectory(int ID);
  void setTrajectory(int ID, double omega, Eigen::Vector3d axis, double radius, Eigen::Vector3d initpos);
  void setTrajectoryTheta(double in);
  double getTrajectoryOmega();
  double getTrajectoryUpdateRate();
  void moveReference();
  void pubrefTrajectory();
  void pubrefState();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);
  Eigen::Vector3d getTargetPosition();
  void loopCallback(const ros::TimerEvent& event);
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_message);
  void motionselectorCallback(const std_msgs::Int32& selector);


  };


#endif
