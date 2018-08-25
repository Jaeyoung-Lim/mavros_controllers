//
// Created by jalim on 05.05.18.
//

#include "trajectory_publisher/trajectoryPublisher.h"

using namespace std;
using namespace Eigen;
trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private){

  trajectoryPub_ = nh_.advertise<nav_msgs::Path>("reference/trajectory", 1);
  referencePub_ = nh_.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
  trajloop_timer_ = nh_.createTimer(ros::Duration(1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_ = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  nh_.param<double>("/trajectory_publisher/initpos_x", init_pos_x_, 0.0);
  nh_.param<double>("/trajectory_publisher/initpos_x", init_pos_y_, 0.0);
  nh_.param<double>("/trajectory_publisher/initpos_x", init_pos_z_, 1.0);
  nh_.param<double>("/trajectory_publisher/updaterate", controlUpdate_dt_, 0.01);
  nh_.param<int>("/trajectory_publisher/trajectoryID", target_trajectoryID_, 0);


  traj_axis_ << 0.0, 0.0, 1.0;
  p_targ << 0.0, 0.0, 0.0;
  v_targ << 0.0, 0.0, 0.0;
  target_initpos << init_pos_x_, init_pos_y_, init_pos_z_;
}

void trajectoryPublisher::setTrajectory(int ID) {
  double radius = 0, omega = 0;
  Eigen::Vector3d axis, initpos;

  switch (ID) {
    case TRAJ_STATIONARY: //stationary trajectory
      omega = 0.0;
      radius = 0.0;
      axis << 0.0, 0.0, 1.0;
      initpos << 0.0, 0.0, 1.0;
      break;
    case TRAJ_CIRCLE: //circular trajectory
      omega = 1.0;
      radius = 2.0;
      axis << 0.0, 0.0, 1.0;
      initpos << 0.0, radius, 0.0;
      break;
    case TRAJ_LAMNISCATE: //Lemniscate of Genoro
      omega = 1.0;
      radius = 2.0;
      axis << 0.0, 0.0, 1.0;
      initpos << 0.0, radius, 0.0;
      break;
  }
  setTrajectory(ID, omega, axis, radius, initpos);
}

void trajectoryPublisher::setTrajectory(int ID, double omega, Eigen::Vector3d axis, double radius,
                                        Eigen::Vector3d initpos) {
  target_trajectoryID_ = ID;
  traj_axis_ = axis;
  traj_omega_ = omega;
  traj_radius_ = radius;
  target_initpos = initpos;
}

void trajectoryPublisher::setTrajectoryTheta(double in) {
  theta_ = in;
}

void trajectoryPublisher::moveReference() {
  curr_time_ = ros::Time::now();
  trigger_time_ = (curr_time_ - start_time_).toSec();

  if(mode_ == MODE_PRIMITIVES){
    //TODO: Play reference trajectory based on time
//    p_targ << a0x + a1x*trigger_time_ + a2x*trigger_time_^2 + a3x*trigger)time_^3,
//              a0y + a1y*trigger_time_ + a2y*trigger_time_^2 + a3y*trigger)time_^3,
//              a0z + a1z*trigger_time_ + a2z*trigger_time_^2 + a3z*trigger)time_^3;
//    v_targ << a1 + 2*a2*trigger_time_ + 3*a3*trigger)time_^2,
//              a1 + 2*a2*trigger_time_ + 3*a3*trigger)time_^2,
//              a1 + 2*a2*trigger_time_ + 3*a3*trigger)time_^2;
  }
  else if(mode_ == MODE_REFERENCE){
    theta_ = traj_omega_* trigger_time_;

    if (target_trajectoryID_ == 0) { //Stationary
      p_targ = target_initpos;
      v_targ.setZero();
    } else if (target_trajectoryID_ == 1) { //Circular trajectory
      p_targ = std::cos(theta_) * target_initpos
               + std::sin(theta_) * traj_axis_.cross(target_initpos)
               + (1 - std::cos(theta_)) * traj_axis_.dot(target_initpos) * traj_axis_;
      v_targ = traj_omega_ * traj_axis_.cross(p_targ);
    } else if (target_trajectoryID_ == 2) { //Lemniscate of Genero
      p_targ = std::cos(theta_) * target_initpos
               + std::sin(theta_) * std::cos(theta_) * traj_axis_.cross(target_initpos)
               + (1 - std::cos(theta_)) * traj_axis_.dot(target_initpos) * traj_axis_;
      v_targ = traj_omega_ * traj_axis_.cross(p_targ); //TODO: This is wrong
    }
  }
}

Eigen::Vector3d trajectoryPublisher::getTargetPosition(){
  return p_targ;
}

double trajectoryPublisher::getTrajectoryOmega(){
  return traj_omega_;
}

double trajectoryPublisher::getTrajectoryUpdateRate(){
  return controlUpdate_dt_;
}

geometry_msgs::PoseStamped trajectoryPublisher::vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation){
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

void trajectoryPublisher::pubrefTrajectory(){

int N = (int) 2*3.141592 / this->getTrajectoryOmega() / this->getTrajectoryUpdateRate(); //Resolution of the trajectory to be published
double theta;
Eigen::Vector3d targetPosition;
Eigen::Vector4d targetOrientation;
targetOrientation << 1.0, 0.0, 0.0, 0.0;
geometry_msgs::PoseStamped targetPoseStamped;

refTrajectory_.header.stamp = ros::Time::now();
refTrajectory_.header.frame_id = 1;

for(int i = 0 ; i < N ; i++){
  theta = 2 * 3.141592 *((double) i / (double) N);
  this->setTrajectoryTheta(theta);
  this->moveReference();
  targetPosition = this->getTargetPosition();
  targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
  refTrajectory_.poses.push_back(targetPoseStamped);
}
}

void trajectoryPublisher::pubrefState(){
  refState_.header.stamp = ros::Time::now();
  refState_.header.frame_id = "map";
  refState_.twist.angular.x = p_targ(0);
  refState_.twist.angular.y = p_targ(1);
  refState_.twist.angular.z = p_targ(2);
  refState_.twist.linear.x = v_targ(0);
  refState_.twist.linear.y = v_targ(1);
  refState_.twist.linear.z = v_targ(2);
  referencePub_.publish(refState_);
}

void trajectoryPublisher::loopCallback(const ros::TimerEvent& event){
  //Slow Loop publishing trajectory information
  trajectoryPub_.publish(refTrajectory_);
}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event){
  //Fast Loop publishing reference states
  moveReference();
  pubrefState();
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){
  unsigned char mode = req.data;
  start_time_ = ros::Time::now();
  //TODO: Trajectory triggering should not be done by changing modes
  switch(mode){
    case 1 :
      target_trajectoryID_ = TRAJ_CIRCLE;
      break;
    case 2 :
      target_trajectoryID_ = TRAJ_LAMNISCATE;
      break;
  }
  res.success = true;
  res.message = "trajectory triggered";
}

void trajectoryPublisher::trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_message) {

  if (segments_message.segments.empty()) {
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  } else {
    ROS_INFO("Trajectory sampler: received %lu waypoints", segments_message.segments.size());
  }

  start_time_ = ros::Time::now();
  //TODO: Read polynomial coefficients
}