//
// Created by jalim on 05.05.18.
//

#include "trajectory_publisher/trajectoryPublisher.h"

using namespace std;
using namespace Eigen;
trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  motion_selector_(0) {

  trajectoryPub_ = nh_.advertise<nav_msgs::Path>("/trajectory_publisher/trajectory", 1);
  referencePub_ = nh_.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
  motionselectorSub_ = nh_.subscribe("/trajectory_publisher/motionselector", 1, &trajectoryPublisher::motionselectorCallback, this,ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &trajectoryPublisher::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity", 1, &trajectoryPublisher::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());

  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_ = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  nh_.param<double>("/trajectory_publisher/initpos_x", init_pos_x_, 0.0);
  nh_.param<double>("/trajectory_publisher/initpos_y", init_pos_y_, 0.0);
  nh_.param<double>("/trajectory_publisher/initpos_z", init_pos_z_, 1.0);
  nh_.param<double>("/trajectory_publisher/updaterate", controlUpdate_dt_, 0.01);
  nh_.param<double>("/trajectory_publisher/horizon", primitive_duration_, 1.0);
  nh_.param<double>("/trajectory_publisher/maxjerk", max_jerk_, 10.0);
  nh_.param<int>("/trajectory_publisher/trajectory_type", trajectory_type_, 1);
  nh_.param<int>("/trajectory_publisher/number_of_primitives", num_primitives_, 7);
  nh_.param<double>("/trajectory_publisher/shape_radius", shape_radius_, 1.0);

  if(trajectory_type_ !=0) num_primitives_ = 1;

  inputs_.resize(num_primitives_);

  if(num_primitives_ == 7){

    inputs_.at(0) << 0.0, 0.0, 0.0; //Constant jerk inputs for minimim time trajectories
    inputs_.at(1) << 1.0, 0.0, 0.0;
    inputs_.at(2) << -1.0, 0.0, 0.0;
    inputs_.at(3) << 0.0, 1.0, 0.0;
    inputs_.at(4) << 0.0, -1.0, 0.0;
    inputs_.at(5) << 0.0, 0.0, 1.0;
    inputs_.at(6) << 0.0, 0.0, -1.0;
  }

  for(int i = 0;  i < num_primitives_; i++){
    motionPrimitives_.emplace_back(std::make_shared<polynomialtrajectory>());
    primitivePub_.push_back(nh_.advertise<nav_msgs::Path>("/trajectory_publisher/primitiveset" + std::to_string(i), 1));
    inputs_.at(i) = inputs_.at(i) * max_jerk_;
  }

  initializePrimitives();

  p_targ << init_pos_x_, init_pos_y_, init_pos_z_;
  v_targ << 0.0, 0.0, 0.0;
  motion_selector_ = 0;

}

void trajectoryPublisher::setTrajectoryTheta(double in) {
  theta_ = in;
}

void trajectoryPublisher::updateReference() {
  curr_time_ = ros::Time::now();
  trigger_time_ = (curr_time_ - start_time_).toSec();

  p_targ = motionPrimitives_.at(motion_selector_)->getPosition(trigger_time_);
  v_targ = motionPrimitives_.at(motion_selector_)->getVelocity(trigger_time_);

}

void trajectoryPublisher::initializePrimitives(){
  for(int i = 0; i < motionPrimitives_.size(); i++ ) motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_, inputs_.at(i));
}

void trajectoryPublisher::updatePrimitives(){
  for(int i = 0; i < motionPrimitives_.size() ; i++ ) motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_);
}

Eigen::Vector3d trajectoryPublisher::getTargetPosition(){
  return p_targ;
}

double trajectoryPublisher::getTrajectoryUpdateRate(){
  return controlUpdate_dt_;
}

void trajectoryPublisher::pubrefTrajectory(int selector){
  //Publish current trajectory the publisher is publishing
  refTrajectory_ = motionPrimitives_.at(selector)->getSegment();
  refTrajectory_.header.stamp = ros::Time::now();
  refTrajectory_.header.frame_id = "map";
  trajectoryPub_.publish(refTrajectory_);

}

void trajectoryPublisher::pubprimitiveTrajectory(){

  for(int i = 0; i < motionPrimitives_.size(); i++ ){
    primTrajectory_ = motionPrimitives_.at(i)->getSegment();
    primTrajectory_.header.stamp = ros::Time::now();
    primTrajectory_.header.frame_id = "map";
    primitivePub_.at(i).publish(primTrajectory_);
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
  pubrefTrajectory(motion_selector_);
  pubprimitiveTrajectory();

}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event){
  //Fast Loop publishing reference states
  pubrefState();
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){
  unsigned char mode = req.data;

  start_time_ = ros::Time::now();
  res.success = true;
  res.message = "trajectory triggered";
}

void trajectoryPublisher::motionselectorCallback(const std_msgs::Int32& selector_msg){

  motion_selector_ = selector_msg.data;
  updatePrimitives();
  start_time_ = ros::Time::now();

}

void trajectoryPublisher::trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_msg) {

  start_time_ = ros::Time::now();
  //TODO: implement a trajectory replay functionality
//  segments_message.segments;
//
//  motionPrimitives_.setPolynomial();

}

void trajectoryPublisher::mavposeCallback(const geometry_msgs::PoseStamped& msg){

  p_mav_(0) = msg.pose.position.x;
  p_mav_(1) = msg.pose.position.y;
  p_mav_(2) = msg.pose.position.z;
  updatePrimitives();

}

void trajectoryPublisher::mavtwistCallback(const geometry_msgs::TwistStamped& msg) {

  v_mav_(0) = msg.twist.linear.x;
  v_mav_(1) = msg.twist.linear.y;
  v_mav_(2) = msg.twist.linear.z;
  updatePrimitives();

}