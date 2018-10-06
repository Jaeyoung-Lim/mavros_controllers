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
  primitivePub_ = nh_.advertise<nav_msgs::Path>("/trajectory_publisher/primitives", 1);
  referencePub_ = nh_.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
  motionselectorSub_ = nh_.subscribe("/trajectory_publisher/motionselector", 1, &trajectoryPublisher::motionselectorCallback, this,ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &trajectoryPublisher::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity", 1, &trajectoryPublisher::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());

  trajloop_timer_ = nh_.createTimer(ros::Duration(1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_ = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  nh_.param<double>("/trajectory_publisher/initpos_x", init_pos_x_, 0.0);
  nh_.param<double>("/trajectory_publisher/initpos_y", init_pos_y_, 0.0);
  nh_.param<double>("/trajectory_publisher/initpos_z", init_pos_z_, 1.0);
  nh_.param<double>("/trajectory_publisher/updaterate", controlUpdate_dt_, 0.01);
  nh_.param<double>("/trajectory_publisher/horizon", primitive_duration_, 1.0);
  nh_.param<int>("/trajectory_publisher/number_of_primitives", num_primitives_, 3);
  nh_.param<int>("/trajectory_publisher/number_of_primitives", num_primitives_, 3);


  Vector3d inputMatrix[num_primitives_];

  inputMatrix[0] << 0.0, 0.0, 0.0;
  inputMatrix[1] << 1.0, 0.0, 0.0;
  inputMatrix[2] << -1.0, 0.0, 0.0;

  motionPrimitives_.resize(num_primitives_);

  p_targ << init_pos_x_, init_pos_y_, init_pos_y_;
  v_targ << 0.0, 0.0, 0.0;
}

void trajectoryPublisher::setTrajectoryTheta(double in) {
  theta_ = in;
}

void trajectoryPublisher::updateReference() {
  curr_time_ = ros::Time::now();
  trigger_time_ = (curr_time_ - start_time_).toSec();

  p_targ = motionPrimitives_.at(motion_selector_).getPosition(trigger_time_);
  v_targ = motionPrimitives_.at(motion_selector_).getVelocity(trigger_time_);

}

void trajectoryPublisher::initializePrimitives(){
  for(int i = 0; i++; i < num_primitives_){
    motionPrimitives_.at(i).generatePrimitives(p_mav_, v_mav_, inputs_.at(i));
  }
}

void trajectoryPublisher::updatePrimitives(){
  for(int i = 0; i++; i < num_primitives_){
    motionPrimitives_.at(i).generatePrimitives(p_mav_, v_mav_);
  }
}

Eigen::Vector3d trajectoryPublisher::getTargetPosition(){
  return p_targ;
}

double trajectoryPublisher::getTrajectoryUpdateRate(){
  return controlUpdate_dt_;
}

void trajectoryPublisher::pubrefTrajectory(int selector){

  refTrajectory_ = motionPrimitives_.at(selector).getSegment();

  refTrajectory_.header.stamp = ros::Time::now();
  refTrajectory_.header.frame_id = 1;
  trajectoryPub_.publish(refTrajectory_);

}

void trajectoryPublisher::pubprimitiveTrajectory(){

  for(int i =0 ; i++ ; i<num_primitives_){
    refTrajectory_ = motionPrimitives_.at(i).getSegment();
    refTrajectory_.header.stamp = ros::Time::now();
    refTrajectory_.header.frame_id = 1;
    primitivePub_.publish(refTrajectory_); //TODO: Good enough for now, but should vectorize this for different primitives
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
//  pubprimitiveTrajectory();
}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event){
  //Fast Loop publishing reference states
//  pubrefState();
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

}

void trajectoryPublisher::mavtwistCallback(const geometry_msgs::TwistStamped& msg) {

  v_mav_(0) = msg.twist.linear.x;
  v_mav_(1) = msg.twist.linear.y;
  v_mav_(2) = msg.twist.linear.z;

}