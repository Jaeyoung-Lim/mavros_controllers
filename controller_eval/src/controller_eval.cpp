//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "controller_eval/controller_eval.h"

//Constructor
ControllerEvaluator::ControllerEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

  mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &ControllerEvaluator::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 1, &ControllerEvaluator::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  gzmavposeSub_ = nh_.subscribe("/gazebo/model_states", 1, &ControllerEvaluator::gzmavposeCallback, this, ros::TransportHints().tcpNoDelay());	

}
ControllerEvaluator::~ControllerEvaluator() {
  //Destructor
}

void ControllerEvaluator::mavposeCallback(const geometry_msgs::PoseStamped& msg){
  mavPos_(0) = msg.pose.position.x;
  mavPos_(1) = msg.pose.position.y;
  mavPos_(2) = msg.pose.position.z;
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;

}

void ControllerEvaluator::mavtwistCallback(const geometry_msgs::TwistStamped& msg){
  
  mavVel_(0) = msg.twist.linear.x;
  mavVel_(1) = msg.twist.linear.y;
  mavVel_(2) = msg.twist.linear.z;
  mavRate_(0) = msg.twist.angular.x;
  mavRate_(1) = msg.twist.angular.y;
  mavRate_(2) = msg.twist.angular.z;
  
}

void ControllerEvaluator::gzmavposeCallback(const gazebo_msgs::ModelStates& msg){	
  for(int i = 0; i < msg.pose.size(); i++){	
    if(msg.name[i] == mav_name_){	
      gt_mavPos_(0) = msg.pose[i].position.x;	
      gt_mavPos_(1) = msg.pose[i].position.y;	
      gt_mavPos_(2) = msg.pose[i].position.z;	
      gt_mavAtt_(0) = msg.pose[i].orientation.w;	
      gt_mavAtt_(1) = msg.pose[i].orientation.x;	
      gt_mavAtt_(2) = msg.pose[i].orientation.y;	
      gt_mavAtt_(3) = msg.pose[i].orientation.z;	
      gt_mavVel_(0) = msg.twist[i].linear.x;	
      gt_mavVel_(1) = msg.twist[i].linear.y;	
      gt_mavVel_(2) = msg.twist[i].linear.z;	
      gt_mavRate_(0) = msg.twist[i].angular.x;	
      gt_mavRate_(1) = msg.twist[i].angular.y;	
      gt_mavRate_(2) = msg.twist[i].angular.z;
      break;
    }	
  }		
}