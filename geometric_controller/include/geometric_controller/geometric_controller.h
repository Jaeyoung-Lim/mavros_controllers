//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>

#include <controller_msgs/FlatTarget.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <dynamic_reconfigure/server.h>
#include <geometric_controller/GeometricControllerConfig.h>

#define MODE_ROTORTHRUST  1
#define MODE_BODYRATE     2
#define MODE_BODYTORQUE   3

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

class geometricCtrl
{
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
    bool fail_detec_, ctrl_enable_, feedthrough_enable_;
    int ctrl_mode_;
    bool landing_commanded_;
    bool sim_enable_, use_dob_;
    bool velocity_yaw_;
    double kp_rot_, kd_rot_;
    double reference_request_dt_;
    double attctrl_tau_;
    double norm_thrust_const_, norm_thrust_offset_;
    double max_fb_acc_;
    double dx_, dy_, dz_;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::AttitudeTarget angularVelMsg_;
    geometry_msgs::PoseStamped referencePoseMsg_;
    std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
    MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

    Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
    Eigen::Vector3d mavPos_, mavVel_, mavRate_;
    double mavYaw_;
    Eigen::Vector3d a_des, a_fb, a_ref, a_rd, a_dob, g_;
    Eigen::Vector4d mavAtt_, q_ref, q_des;
    Eigen::Vector4d cmdBodyRate_; //{wx, wy, wz, Thrust}
    Eigen::Vector3d Kpos_, Kvel_, D_;
    Eigen::Vector3d a0, a1, tau;
    Eigen::Vector3d errorPos_, errorVel_;
    double tau_x, tau_y, tau_z;
    double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
    int posehistory_window_;

    void pubMotorCommands();
    void pubRateCommands();
    void pubReferencePose();
    void pubPoseHistory();
    void pubSystemStatus();
    void appendPoseHistory();
    void odomCallback(const nav_msgs::OdometryConstPtr& odomMsg);
    void targetCallback(const geometry_msgs::TwistStamped& msg);
    void flattargetCallback(const controller_msgs::FlatTarget& msg);
    void yawtargetCallback(const std_msgs::Float32& msg);
    void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory& msg);
    void keyboardCallback(const geometry_msgs::Twist& msg);
    void cmdloopCallback(const ros::TimerEvent& event);
    void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
    void mavposeCallback(const geometry_msgs::PoseStamped& msg);
    void mavtwistCallback(const geometry_msgs::TwistStamped& msg);
    void statusloopCallback(const ros::TimerEvent& event);
    bool ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    Eigen::Vector4d acc2quaternion(Eigen::Vector3d vector_acc, double yaw);
    Eigen::Vector4d rot2Quaternion(Eigen::Matrix3d R);
    Eigen::Matrix3d quat2RotMatrix(Eigen::Vector4d q);
    geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);

    enum FlightState {
      WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED
    } node_state;

    template <class T>
    void waitForPredicate(const T* pred, const std::string& msg, double hz = 2.0){
        ros::Rate pause(hz);
        ROS_INFO_STREAM(msg);
        while(ros::ok() && !(*pred)){
            ros::spinOnce();
            pause.sleep();
        }
    };
    geometry_msgs::Pose home_pose_;
    bool received_home_pose;

  public:
    void dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config,uint32_t level);
    geometricCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    void computeBodyRateCmd(bool ctrl_mode);
    Eigen::Vector4d quatMultiplication(Eigen::Vector4d &q, Eigen::Vector4d &p);
    Eigen::Vector4d attcontroller(Eigen::Vector4d &ref_att, Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
    void getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel);
    void getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel);
    void setBodyRateCommand(Eigen::Vector4d bodyrate_command);
    void setFeedthrough(bool feed_through);
    void setDesiredAcceleration(Eigen::Vector3d acc_desired);
    virtual ~ geometricCtrl();
};


#endif
