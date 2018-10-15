//
// Created by jalim on 29.08.18.
//

#ifndef TRAJECTORY_PUBLISHER_TRAJECTORY_H
#define TRAJECTORY_PUBLISHER_TRAJECTORY_H

#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class trajectory {
  private:
    int N; //Degree of polynomial
    double dt_; //Sampling time
    double T_;
    int type_;
    int target_trajectoryID_;
    Eigen::Vector4d c_x_, c_y_, c_z_; //Coefficients for polynomial representation
    Eigen::Vector3d traj_axis_;
    Eigen::Vector3d traj_origin_;
    double traj_radius_, traj_omega_;

  public:
    trajectory();
    ~trajectory();
    virtual void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega) = 0;
    virtual void generatePrimitives(Eigen::Vector3d pos) = 0;
    virtual void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel) = 0;
    virtual void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk) = 0;
    virtual void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk) = 0;
    virtual Eigen::Vector3d getPosition(double time) = 0;
    virtual Eigen::Vector3d getVelocity(double time) = 0;
    virtual Eigen::Vector3d getAcceleration(double time) = 0;
    virtual double getsamplingTime(){return dt_;};
    virtual double getDuration(){ return T_;};
    virtual nav_msgs::Path getSegment() = 0;
    virtual geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation) = 0;

};


#endif //TRAJECTORY_PUBLISHER_TRAJECTORY_H
