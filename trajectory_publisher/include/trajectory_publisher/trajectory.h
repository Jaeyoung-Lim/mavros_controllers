//
// Created by jalim on 29.08.18.
//

#ifndef TRAJECTORY_PUBLISHER_TRAJECTORY_H
#define TRAJECTORY_PUBLISHER_TRAJECTORY_H

#include <Eigen/Dense>


class trajectory {
  private:
    int N; //Degree of polynomial
    double dt_; //Sampling time
    double T_;
    Eigen::VectorXd c_x_, c_y_, c_z_; //Coefficients for polynomial representation

  public:
    trajectory(double duration);
    void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel);
    void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc);
    void setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients, Eigen::VectorXd &z_coefficients);
    Eigen::VectorXd getCoefficients(int dim);
    Eigen::Vector3d getPosition(double time);
    Eigen::Vector3d getVelocity(double time);
    double getsamplingTime(){return dt_;};
    double getDuration(){ return T_;};
};


#endif //TRAJECTORY_PUBLISHER_TRAJECTORY_H
