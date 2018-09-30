//
// Created by jalim on 29.08.18.
//

#include "trajectory_publisher/trajectory.h"

trajectory::trajectory() :
  N(0),
  dt_(0.01) {

}

void trajectory::setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients, Eigen::VectorXd &z_coefficients){

  a_x_ = x_coefficients;
  a_y_ = y_coefficients;
  a_z_ = z_coefficients;

}

Eigen::VectorXd trajectory::getCoefficients(){

  a_x_ = x_coefficients;
  a_y_ = y_coefficients;
  a_z_ = z_coefficients;

}