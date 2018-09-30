//
// Created by jalim on 29.08.18.
//

#include "trajectory_publisher/trajectory.h"

trajectory::trajectory(double duration) :
  N(0),
  dt_(0.1),
  T_(duration) {

}

void trajectory::setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients, Eigen::VectorXd &z_coefficients){

  c_x_ = x_coefficients;
  c_y_ = y_coefficients;
  c_z_ = z_coefficients;

}

Eigen::VectorXd trajectory::getCoefficients(int dim){

  switch(dim){
    case 0 : return c_x_;
    case 1 : return c_y_;
    case 2 : return c_z_;
  }
}