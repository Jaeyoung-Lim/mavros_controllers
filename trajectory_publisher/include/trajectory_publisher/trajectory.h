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
    void setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients, Eigen::VectorXd &z_coefficients);
    Eigen::VectorXd getCoefficients(int dim);
};


#endif //TRAJECTORY_PUBLISHER_TRAJECTORY_H
