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
    Eigen::VectorXd a_x_, a_y_, a_z_; //Coefficients for polynomial representation

  public:
    trajectory();
    void setCoefficients(Eigen::VectorXd &x_coefficients, Eigen::VectorXd &y_coefficients, Eigen::VectorXd &z_coefficients);
    void getCoefficients();


};


#endif //TRAJECTORY_PUBLISHER_TRAJECTORY_H
