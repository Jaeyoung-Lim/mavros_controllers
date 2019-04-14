  
//  April/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef CONTROLLER_EVAL_ERROR_MEASURER_H
#define CONTROLLER_EVAL_ERROR_MEASURER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <Eigen/Dense>

class ErrorMeasurer
{
  private:
    double mean_error_;
    double threshold_;
    double max_;
    double min_;
    double sum_error_;
    int N;
    int windowsamples_;
    int totalsamples_;
    std::vector<double> sample_error_;

public:
    ErrorMeasurer();
    virtual ~ ErrorMeasurer();
    void Add(double error);
    void GetMeanError(double &mean_error);
    int GetWindowSamples();
};

#endif //CONTROLLER_EVAL_ERROR_MEASURER_H