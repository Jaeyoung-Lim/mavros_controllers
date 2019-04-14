#include "controller_benchmark/error_measurer.h"

ErrorMeasurer::ErrorMeasurer() :
  totalsamples_(0),
  windowsamples_(0),
  N(50) {

    sample_error_.resize(N);

    for(int i = 0 ; i < N ; i++){
      sample_error_[i] = 0.0;
    }
    sum_error_ = 0.0;

}

ErrorMeasurer::~ErrorMeasurer() {
  //Destructor
}

void ErrorMeasurer::Add(double error){
  //Accumulate error value
  if (windowsamples_ < N) {
    sample_error_[windowsamples_] = error;
    sum_error_ += error;
    sample_error_[windowsamples_] = error;

  } else {
    double &oldest_error = sample_error_[windowsamples_ % N];

    sum_error_ = sum_error_ - oldest_error + error;
    oldest_error = error;
  }
  windowsamples_++;

  ++totalsamples_;
}

void ErrorMeasurer::GetMeanError(double &meanerror){
  if(windowsamples_ > N) meanerror = sum_error_ / N;
  else meanerror = sum_error_ / windowsamples_;
}

int ErrorMeasurer::GetWindowSamples(){
  return windowsamples_;

}
