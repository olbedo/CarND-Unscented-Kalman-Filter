#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() == 0) {
    cout << "Error: Size of the estimation vector is zero" << endl;
    return rmse;
  }

  if (estimations.size() != ground_truth.size()) {
    cout << "Error: Sizes of the estimation vector and the ground_truth vector differ" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd diff = estimations.at(i) - ground_truth.at(i);
    rmse.array() += diff.array() * diff.array();
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}
