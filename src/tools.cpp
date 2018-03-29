#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Calculate the RMSE here.
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if(estimations.size() == 0){
    cout << "ERROR - CalculateRMSE () - The estimations vector is empty" << endl;
    return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if(ground_truth.size() != estimations.size()){
    cout << "ERROR - CalculateRMSE () - Data should have the same size" << endl;
    return rmse;
  }

  // Accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}
