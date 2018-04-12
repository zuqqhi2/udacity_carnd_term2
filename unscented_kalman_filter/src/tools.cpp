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
  int size = estimations.size();

  VectorXd rmse(4);
  rmse.fill(0.0);

  //Check element numbers
  if (size != ground_truth.size() || estimations.size() == 0) {
    return rmse;
  }

  // Calculation square
  for (int i = 0; i < size; i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calcuate mean & square root
  rmse /= size;
  rmse = rmse.array().sqrt();
}
