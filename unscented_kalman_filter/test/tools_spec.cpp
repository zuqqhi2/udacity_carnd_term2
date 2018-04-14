#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "../src/tools.h"
#include "../src/Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

TEST_CASE("CalculateRMSE", "[calculate_rmse]") {
  Tools tools;
  VectorXd estimation = VectorXd(4);
  VectorXd gt_values  = VectorXd(4);

  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  estimation << 1, 2,  3,  4;
  gt_values  << 1, 4, -9, 16;
  estimations.push_back(estimation);
  ground_truth.push_back(gt_values);

  VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);

  REQUIRE(fabs(rmse(0)) < 1e-6);
  REQUIRE(fabs(rmse(1) - 2.0)  < 1e-6);
  REQUIRE(fabs(rmse(2) - 12.0) < 1e-6);
  REQUIRE(fabs(rmse(3) - 12.0) < 1e-6);
}
