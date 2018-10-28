#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth) {

  VectorXd result(4);
  result << 0,0,0,0;

  if (estimations.empty()) {
    cerr << "Empty estimations vector\n";
    return result;
  }

  if (estimations.size() != ground_truth.size()) {
    cerr << "Vector sizes mismatch\n";
    return result;
  }

  unsigned long n = estimations.size();

  for(int i = 0; i < n; i++){

    VectorXd x_hat = estimations[i];
    VectorXd x_true = ground_truth[i];

    VectorXd diff = x_hat - x_true;
    VectorXd sq_diff = diff.array() * diff.array();

    result = result + sq_diff;
  }

  result = result / n;

  result = result.array().sqrt();

  return result;

}