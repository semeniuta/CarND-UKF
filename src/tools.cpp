#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth) {

  VectorXd result(4);
  result.fill(0.0);

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

    VectorXd x_hat = CTRVTransform(estimations[i]);
    VectorXd x_true = ground_truth[i];

    VectorXd diff = x_hat - x_true;
    VectorXd sq_diff = diff.array() * diff.array();

    result = result + sq_diff;
  }

  result = result / n;

  result = result.array().sqrt();

  return result;

}


VectorXd Tools::CTRVTransform(const VectorXd& x_CTRV) {

  VectorXd x_hat{4};

  double px = x_CTRV(0);
  double py = x_CTRV(1);
  double v = x_CTRV(2);
  double yaw = x_CTRV(3);

  x_hat << px,
           py,
           v * cos(yaw),
           v * sin(yaw);

  return x_hat;

}