#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools() = default;

  /**
  * Destructor.
  */
  virtual ~Tools() = default;

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth);

  VectorXd CTRVTransform(const VectorXd& x_CTRV);

};

#endif /* TOOLS_H_ */