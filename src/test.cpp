#include "ukf_funcs.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = n_x + 2;

  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  VectorXd x{5};
  x <<   5.7441,
      1.3800,
      2.2049,
      0.5015,
      0.3528;

  //set example covariance matrix
  MatrixXd P{5, 5};
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
      -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
      0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
      -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
      -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  MatrixXd Xsig_aug = generate_sigma_points(x, P, std_a, std_yawdd, lambda);

  std::cout << "Xsig_aug = \n" << Xsig_aug << "\n";

  /* expected result:
   Xsig_aug =
  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
    1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
  0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
       0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
       0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
*/

  double delta_t = 0.1;

  MatrixXd Xsig_pred = predict_sigma_points(Xsig_aug, delta_t);

  std::cout << "Xsig_pred = \n" << Xsig_pred << "\n";

  /*

   expected result:
  Xsig_pred =

  5.93553 6.06251 5.92217 5.9415 5.92361 5.93516 5.93705 5.93553 5.80832 5.94481 5.92935 5.94553 5.93589 5.93401 5.93553

  1.48939 1.44673 1.66484 1.49719 1.508 1.49001 1.49022 1.48939 1.5308 1.31287 1.48182 1.46967 1.48876 1.48855 1.48939

  2.2049 2.28414 2.24557 2.29582 2.2049 2.2049 2.23954 2.2049 2.12566 2.16423 2.11398 2.2049 2.2049 2.17026 2.2049

  0.53678 0.473387 0.678098 0.554557 0.643644 0.543372 0.53678 0.538512 0.600173 0.395462 0.519003 0.429916 0.530188 0.53678 0.535048

  0.3528 0.299973 0.462123 0.376339 0.48417 0.418721 0.3528 0.387441 0.405627 0.243477 0.329261 0.22143 0.286879 0.3528 0.318159

   */

  VectorXd weights = init_weights(n_aug, lambda);

  mean_cov_from_sigma_points(&x, &P, Xsig_pred, weights);

  std::cout << "x = \n" << x << "\n";

  std::cout << "P = \n" << P << "\n";

  /*

   expected result x:
 x =
5.93637

1.49035

2.20528

0.536853

0.353577

expected result p:
 P =
0.00543425 -0.0024053 0.00341576 -0.00348196 -0.00299378

-0.0024053 0.010845 0.0014923 0.00980182 0.00791091

0.00341576 0.0014923 0.00580129 0.000778632 0.000792973

-0.00348196 0.00980182 0.000778632 0.0119238 0.0112491

-0.00299378 0.00791091 0.000792973 0.0112491 0.0126972

   */

  VectorXd z{3};
  z <<
    5.9214,   //rho in m
      0.2187,   //phi in rad
      2.0062;   //rho_dot in m/s

  update_radar(
      &x,
      &P,
      z,
      Xsig_pred,
      weights,
      std_radr,
      std_radphi,
      std_radrd
  );

  std::cout << "x = \n" << x << "\n";

  std::cout << "P = \n" << P << "\n";

  /*

   expected result z_out:
 z_pred =
6.12155

0.245993

2.10313

expected result p:
 S =
0.0946171 -0.000139448 0.00407016

-0.000139448 0.000617548 -0.000770652

0.00407016 -0.000770652 0.0180917

   */

  /*

   expected result x:
 x =
5.92276

1.41823

2.15593

0.489274

0.321338

expected result P:
 P =
0.00361579 -0.000357881 0.00208316 -0.000937196 -0.00071727

-0.000357881 0.00539867 0.00156846 0.00455342 0.00358885

0.00208316 0.00156846 0.00410651 0.00160333 0.00171811

-0.000937196 0.00455342 0.00160333 0.00652634 0.00669436

-0.00071719 0.00358884 0.00171811 0.00669426 0.00881797

   */




}