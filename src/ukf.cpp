#include "ukf.h"
#include "ukf_funcs.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  n_x_ = 5;

  n_aug_ = n_x_ + 2;

  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // TODO Tune this parameter
  std_a_ = 0.1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // TODO Tune this parameter
  std_yawdd_ = 0.05;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  weights_ = init_weights(n_aug_, lambda_);

  H_lidar_ = MatrixXd{2, 5};
  H_lidar_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage& meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  Tools tools;
  VectorXd x_hat;

  if (!is_initialized_) {

    previous_timestamp_ = meas_package.timestamp_;
    x_.fill(0.);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];

      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];

    }

    P_.fill(0.);
    P_(0, 0) = 1;
    P_(1, 1) = 1;
    P_(2, 2) = 10;
    P_(3, 3) = 10;
    P_(4, 4) = 10;

    x_hat = tools.CTRVTransform(x_);
    cout << "x_hat (init) = \n" << x_hat << "\n";

    is_initialized_ = true;

    return;
  }

  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1e6;
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(dt);
  NormalizeAnglesInState();

  cout << "x_CTRV (after prediction) = \n" << x_ << "\n";
  x_hat = tools.CTRVTransform(x_);
  cout << "x_hat (predict) = \n" << x_hat << "\n";

  switch (meas_package.sensor_type_) {

    case MeasurementPackage::RADAR:

      std::cout << "Updating (RADAR)\n";

      // TODO Maybe data members preparation
      UpdateRadar(meas_package);
      break;

    case MeasurementPackage::LASER:

      std::cout << "Updating (LIDAR)\n";

      // TODO Maybe data members preparation
      UpdateLidar(meas_package);
      break;
  }

  //NormalizeAnglesInState();

  //cout << "x_CTRV (after update) = \n" << x_ << "\n";
  //x_hat = tools.CTRVTransform(x_);
  //cout << "x_hat (update) = \n" << x_hat << "\n";

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  MatrixXd Xsig_aug = generate_sigma_points(x_, P_, std_a_, std_yawdd_, lambda_);

  Xsig_pred_ = predict_sigma_points(Xsig_aug, delta_t);

  mean_cov_from_sigma_points(&x_, &P_, Xsig_pred_, weights_);

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage& meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  double nis = update_lidar(
      &x_,
      &P_,
      meas_package.raw_measurements_,
      H_lidar_,
      std_laspx_,
      std_laspy_
  );

  std::cout << "NIS_lidar = " << nis << "\n";

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  double nis = update_radar(
      &x_,
      &P_,
      meas_package.raw_measurements_,
      Xsig_pred_,
      weights_,
      std_radr_,
      std_radphi_,
      std_radrd_
  );

  std::cout << "NIS_radar = " << nis << "\n";

}


void UKF::NormalizeAnglesInState() {

  if ((x_(3) <= -M_PI) || (x_(3) >= M_PI)) {
    std::cout << "yaw outside of range\n";
  }

  if ((x_(4) <= -M_PI) || (x_(4) >= M_PI)) {
    std::cout << "yawdd outside of range\n";
  }

  x_(3) = normalize_angle(x_(3)); // yaw
  x_(4) = normalize_angle(x_(4)); // yaw rate

}