#include "ukf.h"
#include "ukf_funcs.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  is_initialized_ = false;

  n_x_ = 5;

  n_aug_ = n_x_ + 2;

  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // TODO Tune this parameter
  std_a_ = 0.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // TODO Tune this parameter
  std_yawdd_ = 1.;

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

    is_initialized_ = true;

    return;
  }

  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1e6;
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(dt);
  NormalizeAnglesInState();

  switch (meas_package.sensor_type_) {

    case MeasurementPackage::RADAR:

      UpdateRadar(meas_package);
      break;

    case MeasurementPackage::LASER:

      UpdateLidar(meas_package);
      break;
  }

  NormalizeAnglesInState();

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  MatrixXd Xsig_aug = generate_sigma_points(x_, P_, std_a_, std_yawdd_, lambda_);

  Xsig_pred_ = predict_sigma_points(Xsig_aug, delta_t);

  mean_cov_from_sigma_points(&x_, &P_, Xsig_pred_, weights_);

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage& meas_package) {

  nis_ = update_lidar(
      &x_,
      &P_,
      meas_package.raw_measurements_,
      H_lidar_,
      std_laspx_,
      std_laspy_
  );

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package) {

  nis_ = update_radar(
      &x_,
      &P_,
      meas_package.raw_measurements_,
      Xsig_pred_,
      weights_,
      std_radr_,
      std_radphi_,
      std_radrd_
  );

}


void UKF::NormalizeAnglesInState() {

  x_(3) = normalize_angle(x_(3)); // yaw
  x_(4) = normalize_angle(x_(4)); // yaw rate

}