#include "ukf.h"
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
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // TODO Tune this parameter
  std_yawdd_ = M_PI / 2;

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

  weights_ = VectorXd(2 * n_aug_ + 1);
  InitWeights();

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
    P_(2, 2) = 1000;
    P_(3, 3) = 1000;
    P_(4, 4) = 1000;

    is_initialized_ = true;

    return;
  }

  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1e6;
  previous_timestamp_ = meas_package.timestamp_;

  Prediction(dt);

  switch (meas_package.sensor_type_) {

    case MeasurementPackage::RADAR:

      // TODO Maybe data members preparation
      UpdateRadar(meas_package);
      break;

    case MeasurementPackage::LASER:

      // TODO Maybe data members preparation
      UpdateLidar(meas_package);
      break;
  }

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

  MatrixXd Xsig_aug = GenerateSigmaPoints();
  PredictSigmaPoints(Xsig_aug, delta_t); // fills Xsig_pred_

  MeanCovFromSigmaPoints();
  MeanCovFromSigmaPoints(); // estimates x_, P_ from Xsig_pred_

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

  VectorXd z_pred = H_lidar_ * x_;

  VectorXd y = meas_package.raw_measurements_ - z_pred;

  //y(1) = normalize_phi(y(1)); // new: no need to normalize in the LIDAR case

  MatrixXd Ht = H_lidar_.transpose();
  MatrixXd R{2, 2};s
  R.fill(0.0);
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;

  MatrixXd S = H_lidar_ * P_ * Ht + R;

  // TODO Given PHt (Tc), the latter parts should be common

  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * S.inverse();

  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I - K * H_lidar_) * P_;

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

  int n_z = 3; // r, phi, r_dot

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  // Transform sigma points into measurement space

  for (unsigned int i = 0; i < 2 * n_aug_ + 1; i++) {

    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double rho = sqrt( px * px + py * py );
    double phi = atan2(py, px);
    double rhod = (px * cos(yaw) * v + py * sin(yaw) * v) / rho;

    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = rhod;

  }

  // Calculate mean predicted measurement

  z_pred.fill(0.0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; i++) {

    z_pred += weights_(i) * Zsig.col(i);
  }

  // Calculate innovation covariance matrix S

  MatrixXd R{3, 3};
  R.fill(0.0);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  S.fill(0.0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd diff = Zsig.col(i) - z_pred;
    diff(1) = normalize_phi(diff(1));

    S += weights_(i) * (diff * diff.transpose());
  }

  S += R;

  Update(S, Zsig, meas_package.raw_measurements_, z_pred, n_z);

}

void UKF::InitWeights() {

  weights_(0) = lambda_ / (lambda_ + n_aug_);

  double w_not_mean = 1 / (2 * (lambda_ + n_aug_));
  for (unsigned int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = w_not_mean;
  }

}

MatrixXd UKF::GenerateSigmaPoints() {

  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.fill(0.);
  x_aug.head(n_x_) = x_;

  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // Square root matrix of P_aug
  MatrixXd A = P_aug.llt().matrixL();

  double factor = sqrt(lambda_ + n_aug_);

  Xsig_aug.col(0) = x_aug;

  for (unsigned int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(1 + i) = x_aug + factor * A.col(i);
    Xsig_aug.col(n_aug_ + 1 + i) = x_aug - factor * A.col(i);
  }

  return Xsig_aug;

}

void UKF::PredictSigmaPoints(const MatrixXd& Xsig_aug, double delta_t) {

  VectorXd Delta{n_x_};
  VectorXd Noise{n_x_};

  for (unsigned int j = 0; j < 2 * n_aug_ + 1; j++) {

    double v = Xsig_aug(2, j);
    double psi = Xsig_aug(3, j);
    double psi_dot = Xsig_aug(4, j);
    double noise_a = Xsig_aug(5, j);
    double noise_yawdd = Xsig_aug(6, j);

    if (fabs(psi_dot) > 0.001) {

      double ratio = v / psi_dot;
      double psi_increase = psi_dot * delta_t;

      Delta(0) = ratio * ( sin(psi + psi_increase) - sin(psi));
      Delta(1) = ratio * (-cos(psi + psi_increase) + cos(psi));
      Delta(2) = 0;
      Delta(3) = psi_increase;
      Delta(4) = 0;

    } else { // psi_dot is close to 0

      Delta(0) = v * cos(psi) * delta_t;
      Delta(1) = v * sin(psi) * delta_t;
      Delta(2) = 0;
      Delta(3) = 0;
      Delta(4) = 0;

    }

    double delta_t_sq = delta_t * delta_t;

    Noise(0) = 0.5 * delta_t_sq * cos(psi) * noise_a;
    Noise(1) = 0.5 * delta_t_sq * sin(psi) * noise_a;
    Noise(2) = delta_t * noise_a;
    Noise(3) = 0.5 * delta_t_sq * noise_yawdd;
    Noise(4) = delta_t * noise_yawdd;

    Xsig_pred_.col(j) = Xsig_aug.block(0, j, n_x_, 1) + Delta + Noise;

  }
}

void UKF::MeanCovFromSigmaPoints() {

  //predict state mean

  x_.fill(0.0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix

  P_.fill(0.0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd mean_diff = Xsig_pred_.col(i) - x_;

    mean_diff(3) = normalize_phi(mean_diff(3));
    mean_diff(4) = normalize_phi(mean_diff(4)); // new: yaw rate is also an angle

    P_ += weights_(i) * (mean_diff * mean_diff.transpose());
  }

}

void UKF::Update(const MatrixXd& S, const MatrixXd& Zsig, const VectorXd& z, const VectorXd& z_pred, int n_z) {

  // Cross correlation matrix

  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix

  Tc.fill(0.0);
  for (unsigned int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd diff_x = Xsig_pred_.col(i) - x_;
    diff_x(3) = normalize_phi(diff_x(3));
    diff_x(4) = normalize_phi(diff_x(4)); // new: yaw rate is also an angle

    VectorXd diff_z = Zsig.col(i) - z_pred;
    diff_z(1) = normalize_phi(diff_z(1));

    Tc += weights_(i) * diff_x * diff_z.transpose();
  }

  // Kalman gain

  MatrixXd K = Tc * S.inverse();

  // Update state mean and covariance matrix

  x_ += K * (z - z_pred);
  P_ -= K * S * K.transpose();

}

void NormalizeAngle(VectorXd* p_vec, int idx) {

  double x = (*p_vec)(idx);

  while (x >  M_PI) x -= 2.*M_PI;
  while (x < -M_PI) x += 2.*M_PI;

  (*p_vec)(idx) = x;

}


double normalize_phi(double phi) {

  if ((phi > -M_PI) && (phi < M_PI)) {
    return phi;
  }

  double TWO_PI = 2 * M_PI;
  double n_twopies = (abs(phi) - M_PI) / TWO_PI;

  double phi_norm;

  if (phi < -M_PI) {

    phi_norm = phi + ceil(n_twopies) * TWO_PI;

  } else  { // phi > M_PI

    phi_norm = phi - ceil(n_twopies) * TWO_PI;

  }

  return phi_norm;

}

