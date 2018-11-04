#include "ukf_funcs.h"


double normalize_angle(double phi) {

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


void update_radar(
    VectorXd* x,
    MatrixXd* P,
    const VectorXd& z,
    const MatrixXd& Xsig_pred,
    const VectorXd& weights,
    double std_r,
    double std_phi,
    double std_rd
    ) {

  int n_z = 3; // r, phi, r_dot
  long n_x = x->size();
  long n_cols = Xsig_pred.cols(); // 2 * n_aug + 1

  MatrixXd Zsig = MatrixXd(n_z, n_cols);

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  // Transform sigma points into measurement space

  for (unsigned int i = 0; i < n_cols; i++) {

    double px = Xsig_pred(0, i);
    double py = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double rho = sqrt( px * px + py * py );
    double phi = atan2(py, px);
    double rhod = (px * cos(yaw) * v + py * sin(yaw) * v) / rho;

    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = rhod;

  }

  // Calculate mean predicted measurement

  z_pred.fill(0.0);
  for (unsigned int i = 0; i < n_cols; i++) {

    z_pred += weights(i) * Zsig.col(i);
  }

  // Calculate innovation covariance matrix S

  MatrixXd R{3, 3};
  R.fill(0.0);
  R(0, 0) = std_r * std_r;
  R(1, 1) = std_phi * std_phi;
  R(2, 2) = std_rd * std_rd;

  S.fill(0.0);
  for (unsigned int i = 0; i < n_cols; i++) {

    VectorXd diff = Zsig.col(i) - z_pred;
    diff(1) = normalize_angle(diff(1));

    S += weights(i) * (diff * diff.transpose());
  }

  S += R;

  // Cross correlation matrix

  MatrixXd Tc = MatrixXd(n_x, n_z);

  //calculate cross correlation matrix

  Tc.fill(0.0);
  for (unsigned int i = 0; i < n_cols; i++) {

    VectorXd diff_x = Xsig_pred.col(i) - *x;
    diff_x(3) = normalize_angle(diff_x(3)); // yaw
    diff_x(4) = normalize_angle(diff_x(4)); // yaw rate

    VectorXd diff_z = Zsig.col(i) - z_pred;
    diff_z(1) = normalize_angle(diff_z(1)); // radar phi

    Tc += weights(i) * diff_x * diff_z.transpose();
  }

  // Kalman gain

  MatrixXd K = Tc * S.inverse();

  // Update state mean and covariance matrix

  VectorXd y = z - z_pred;
  y(1) = normalize_angle(y(1)); // radar phi

  *x = *x + K * y;
  *P = *P - K * S * K.transpose();

}

MatrixXd generate_sigma_points(
    const VectorXd& x,
    const MatrixXd& P,
    double std_a,
    double std_yawdd,
    double lambda
    ) {

  long n_x = x.size();
  long n_aug = n_x + 2;

  VectorXd x_aug = VectorXd(n_aug);
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  x_aug.fill(0.);
  x_aug.head(n_x) = x;

  P_aug.topLeftCorner(n_x, n_x) = P;
  P_aug(n_x, n_x) = std_a * std_a;
  P_aug(n_x + 1, n_x + 1) = std_yawdd * std_yawdd;

  // Square root matrix of P_aug
  MatrixXd A = P_aug.llt().matrixL();

  double factor = sqrt(lambda + n_aug);

  Xsig_aug.col(0) = x_aug;

  for (unsigned int i = 0; i < n_aug; i++) {
    Xsig_aug.col(1 + i) = x_aug + factor * A.col(i);
    Xsig_aug.col(n_aug + 1 + i) = x_aug - factor * A.col(i);
  }

  return Xsig_aug;

}


MatrixXd predict_sigma_points(const MatrixXd& Xsig_aug, double delta_t) {

  long n_cols = Xsig_aug.cols();
  long n_aug = Xsig_aug.rows();
  long n_x = n_aug - 2;

  MatrixXd Xsig_pred{n_x, n_cols};

  VectorXd Delta{n_x};
  VectorXd Noise{n_x};

  for (unsigned int j = 0; j < n_cols; j++) {

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

    Xsig_pred.col(j) = Xsig_aug.block(0, j, n_x, 1) + Delta + Noise;
  }

  return Xsig_pred;

}

void mean_cov_from_sigma_points(VectorXd* x, MatrixXd* P, const MatrixXd& Xsig_pred, const VectorXd& weights) {

  //predict state mean

  long n_cols = Xsig_pred.cols();

  x->fill(0.0);
  for (unsigned int i = 0; i < n_cols; i++) {
    *x = *x + weights(i) * Xsig_pred.col(i);
  }

  //predict state covariance matrix

  P->fill(0.0);
  for (unsigned int i = 0; i < n_cols; i++) {

    VectorXd diff_x = Xsig_pred.col(i) - *x;

    diff_x(3) = normalize_angle(diff_x(3)); // yaw
    diff_x(4) = normalize_angle(diff_x(4)); // yaw rate

    *P = *P + weights(i) * (diff_x * diff_x.transpose());
  }

}

VectorXd init_weights(int n_aug, double lambda) {

  VectorXd weights{2 * n_aug + 1};

  weights(0) = lambda / (lambda + n_aug);

  double w_not_mean = 1 / (2 * (lambda + n_aug));
  for (unsigned int i = 1; i < 2 * n_aug + 1; i++) {
    weights(i) = w_not_mean;
  }

  return weights;

}