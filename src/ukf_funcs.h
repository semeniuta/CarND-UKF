#ifndef UNSCENTEDKF_UKF_FUNCS_H
#define UNSCENTEDKF_UKF_FUNCS_H

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

double normalize_angle(double phi);

void update_radar(
    VectorXd* x,
    MatrixXd* P,
    const VectorXd& z,
    const MatrixXd& Xsig_pred,
    const VectorXd& weights,
    double std_r,
    double std_phi,
    double std_rd
);

MatrixXd generate_sigma_points(
    const VectorXd& x,
    const MatrixXd& P,
    double std_a,
    double std_yawdd,
    double lambda
);

MatrixXd predict_sigma_points(const MatrixXd& Xsig_aug, double delta_t);

void mean_cov_from_sigma_points(VectorXd* x, MatrixXd* P, const MatrixXd& Xsig_pred, const VectorXd& weights);




#endif
