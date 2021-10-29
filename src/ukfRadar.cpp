#include "ukfRadar.h"
#include "Eigen/Dense"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
void UkfRadar::updateMainUKF(int &n_aug, Eigen::MatrixXd &Xsig_pred, Eigen::MatrixXd &R_radar,
                             int &n_x, Eigen::VectorXd &x, Eigen::VectorXd &weights, Eigen::MatrixXd &P, double &nis_radar)
{
    std::cout<<"UkfRadar::updateMainUKF"<<std::endl;
    n_aug = n_aug_;
    Xsig_pred = Xsig_pred_;
    R_radar = R_radar_;
    n_x = n_x_;
    x = x_;
    weights = weights_;
    P = P_;
    nis_radar = nis_radar;
}

void UkfRadar::setMainUKF(Eigen::MatrixXd &Xsig_pred, Eigen::MatrixXd &R_radar,
                          int &n_x, Eigen::VectorXd &x, Eigen::VectorXd &weights, Eigen::MatrixXd &P, int& n_aug)
{

    std::cout<<"UkfRadar::setMainUKF"<<std::endl;
    Xsig_pred_ = Xsig_pred;
    R_radar_ = R_radar;
    n_x_= n_x;
    x_ = x;
    weights_ = weights;
    P_ = P;
    n_aug_ = n_aug;
}

void UkfRadar::updateRadar(MeasurementPackage* currentPackage)
{
    std::cout<<"UkfRadar::updateRadar"<<std::endl;
    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z_ = 3;
    VectorXd z = VectorXd(n_z_);
    double meas_rho = currentPackage->raw_measurements_(0);
    double meas_phi = currentPackage->raw_measurements_(1);
    double meas_rhod = currentPackage->raw_measurements_(2);
    z << meas_rho,
            meas_phi,
            meas_rhod;

    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_);
    z_pred.fill(0.0);
    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_, n_z_);
    S.fill(0.0);

    // transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        // 2n+1 simga points
        // extract values for better readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        double v1 = cos(yaw) * v;
        double v2 = sin(yaw) * v;

        // measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y); //r
        Zsig(1, i) = atan2(p_y, p_x); // phi
        Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); //r_dot

        //calculate mean predicted measurement
        z_pred += weights_(i) * Zsig.col(i);
    }

    // innovation covariance matrix S
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        // 2n+1 simga points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S += weights_(i) * z_diff * z_diff.transpose();
    }

    S += R_radar_;


    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_);
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        // 2n+1 simga points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    //  Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    // update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();

    // compute normalized innovation squared(NIS)
    nis_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
