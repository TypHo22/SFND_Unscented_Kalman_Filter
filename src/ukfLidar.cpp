#include "ukfLidar.h"
#include "Eigen/Dense"
#include "iostream"
#include "exception"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

UkfLidar::UkfLidar()
{
    std::cout << "UkfLidar::UkfLidar" << std::endl;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    /**
     * TODO: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */
    n_x_ = 5;
    n_aug_ = 7;
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    // Define spreading parameter for augmentation
    lambda_ = 3 - n_aug_;

    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_.fill(0.5 / (lambda_ + n_aug_));
    weights_(0) = lambda_ / (lambda_ + n_aug_);

    R_lidar_ = MatrixXd(2, 2);
    R_lidar_.fill(0.0);
    R_lidar_(0, 0) = std_laspx_ * std_laspx_;
    R_lidar_(1, 1) = std_laspy_ * std_laspy_;
}

UkfLidar::~UkfLidar()
{
    std::cout<<"UkfLidar::~UkfLidar"<<std::endl;
}

void UkfLidar::updateLidar(MeasurementPackage* currentPackage_)
{
    std::cout << "UKF::UpdateLidar" << std::endl;
    // set measurement dimension, px,py
    int n_z_ = 2;

    // create example vector for incoming lidar measurement
    VectorXd z = VectorXd(n_z_);
    z << currentPackage_->raw_measurements_(0),
            currentPackage_->raw_measurements_(1);

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

        // measurement model
        Zsig(0, i) = p_x;
        Zsig(1, i) = p_y;
        // mean predicted measurement
        z_pred += weights_(i) * Zsig.col(i);
    }

    // innovation covariance matrix S
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        // 2n+1 simga points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        S += weights_(i) * z_diff * z_diff.transpose();
    }

    S += R_lidar_;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_);
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        // 2n+1 simga points

        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // normalize angles
        while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    //  Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();

    // compute normalized innovation squared(NIS)
    nis_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
}

void UkfLidar::updateMainUKF(int &n_aug, Eigen::MatrixXd &Xsig_pred, Eigen::MatrixXd &R_lidar,
                             int &n_x, Eigen::VectorXd &x, Eigen::VectorXd &weights, Eigen::MatrixXd &P,
                             double &nis_lidar) const
{
    std::cout<<"UkfLidar::updateMainUKF"<<std::endl;
    n_aug = n_aug_;
    Xsig_pred = Xsig_pred_;
    R_lidar = R_lidar_;
    n_x = n_x_;
    weights = weights_;
    P = P_;
    nis_lidar = nis_lidar_;
    x = x_;
}

void UkfLidar::setMainUKF(Eigen::MatrixXd &Xsig_pred, Eigen::MatrixXd &R_lidar, int &n_x, Eigen::VectorXd &x, Eigen::VectorXd &weights, Eigen::MatrixXd &P)
{
    std::cout<<"UkfLidar::setMainUKF"<<std::endl;
    Xsig_pred_ = Xsig_pred;
    R_lidar_ = R_lidar;
    n_x_ = n_x;
    weights_ = weights;
    P_ = P;
    x_ = x;
}



