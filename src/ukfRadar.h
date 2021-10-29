#ifndef UKFRADAR_H
#define UKFRADAR_H

#include "ukfRadar.h"
#include <memory>
#include "measurement_package.h"
class UkfRadar
{
public:
    UkfRadar(){}
    void setMainUKF(Eigen::MatrixXd& Xsig_pred,
                    Eigen::MatrixXd& R_radar, int& n_x,
                    Eigen::VectorXd& x, Eigen::VectorXd& weights,
                    Eigen::MatrixXd& P, int& n_aug);

    void updateRadar(MeasurementPackage *currentPackage);

    void updateMainUKF(int& n_aug, Eigen::MatrixXd& Xsig_pred,
                       Eigen::MatrixXd& R_radar, int& n_x,
                       Eigen::VectorXd& x, Eigen::VectorXd& weights,
                       Eigen::MatrixXd& P, double& nis_lidar);
private:
    int n_aug_;
    // predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred_;
    Eigen::MatrixXd R_radar_;
    // State dimension
    int n_x_;
    // Weights of sigma points
    Eigen::VectorXd weights_;
    // state covariance matrix
    Eigen::MatrixXd P_;
    double nis_radar_;
    double std_laspy_;
    double std_laspx_;
    double lambda_;
    Eigen::VectorXd x_;
};

#endif //UKFRADAR_H
