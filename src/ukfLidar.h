#ifndef UKFLIDAR_H
#define UKFLIDAR_H


#include "ukfLidar.h"
#include <memory>
#include "measurement_package.h"

class UkfLidar
{
public:
    UkfLidar();
    ~UkfLidar();
    void updateLidar(MeasurementPackage *currentPackage_);
    void updateMainUKF(int& n_aug, Eigen::MatrixXd& Xsig_pred,
                       Eigen::MatrixXd& R_lidar,int& n_x,
                       Eigen::VectorXd& x, Eigen::VectorXd& weights,
                       Eigen::MatrixXd& P, double& nis_lidar) const;
    void setMainUKF(Eigen::MatrixXd& Xsig_pred,
                    Eigen::MatrixXd& R_lidar, int& n_x,
                    Eigen::VectorXd& x, Eigen::VectorXd& weights,
                    Eigen::MatrixXd& P);

private:
      int n_aug_;
      // predicted sigma points matrix
      Eigen::MatrixXd Xsig_pred_;
      Eigen::MatrixXd R_lidar_;
      // State dimension
      int n_x_;
      // Weights of sigma points
      Eigen::VectorXd weights_;
      // state covariance matrix
      Eigen::MatrixXd P_;
      double nis_lidar_;
      double std_laspy_;
      double std_laspx_;
      double lambda_;
      Eigen::VectorXd x_;
};

#endif //UKFLIDAR_H
