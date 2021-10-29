#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include "ukfLidar.h"
#include "ukfRadar.h"
#include <memory>

class UKF
{
 public:
  /**
   * Constructor
   */
   UKF();

  /**
   * Destructor
   */
   virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
    void ProcessMeasurement(MeasurementPackage& meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
    void Prediction(double delta_t);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  Eigen::MatrixXd R_radar_;

  Eigen::MatrixXd R_lidar_;

  // time when the state is true, in us
  long long time_us_;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  double nis_radar_;

  double nis_lidar_;

  MeasurementPackage* currentPackage_;

private:
  void initialize();
  void initLidar();
  void initRadar();
  //UkfLidar* ukfLidar_;
  UkfRadar ukfRadar_;
  UkfLidar ukfLidar_;

  double computeDeltaT();
};

#endif  // UKF_H
