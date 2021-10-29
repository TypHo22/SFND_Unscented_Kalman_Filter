#include "ukf.h"
#include "Eigen/Dense"
#include "iostream"
#include "exception"
#include "UkfDefines.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */

UKF::UKF()
{
    std::cout << "UKF::UKF" << std::endl;
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

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


    // add measurement noise covariance matrix
    R_radar_ = MatrixXd(3, 3);
    R_radar_.fill(0.0);
    R_radar_(0, 0) = std_radr_ * std_radr_;
    R_radar_(1, 1) = std_radphi_ * std_radphi_;
    R_radar_(2, 2) = std_radrd_ * std_radrd_;


    R_lidar_ = MatrixXd(2, 2);
    R_lidar_.fill(0.0);
    R_lidar_(0, 0) = std_laspx_ * std_laspx_;
    R_lidar_(1, 1) = std_laspy_ * std_laspy_;
    is_initialized_ = false;

}

UKF::~UKF()
{

}

void UKF::ProcessMeasurement(MeasurementPackage &meas_package) {
    /**
     * TODO: Complete this function! Make sure you switch between lidar and radar
     * measurements.
     */
    std::cout << "UKF::ProcessMeasurement" << std::endl;
    currentPackage_ = &meas_package;

    if(!currentPackage_)
        throw std::invalid_argument("received unvalid meas_package");

    // x_ is [px, py, vel, ang, ang_rate]
    if (!is_initialized_)
        initialize();

    // compute delta t
    double dt = computeDeltaT();

    // predict step
    Prediction(dt);
    // update step

    switch (currentPackage_->sensor_type_)
    {
        case (MeasurementPackage::LASER):
        {
            if(use_laser_)
            {
                ukfLidar_.setMainUKF(Xsig_pred_,R_lidar_,n_x_,x_,weights_,P_);
                ukfLidar_.updateLidar(currentPackage_);
                ukfLidar_.updateMainUKF(n_aug_,Xsig_pred_,R_lidar_,n_x_,x_,weights_,P_,nis_lidar_);
            }

            break;
        }
        case (MeasurementPackage::RADAR):
        {
            if(use_radar_)
            {
                ukfRadar_.setMainUKF(Xsig_pred_,R_radar_,n_x_,x_,weights_,P_,n_aug_);
                ukfRadar_.updateRadar(currentPackage_);
                ukfRadar_.updateMainUKF(n_aug_,Xsig_pred_,R_radar_,n_x_,x_,weights_,P_,nis_radar_);
            }

            break;
        }
    }
}

void UKF::Prediction(double delta_t)
{
    /**
     * TODO: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */

    std::cout << "UKF::Prediction" << std::endl;
    // create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug.fill(0.0);

    // create augmented mean state
    x_aug.head(n_x_) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; ++i) {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }

    // predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        // extract values for better readability
        const double p_x = Xsig_aug(0, i);
        const double p_y = Xsig_aug(1, i);
        const double v = Xsig_aug(2, i);
        const double yaw = Xsig_aug(3, i);
        const double yawd = Xsig_aug(4, i);
        const double nu_a = Xsig_aug(5, i);
        const double nu_yawdd = Xsig_aug(6, i);

        // predicted state values
        double px_p, py_p;

        // avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        } else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        // add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        // write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }

    //create vector for predicted state
    // create covariance matrix for prediction

    x_.fill(0.0);
    for(size_t a = 0; a < 2 * n_aug_ + 1; ++a)
        x_ = x_ + weights_(a) * Xsig_pred_.col(a);

    P_.fill(0.0);


    for(size_t b = 0; b < 2 * n_aug_ + 1; ++b)
    {
        VectorXd x_diff = Xsig_pred_.col(b) - x_;

        while (x_diff(3) > M_PI)
            x_diff(3) -= 2. * M_PI;

        while (x_diff(3) < -M_PI)
            x_diff(3) += 2. * M_PI;

        P_ = P_ + weights_(b) * x_diff * x_diff.transpose();
    }
}

void UKF::initialize()
{
    std::cout << "UKF::initialize" << std::endl;
    switch (currentPackage_->sensor_type_)
    {
        case (MeasurementPackage::LASER):
        {
            initLidar();
            break;
        }
        case (MeasurementPackage::RADAR):
        {
            initRadar();
            break;
        }
    }

    time_us_ = currentPackage_->timestamp_;
    is_initialized_ = true;
}

void UKF::initLidar()
{
    std::cout << "UKF::initLidar" << std::endl;
    x_ << currentPackage_->raw_measurements_(0), currentPackage_->raw_measurements_(1), 0., 0, 0;
    P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
    0, std_laspy_ * std_laspy_, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1;
}

void UKF::initRadar()
{
    std::cout << "UKF::initRadar" << std::endl;
    const double rho = currentPackage_->raw_measurements_(0);
    const double phi = currentPackage_->raw_measurements_(1);
    const double rho_d = currentPackage_->raw_measurements_(2);
    double p_x = rho * cos(phi);
    double p_y = rho * sin(phi);
    // To note: this is inaccurate value, aim to initialize velocity which's magnitude/order is close to real value
    double vx = rho_d * cos(phi);
    double vy = rho_d * sin(phi);
    double v = sqrt(vx * vx + vy * vy);

    x_ << p_x, p_y, v,0, 0;
    P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
            0, std_radr_ * std_radr_, 0, 0, 0,
            0, 0, std_radrd_ * std_radrd_, 0, 0,
            0, 0, 0, std_radphi_, 0,
            0, 0, 0, 0, std_radphi_;

}

double UKF::computeDeltaT()
{
    std::cout << "UKF::computeDeltaT" << std::endl;
    double dT = (currentPackage_->timestamp_ - time_us_)/1000000.0;
    time_us_ = currentPackage_->timestamp_;
    return dT;
}




