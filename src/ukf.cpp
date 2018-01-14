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
  // switching flags
  use_laser_ = true;
  use_radar_ = true;
  
  // state and augmented state dimensions
  n_x_ = 5;
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd::Constant(n_x_, 1);

  // initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.1;
  
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
  
  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized = false;
 
  time_us_ = 0.0;
  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;
  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  // the current NIS for radar and laser
  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /* 
  Initialization :
    As stated in the comments, the sensor switch only works
  after initialization  
  */
  if (!is_initialized){
      // x_ has been initialized with constant values in the constructor
      // P has also been initialized in the constructor
      
      time_us_ = meas_package.timestamp_;
      // Finally initializing the state vector with measurements
      if (meas_package.sensor_type_ == MeasurementPackage::LASER){
          x_(0) = meas_package.raw_measurements_(0);
          x_(1) = meas_package.raw_measurements_(1);
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
          float rho = meas_package.raw_measurements_(0);
          float phi = meas_package.raw_measurements_(1);
          float rho_d = meas_package.raw_measurements_(2);
          
          x_(0) = rho * cos(phi);
          x_(1) = rho * sin(phi);

      }
      is_initialized = true;
      return;
  }
  
  // Prediction + Update Steps
  
 float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
 time_us_ = meas_package.timestamp_;
 
 Prediction(dt);
 
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
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
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
