#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

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
  is_initialized_ = false;
 
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
  
  Tools tools_;
  
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
  if (!is_initialized_){
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
      is_initialized_ = true;
      return;
  }
  
  // Prediction + Update Steps
  
 float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
 time_us_ = meas_package.timestamp_;
 
 Prediction(dt);
 std::cout << "Prediction..." << std::endl;
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
      std::cout << "Lidar..." << std::endl;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
      std::cout << "Radar..." << std::endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    
  //create sigma point
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  GenerateSigmaPoints(P_, x_, &Xsig);
  
  // Augmented state sigma points
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  for (int i = 1; i <= n_aug_ - n_x_; i++){
    x_aug(i) = 0;
  }
  
  MatrixXd P_aug = MatrixXd::Constant(n_aug_, n_aug_, 0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_ + 0, n_x_ + 0) = std_a_*std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_*std_yawdd_;
  
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  GenerateSigmaPoints(P_aug, x_aug, &Xsig_aug);
  MapSigmaPoints(Xsig_aug, &Xsig_pred_, delta_t);
  
  UpdateWeights();
  UpdateStateMean(&x_, Xsig_pred_);
  UpdateCovarianceMatrix(&P_, Xsig_pred_, x_, 3, Xsig_pred_, x_, 3);
  
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    
    VectorXd z = meas_package.raw_measurements_;
     //set measurement dimension, lidar can measure p_x and p_y
    int n_z = 2;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);

        Zsig(0, i) = p_x;
        Zsig(1, i) = p_y;
    }
    
    R_lidar_ = MatrixXd(n_z, n_z);
    R_lidar_ << std_laspx_*std_laspx_, 0,
                0, std_laspy_*std_laspy_;
    
    NIS_laser_ = Update(z, Zsig, n_z, -1, R_lidar_);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  
  VectorXd z = meas_package.raw_measurements_;
  
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v   = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y); 
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);
  }
  
  R_radar_ = MatrixXd(n_z, n_z);
  R_radar_ << std_radr_*std_radr_, 0,0,
       0, std_radphi_*std_radphi_,0,
       0,0, std_radrd_*std_radrd_;
  
  NIS_radar_ = Update(z, Zsig, n_z, 1, R_radar_);
  
}

void UKF::GenerateSigmaPoints(MatrixXd P, VectorXd x, MatrixXd* Xsig_out) {

  //set state dimension
  int n_x = x.size();
  
  //define spreading parameter
  double lambda = 3 - n_x;

  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  //calculate square root of P
  MatrixXd A = P.llt().matrixL();

  //set first column of sigma point matrix
  Xsig.col(0)  = x;

  //set remaining sigma points
  for (int i = 0; i < n_x; i++)
  {
    Xsig.col(i+1)     = x + sqrt(lambda+n_x) * A.col(i);
    Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * A.col(i);
  }

  //write result
  *Xsig_out = Xsig;
}

void UKF::MapSigmaPoints(MatrixXd Xin, MatrixXd* Xmap, double dt) {
   
   MatrixXd Xout = MatrixXd::Constant(Xmap->rows(), Xmap->cols(), 0);
   
   if (Xout.cols() != Xin.cols() || Xout.rows() >= Xin.rows()){
       std::cout << "ERROR : Dimension mismatch" << std::endl;
       std::cout << "(" << Xin.rows() << ", " << Xin.cols() << ")" << std::endl;
       std::cout << "(" << Xout.rows() << ", " << Xout.cols() << ")" << std::endl;
       
       return;
   }
   
   for (int i = 0; i < Xin.cols(); i++){
       /* It's hardcoded for this problem, but if one wants to
       reuse this module, this function needs to be overwritten
       */
       double p_x      = Xin(0, i);
       double p_y      = Xin(1, i);
       double v        = Xin(2, i);
       double yaw      = Xin(3, i);
       double yawd     = Xin(4, i);
       double nu_a     = Xin(5, i);
       double nu_yawdd = Xin(6, i);
       
       // states
       double px_p, py_p;
       double v_p = v;
       double yaw_p = yaw + yawd * dt;
       double yawd_p = yawd;
       
       double eps = 1E-3;

       if (fabs(yawd) < eps) {
          px_p = p_x + v * dt * cos(yaw);
          py_p = p_y + v * dt * sin(yaw);
        }else{
            px_p = p_x + v / yawd * (sin(yaw + yawd * dt) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * dt));
        }
        
        px_p += 0.5 * nu_a * dt * dt * cos(yaw);
        py_p += 0.5 * nu_a * dt * dt * sin(yaw);
        
        v_p += nu_a*dt;
        yaw_p += 0.5*nu_yawdd*dt*dt;
        yawd_p += nu_yawdd*dt;

        //write predicted sigma point into right column
        Xout(0, i) = px_p;
        Xout(1, i) = py_p;
        Xout(2, i) = v_p;
        Xout(3, i) = yaw_p;
        Xout(4, i) = yawd_p;
  }
  *Xmap = Xout;
}

void UKF::UpdateWeights(){
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
    }
}

void UKF::UpdateStateMean(VectorXd* x, MatrixXd Xsig){
    VectorXd xx( x->size());
    
    xx.fill(0.0);
    
    for (int i = 0; i < Xsig.cols(); i++) {
        xx = xx + weights_(i) * Xsig.col(i);
    }   
    
    *x = xx;
}

void UKF::UpdateCovarianceMatrix(MatrixXd* Pout, MatrixXd Xsig_pred,VectorXd x, int x_ang_row, MatrixXd Zsig_pred,VectorXd z, int z_ang_row){
    
    MatrixXd P = MatrixXd::Constant(Pout->rows(), Pout->cols(), 0);
    
    for (int i = 0; i < Xsig_pred.cols(); i++) {
        // Residuals computation
        VectorXd x_diff = Xsig_pred.col(i) - x; 
        if (x_ang_row > -1){ tools_.normalize_angles(x_diff(x_ang_row));}
        
        VectorXd z_diff = Zsig_pred.col(i) - z; 
        if (z_ang_row > -1){ tools_.normalize_angles(z_diff(z_ang_row));}
        
        P = P + weights_(i) * x_diff * z_diff.transpose();
    }
    *Pout = P;
}

double UKF::Update(VectorXd z, MatrixXd Zsig, int n_z, int z_ang_row, MatrixXd R){
    // Performs the update step and returns NIS
    
    VectorXd z_pred = VectorXd(n_z);
    UpdateStateMean(&z_pred, Zsig);
      
    MatrixXd S = MatrixXd(n_z, n_z);
    UpdateCovarianceMatrix(&S, Zsig, z_pred, z_ang_row, Zsig, z_pred, z_ang_row);

    S = S + R;

    MatrixXd Tc = MatrixXd(n_x_, n_z);
    UpdateCovarianceMatrix(&Tc, Xsig_pred_, x_, 3, Zsig, z_pred, z_ang_row);

    // Finally, updating Kalman Gains and updating the predictions
    MatrixXd K = Tc * S.inverse();

    VectorXd z_diff = z - z_pred;
    if (z_ang_row > -1){tools_.normalize_angles(z_diff(z_ang_row));}

    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
      
    return z_diff.transpose() * S.inverse() * z_diff;
}