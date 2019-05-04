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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  ///* time when the state is true, in us
  time_us_ = 0;

  ///* State dimension
  n_x_ = x_.size();

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  ///* Weights of sigma points
  weights_ = VectorXd(n_sig_);

  ///* Normalized Innovation Squared (NIS)
  nis_lidar_ = 0.0;
  nis_radar_ = 0.0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    //cout << "Initialization: " << endl;

    double var_px = 1.0;
    double var_py = 1.0;
    double var_v = 0.35;
    double var_psi = 0.01;
    double var_psidot = 0.40;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[2];
      //float vx = rho_dot * cos(phi);
      //float vy = rho_dot * sin(phi);
      x_[0] = rho * cos(phi);   // px
      x_[1] = rho * sin(phi);   // py
      x_[2] = 5;                // v
      x_[3] = 0;                // psi
      x_[4] = 0;                // psi_dot
      // variance
      var_px = std_laspx_ * std_laspx_;
      var_py = std_laspy_ * std_laspy_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 5, 0, 0;
      // variance
      var_px = std_radr_ * std_radr_; //cos(std_radphi_);
      var_py = std_radr_ * std_radr_; //sin(std_radphi_);
    }

    //state covariance matrix P
    P_ << var_px,      0,     0,       0,          0,
               0, var_py,     0,       0,          0,
               0,      0, var_v,       0,          0,
               0,      0,     0, var_psi,          0,
               0,      0,     0,       0, var_psidot;

    //store time stamp of measurement in micro seconds
    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  //predict
  //cout << "Predict: " << endl;
  Prediction(delta_t);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  //cout << "Update: " << endl;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    if (use_radar_) {
      UpdateRadar(meas_package);
    }
  } else {
    // Laser updates
    if (use_laser_) {
      UpdateLidar(meas_package);
    }
  }

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
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

  /* Generate sigma points
  ****************************************************************************/
  //cout << "Generate sigma points: " << endl;

  //create augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
  Xsig_aug.col(0) = x_aug;

  //set remaining sigma points
  for (int j = 0; j < n_aug_; j++)  {
    Xsig_aug.col(j+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(j);
    Xsig_aug.col(j+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(j);
  }

  /* Predict augmented sigma points
  ****************************************************************************/
  //cout << "Predict augmented sigma points: " << endl;
  VectorXd noise = VectorXd(n_x_);

  // iterate over sigma points
  for (int j = 0; j < n_sig_; j++) {
    // get current sigma point
    double p_x = Xsig_aug(0,j);
    double p_y = Xsig_aug(1,j);
    double v = Xsig_aug(2,j);
    double psi = Xsig_aug(3,j);
    double psi_dot = Xsig_aug(4,j);
    double ny_a = Xsig_aug(5,j);
    double ny_psi_dot = Xsig_aug(6,j);
    // do some pre-calculations
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);
    double half_dt_squared = 0.5 * delta_t * delta_t;
    double t1 = half_dt_squared * ny_a;
    // calculate noise
    noise(0) = t1 * cos_psi;
    noise(1) = t1 * sin_psi;
    noise(2) = delta_t * ny_a;
    noise(3) = half_dt_squared * ny_psi_dot;
    noise(4) = delta_t * ny_psi_dot;
    // predict sigma points - avoid division by zero
    if (fabs(psi_dot) >= 0.00001) {
      double t2 = v/psi_dot;
      double psi_dot_dt = psi_dot * delta_t;
      double t3 = psi + psi_dot_dt;
      Xsig_pred_(0,j) = p_x + t2 * (sin(t3) - sin_psi) + noise(0);
      Xsig_pred_(1,j) = p_y + t2 * (cos_psi - cos(t3)) + noise(1);
      Xsig_pred_(2,j) = v + noise(2);
      Xsig_pred_(3,j) = psi + psi_dot_dt + noise(3);
      Xsig_pred_(4,j) = psi_dot + noise(4);
    } else {
      Xsig_pred_(0,j) = p_x + v * cos_psi * delta_t + noise(0);
      Xsig_pred_(1,j) = p_y + v * sin_psi * delta_t + noise(1);
      Xsig_pred_(2,j) = v + noise(2);
      Xsig_pred_(3,j) = psi + noise(3);
      Xsig_pred_(4,j) = noise(4);
    }
  }

  /* Predict mean and covariance
  ****************************************************************************/
  //cout << "Predict mean and covariance: " << endl;

  //set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  weights_(1) = 0.5 / (lambda_ + n_aug_);
  for (int j = 2; j < n_sig_; j++) {
    weights_(j) = weights_(1);
  }

  //predict state mean
  x_ = weights_(0) * Xsig_pred_.col(0);
  //iterate over sigma points
  for (int j = 1; j < n_sig_; j++) {
    x_ += weights_(j) * Xsig_pred_.col(j);
  }

  //predict state covariance matrix
  P_.fill(0.0);
  //iterate over sigma points
  for (int j = 0; j < n_sig_; j++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(j) - x_;
    //angle normalization
    while ( x_diff(3) >  M_PI ) x_diff(3) -= 2. * M_PI;
    while ( x_diff(3) < -M_PI ) x_diff(3) += 2. * M_PI;
    //predict state covariance
    P_ += weights_(j) * x_diff * x_diff.transpose() ;
  }
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

  int n_z = 2;

  /* Predict measurement
  ****************************************************************************/

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  Zsig = Xsig_pred_.topRows(n_z);    // px & py

  //calculate mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred = Zsig * weights_;

  //calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.);
  //iterate over sigma points
  for (int j = 0; j < n_sig_; j++) {
    // residual
    VectorXd z_diff = Zsig.col(j) - z_pred;
    //calculate innovation covariance
    S += weights_(j) * z_diff * z_diff.transpose();
  }

  // determine measurement noise covariance
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_;

  S += R;

  /* Update state
  ****************************************************************************/

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  //iterate over sigma points
  for (int j = 0; j < n_sig_; j++) {
    // residual
    VectorXd z_diff = Zsig.col(j) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(j) - x_;
    //angle normalization
    while ( x_diff(3) >  M_PI ) x_diff(3) -= 2. * M_PI;
    while ( x_diff(3) < -M_PI ) x_diff(3) += 2. * M_PI;
    //calculate cross correlation
    Tc += weights_(j) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = Tc * S_inv;

  //update state mean and covariance matrix
  // residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  /* Calculate Normalized Innovation Squared (NIS)
  ****************************************************************************/

  nis_lidar_ = z_diff.transpose() * S_inv * z_diff;
  //cout << "NIS_lidar: " << nis_lidar_ << endl;

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
  int n_z = 3;

  /* Predict measurement
  ****************************************************************************/

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  //transform sigma points into measurement space
  //iterate over sigma points
  for (int j = 0; j < n_sig_; j++) {
    double px = Xsig_pred_(0,j);
    double py = Xsig_pred_(1,j);
    double v = Xsig_pred_(2,j);
    double psi = Xsig_pred_(3,j);
    // convert to polar coordinates
    double rho = sqrt(px * px + py * py);
    Zsig(0,j) = rho;
    Zsig(1,j) = atan2(py, px);            // phi
    if (fabs(rho) < 0.0001) {
        Zsig(2,j) = 0;                    // rho_dot
    } else {
        Zsig(2,j) = (px * cos(psi) * v + py * sin(psi) * v) / rho;
    }
  }

  //calculate mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred = Zsig * weights_;

  //calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.);
  //iterate over sigma points
  for (int j = 0; j < n_sig_; j++) {
    // residual
    VectorXd z_diff = Zsig.col(j) - z_pred;
    //angle normalization
    while ( z_diff(1) >  M_PI ) z_diff(1) -= 2.*M_PI;
    while ( z_diff(1) < -M_PI ) z_diff(1) += 2.*M_PI;
    //calculate innovation covariance
    S += weights_(j) * z_diff * z_diff.transpose();
  }

  // determine measurement noise covariance
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;

  S += R;

  /* Update state
  ****************************************************************************/

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  //iterate over sigma points
  for (int j = 0; j < n_sig_; j++) {
    // residual
    VectorXd z_diff = Zsig.col(j) - z_pred;
    //angle normalization
    while ( z_diff(1) >  M_PI ) z_diff(1) -= 2. * M_PI;
    while ( z_diff(1) < -M_PI ) z_diff(1) += 2. * M_PI;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(j) - x_;
    //angle normalization
    while ( x_diff(3) >  M_PI ) x_diff(3) -= 2. * M_PI;
    while ( x_diff(3) < -M_PI ) x_diff(3) += 2. * M_PI;
    //calculate cross correlation
    Tc += weights_(j) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = Tc * S_inv;

  //update state mean and covariance matrix
  // residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  //angle normalization
  while ( z_diff(1) >  M_PI ) z_diff(1) -= 2. * M_PI;
  while ( z_diff(1) < -M_PI ) z_diff(1) += 2. * M_PI;

  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  /* Calculate Normalized Innovation Squared (NIS)
  ****************************************************************************/

  nis_radar_ = z_diff.transpose() * S_inv * z_diff;
  //cout << "NIS_radar: " << nis_radar_ << endl;

}
