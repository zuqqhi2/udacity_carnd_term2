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
  //std_a_ = 30;
  std_a_ = 0.4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 30;
  std_yawdd_ = 0.4;

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
  is_initialized_ = false;
  n_x_ = 5;   // px, py, v, yaw, yawd
  n_aug_ = 7; // px, py, v, yaw, yawd, nu_x, nu_yawdd
  lambda_ = 3 - n_aug_;

  n_sigma_points_ = 2 * n_aug_ + 1;
  Xsig_pred_ = MatrixXd(n_sigma_points_, n_x_);

  // Initial state
  x_ << 1.0, 1.0, 1.0, 1.0, 0.1;

  // Initial state covariance
  P_ << 0.15,    0, 0, 0, 0,
           0, 0.15, 0, 0, 0,
           0,    0, 1, 0, 0,
           0,    0, 0, 1, 0,
           0,    0, 0, 0, 1;

  // create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  // 2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  // Latest NIS
  NIS_laser_ = 0.0;
  NIS_radar_ = 0.0;
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

  //=============================================
  // Initialization
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double prime_rho = meas_package.raw_measurements_[2];
      x_(0) = rho       * cos(phi);
      x_(1) = rho       * sin(phi);
      x_(2) = prime_rho * cos(phi);
      x_(3) = prime_rho * sin(phi);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
    }

    // done initializing, no need to predict or update
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  //=============================================
  // Prediction Step
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  Prediction(dt);

  //=============================================
  // Update Step
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    UpdateRadar(meas_package);
  } else {
    // Laser updates
    UpdateLidar(meas_package);
  }
  cout << NIS_laser_ << "," << NIS_radar_ << endl;
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
  int n_sigma_points = 2 * n_aug_ + 1;

  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_points);
  GenerateSigmaPoints(std_a_, std_yawdd_, &Xsig_aug);

  SigmaPointPrediction(delta_t, Xsig_aug, &Xsig_pred_);

  PredictMeanAndCovariance(Xsig_pred_, &x_, &P_);
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

  // Predict
  int n_z = 2;
  VectorXd z_out = VectorXd(n_z);
  MatrixXd S_out = MatrixXd(n_z, n_z);
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  PredictLidarMeasurement(std_laspx_, std_laspy_, Xsig_pred_, &Zsig, &z_out, &S_out);

  // Update State
  UpdateState(meas_package.sensor_type_, Xsig_pred_, Zsig, z_out, S_out, meas_package.raw_measurements_, &x_, &P_);
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
  // Predict
  int n_z = 3;
  VectorXd z_out = VectorXd(n_z);
  MatrixXd S_out = MatrixXd(n_z, n_z);
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  PredictRadarMeasurement(std_radr_, std_radphi_, std_radrd_, Xsig_pred_, &Zsig, &z_out, &S_out);

  // Update State
  UpdateState(meas_package.sensor_type_, Xsig_pred_, Zsig, z_out, S_out, meas_package.raw_measurements_, &x_, &P_);
}


void UKF::GenerateSigmaPoints(double std_a, double std_yawdd, MatrixXd* Xsig_out) {
  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a * std_a;
  P_aug(6, 6) = std_yawdd * std_yawdd;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++) {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // write result
  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(double delta_t, MatrixXd &Xsig_aug, MatrixXd* Xsig_out) {
  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0, i) = px_p;
    Xsig_pred(1, i) = py_p;
    Xsig_pred(2, i) = v_p;
    Xsig_pred(3, i) = yaw_p;
    Xsig_pred(4, i) = yawd_p;
  }

  // write result
  *Xsig_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(MatrixXd &Xsig_pred, VectorXd* x_out, MatrixXd* P_out) {
  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    x = x+ weights_(i) * Xsig_pred.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    // angle normalization
    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  // write result
  *x_out = x;
  *P_out = P;
}

void UKF::PredictLidarMeasurement(double std_laspx, double std_laspy,
   MatrixXd &Xsig_pred, MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out) {
  // create matrix for sigma points in measurement space
  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred(0, i);
    double p_y = Xsig_pred(1, i);

    // measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<    std_laspx * std_laspx, 0,
          0,                     std_laspy * std_laspy;
  S = S + R;

  // write result
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}


void UKF::PredictRadarMeasurement(double std_radr, double std_radphi,
   double std_radrd, MatrixXd &Xsig_pred, MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out) {
  // create matrix for sigma points in measurement space
  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred(0, i);
    double p_y = Xsig_pred(1, i);
    double v   = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1, i) = atan2(p_y, p_x);                               // phi
    Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    z_diff(1) = NormalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<    std_radr * std_radr, 0,                       0,
          0,                   std_radphi * std_radphi, 0,
          0,                   0,                       std_radrd * std_radrd;
  S = S + R;

  // write result
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}


void UKF::UpdateState(MeasurementPackage::SensorType sensor_type, MatrixXd &Xsig_pred,
   MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S,
   VectorXd &z, VectorXd* x_out, MatrixXd* P_out) {
  VectorXd x = VectorXd(n_x_);
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // create matrix for cross correlation Tc
  int n_z = 2;
  if (sensor_type == MeasurementPackage::RADAR) {
    n_z = 3;
  }
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    if (sensor_type == MeasurementPackage::RADAR) {
      z_diff(1) = NormalizeAngle(z_diff(1));
    }

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    // angle normalization
    if (sensor_type == MeasurementPackage::RADAR) {
      x_diff(3) = NormalizeAngle(x_diff(3));
    }

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  if (sensor_type == MeasurementPackage::RADAR) {
    z_diff(1) = NormalizeAngle(z_diff(1));
  }

  // Update NIS
  if (sensor_type == MeasurementPackage::RADAR) {
    NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  } else {
    NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
  }

  // update state mean and covariance matrix
  x = x_ + K * z_diff;
  P = P_ - K * S * K.transpose();

  // write result
  *x_out = x;
  *P_out = P;
}

double UKF::NormalizeAngle(double angle) {
  while (angle >  M_PI) angle -= 2. * M_PI;
  while (angle < -M_PI) angle += 2. * M_PI;

  return angle;
}
