#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  //=============================================
  // Original Variables

  // Number of sigma points
  int n_sigma_points_;

  // Latest NIS value
  double NIS_laser_;
  double NIS_radar_;


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
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


  //=============================================
  // Original Functions
  void GenerateSigmaPoints(double std_a, double std_yawdd, MatrixXd* Xsig_out);
  void SigmaPointPrediction(double delta_t,
    MatrixXd &Xsig_aug, MatrixXd* Xsig_out);
  void PredictMeanAndCovariance(MatrixXd &Xsig_pred,
    VectorXd* x_pred, MatrixXd* P_pred);
  void PredictLidarMeasurement(double std_laspx, double std_laspy,
    MatrixXd &Xsig_pred, MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out);
  void PredictRadarMeasurement(double std_radr, double std_radphi,
    double std_radrd, MatrixXd &Xsig_pred, MatrixXd* Zsig_out, VectorXd* z_out, MatrixXd* S_out);
  void UpdateState(MeasurementPackage::SensorType sensor_type, MatrixXd &Xsig_pred,
    MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, VectorXd &z, VectorXd* x_out, MatrixXd* P_out);

};

#endif /* UKF_H */
