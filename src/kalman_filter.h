#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  KalmanFilter();

  virtual ~KalmanFilter();

  /**
   * Prediction of the state and the state covariance using the process model
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  // State vector
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // State transition matrix
  Eigen::MatrixXd F_;

  // Process covariance matrix
  Eigen::MatrixXd Q_;

  // Measurement matrix
  Eigen::MatrixXd H_;

  // Measurement covariance matrix
  Eigen::MatrixXd R_;
  
  // Identity matrix
  Eigen::MatrixXd I_;
};

#endif // KALMAN_FILTER_H_
