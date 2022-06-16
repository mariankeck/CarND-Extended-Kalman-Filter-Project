#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + K * y;
  // P_ = (I_ - K * H_) * P_;
  P_ -= K * H_ * P_;  // Shorter form as above, feedback from reviewer
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Calculate measurement function h
  float rho = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
  float phi = atan2(x_[1], x_[0]);
  float rho_dot = fabs(rho) < 0.0001 ? 0 : (x_[0] * x_[2] + x_[1] * x_[3]) / rho;

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  // Ensure that the phi error is between +/-pi
  while (y[1] < -M_PI) y[1] += 2 * M_PI;
  while (y[1] > M_PI) y[1] -= 2 * M_PI;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();

  x_ = x_ + K * y;
  P_ -= K * H_ * P_;
}
