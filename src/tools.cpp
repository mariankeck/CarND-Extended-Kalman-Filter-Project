#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, 
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check the validity of the inputs
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground truth data" << endl;
    return rmse;
  }

  // Accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // Coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the mean and square root
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x) {
  MatrixXd Hj(3, 4);
  
  // Get state parameters
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // Check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "Error calculating Jacobian - Division by Zero" << endl;
    return Hj;
  }

  // Calculate the Jacobian matrix
  Hj << px / c2, py / c2, 0, 0,
        -py / c1, px / c1, 0, 0,
        py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, 
          px / c2, py / c2;

  return Hj;
}
