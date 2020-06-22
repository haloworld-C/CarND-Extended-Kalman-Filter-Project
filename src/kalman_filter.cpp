#include "kalman_filter.h"
#include "tools.h"
#include<math.h>
#include<iostream>

using std::cout;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd F_t = F_.transpose();
  P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  // by haloworld
  y_ = z - H_ * x_;
  
  H_t = H_.transpose();
  S_ = H_ * P_ * H_t + R_;
  S_i = S_.inverse();
  K_ = P_ * H_t * S_i;
  KH = K_ * H_;
  // int r = KH.rows();
  // int c = KH.cols();
  I_ = MatrixXd::Identity(4, 4);
  x_ = x_ + K_ * y_;
  P_ = (I_ - KH) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    //calculator taylor linear
	Tools tools;
	MatrixXd H_L = tools.CalculateJacobian(x_);
	// caculator EKF update
	VectorXd y_ = z - H_L * x_;

	H_lt = H_lt.transpose();
  S_ = H_ * P_ * H_lt + R_;
  S_i = S_.inverse();
  K_ = P_ * H_lt * S_i;
  KH = K_ * H_lt;
  // int r = KH.rows();
  // int c = KH.cols();
  I_ = MatrixXd::Identity(4, 4);
  x_ = x_ + K_ * y_;
  P_ = (I_ - KH) * P_;
}
