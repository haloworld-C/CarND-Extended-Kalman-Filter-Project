#include "kalman_filter.h"
#include "tools.h"
#include<math.h>

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
  VectorXd y_ = z - H_ * x_;
  
  MatrixXd H_t = H_.transpose();
  MatrixXd S_ = H_ * P_ * H_t + R_;
  MatrixXd S_i = S_.inverse();
  MatrixXd K_ = P_ * H_t * S_i;
  MatrixXd KH = K_ * H_;

  MatrixXd I_ = MatrixXd::Identity(4, 4);
  x_ = x_ + K_ * y_;
  P_ = (I_ - KH) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    //calculator taylor linear
	Tools tools;
	H_j = tools.CalculateJacobian(x_);
  // transfer xy axis coordinate to polar coordinate
  float rho = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  float theta = atan2(x_[1],x_[0]);
  float rho_dot = 0;
  if(fabs(rho) > 0.0001){
    rho_dot = (x_[0]*x_[2] + x_[1]*x_[3])/rho;
  }
	// caculator EKF update
  VectorXd Hx(3);
  Hx << rho, theta, rho_dot;
	VectorXd y_ = z - Hx;

  if(y_[1] > M_PI){
    y_[1] -= 2*M_PI;
  }
  if(y_[1] < -M_PI){
    y_[1] += 2*M_PI;
  }

	MatrixXd H_jt = H_j.transpose();
  MatrixXd S_ = H_j * P_ * H_jt + R_;
  MatrixXd S_i = S_.inverse();
  MatrixXd K_ = P_ * H_jt * S_i;
  MatrixXd KH = K_ * H_j;

  MatrixXd I_ = MatrixXd::Identity(4, 4);
  x_ = x_ + K_ * y_;
  P_ = (I_ - KH) * P_;
}
