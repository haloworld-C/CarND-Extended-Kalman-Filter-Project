#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);
  // Hj_ = MatrixXd(3, 4);
  // set F
  ekf_.F_ = MatrixXd::Identity(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {

}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    // cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      // float rho_dot = measurement_pack.raw_measurements_[2];

      ekf_.x_[0] = rho * cos(theta);
      ekf_.x_[1] = rho * sin(theta);
      // why not set velocity
      // ekf_.x_[2] = rho_dot * cos(theta);
      // ekf_.x_[3] = rho_dot * sin(theta);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      float dot_x = measurement_pack.raw_measurements_[0];
      float dot_y = measurement_pack.raw_measurements_[1];
      ekf_.x_[0] = dot_x;
      ekf_.x_[1] = dot_y;
      // ekf_.x_[2] = 0;
      // ekf_.x_[3] = 0;

    }
    
    // initial time
    this->previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float delta_t = (measurement_pack.timestamp_ - this->previous_timestamp_)/ 1000000.0; //caculate delta_t and transfered in second
  previous_timestamp_ = measurement_pack.timestamp_;
  //initial ekf matrix
  float dt_2 = delta_t * delta_t;
  float dt_3 = dt_2 * delta_t;
  float dt_4 = dt_3 * delta_t;

  // set matrix F
  ekf_.F_(0, 2) = delta_t;
  ekf_.F_(1, 3) = delta_t;
   // set matrix Q
  float noise_ax = 9;
  float noise_ay = 9;
  float noise_ax2 = noise_ax * noise_ax;
  float noise_ay2 = noise_ay * noise_ay;
  
  ekf_.Q_ << dt_4 * noise_ax2 /4, 0, dt_3 * noise_ax2 /2, 0,
             0, dt_4 * noise_ay2 / 2, 0, dt_3 * noise_ay2 /2,
             dt_3 * noise_ax2 /2, 0, dt_2 *noise_ax2, 0,
             0, dt_3 * noise_ay2 /2, 0, dt_2 *noise_ay2;


  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);


  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
