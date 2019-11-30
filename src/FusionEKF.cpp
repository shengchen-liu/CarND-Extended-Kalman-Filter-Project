#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Initializing P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

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
    cout << "EKF: " << endl;
    VectorXd x_ = VectorXd(4);

    // state covariance matrix (initial velocity is unknown, hence the level of uncertainty is high)
    MatrixXd P(4, 4);
    P << 1, 0,    0,    0,
         0, 1,    0,    0,
         0, 0, 1000,    0,
         0, 0,    0, 1000;

    // state transition matrix (initially Δt is 0)
    MatrixXd F(4, 4);
    F << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // process covariance matrix (initially Δt is 0, hence Q consists of 0's;
    //                            Eigen initializes matrices with 0's by default)
    MatrixXd Q(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      cout << "EKF : First measurement RADAR" << endl;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = measurement_pack.raw_measurements_[0]; // range
  	  double phi = measurement_pack.raw_measurements_[1]; // bearing
  	  // Coordinates convertion from polar to cartesian
  	  // double x = rho * cos(phi);
      // if ( x < 0.0001 ) {
      //   x = 0.0001;
      // }
  	  // double y = rho * sin(phi);
      // if ( y < 0.0001 ) {
      //   y = 0.0001;
      // }
  	  // double vx = rho_dot * cos(phi);
  	  // double vy = rho_dot * sin(phi);
      double x = rho * cos(phi);
      double y = rho * sin(phi);
      // although radar gives velocity data in the form of the range rate rho dot​, 
      // a radar measurement does not contain enough information to determine the state variable velocities vx and vy
      double vx = 0;
      double vy = 0;
      x_ << x, y, vx , vy;
      ekf_.Init(x_, P, F, H_laser_, R_radar_, Q);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      cout << "EKF : First measurement LASER" << endl;
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      ekf_.Init(x_, P, F, H_laser_, R_laser_, Q);

    }

    previous_timestamp_ = measurement_pack.timestamp_;
    
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

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // State transition matrix update
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Set process Noise covariance matrix computation
  // Noise values from the task
  double noise_ax = 9.0;
  double noise_ay = 9.0;

  double dt_2 = dt * dt; //dt^2
  double dt_3 = dt_2 * dt; //dt^3
  double dt_4 = dt_3 * dt; //dt^4
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax, 0,
	         0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
	         dt_3/2 * noise_ax, 0, dt_2 * noise_ax, 0,
 	         0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;

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
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.R_ = R_radar_;
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
