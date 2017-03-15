#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Xk = A * X(k-1) + B * U(k-1) + wk


/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    previous_timestamp_ = 0.0;

     // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    //state covariance matrix P
    MatrixXd P_ = MatrixXd(4, 4);
    P_ <<   1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    MatrixXd F_ = MatrixXd(4, 4);
    MatrixXd Q_ = MatrixXd(4, 4);

    // Vector state, unknow at the beginning
    VectorXd x_ = VectorXd(4);

    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

    // Acceleration noise component
    noise_ax = 5;
    noise_ay = 5;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

    if (!is_initialized_) {

        // first measurement, we do not now px and py (position x and y), neither vx and vy
        float px = 0; 
        float py = 0;
        float vx = 0; 
        float vy = 0; 

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            float rho = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];

            px = rho * cos(phi);
            py = rho * sin(phi);
        }
    
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];
        }

    ekf_.x_ << px, py, vx, vy;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds

    ekf_.F_ <<  1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;

    ekf_.Q_ <<  pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
                0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
                pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0,
                0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

    ekf_.Predict();

    // Update timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.R_ = R_radar_;
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);    
  } else {
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}