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
    R_radar_ << 0.0225, 0, 0,
            0, 0.0225, 0,
            0, 0, 0.0225;

    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    //state covariance matrix P
    MatrixXd P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    MatrixXd F_ = MatrixXd(4, 4);
    MatrixXd Q_ = MatrixXd(4, 4);

    VectorXd x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;

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

        // first measurement
        //cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            float x_cart = (float)(measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]));
            float y_cart = (float)(measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]));

        // Test if there is no null values
        if (x_cart == 0 or y_cart == 0){
            return;
        }


        ekf_.x_ << x_cart, y_cart, 0, 0;
    }
    
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        if (measurement_pack.raw_measurements_[0] == 0 or measurement_pack.raw_measurements_[1] == 0){
            return;
        }

        ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

    //cout << "Current " << measurement_pack.timestamp_ << endl; 
    //cout << "Previous " << previous_timestamp_ << endl; 

    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds

    cout << "dt = " << dt << endl; 

    if(dt > 0){

        ekf_.F_ <<  1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;

        ekf_.Q_ <<  pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
                0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
                pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0,
                0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

        ekf_.Predict();
    }

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