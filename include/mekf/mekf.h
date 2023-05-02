// Multiplicative Extended Kalman Filter (MEKF) Header File

#ifndef MEKF_H
#define MEKF_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

using namespace std;
using namespace Eigen;

class MEKF
{
public:
    // Class Constructor
    MEKF(double gyro_noise, double accel_noise, double mag_noise);

    // Attitude Variables
    Vector4d q; // quaternion vector

    // Methods
    void predict(double gyro[3], double dt);
    // void time_update(double dt);
    void update(double (&q_orient)[4], double accel[3], double mag[3]);
    void filter_update(double (&q_orient)[4], double gyro[3], double accel[3], double mag[3], double dt);

private:
    // Noise Parameter Variables
    double g_noise, a_noise, m_noise; // squared noise values
    Vector3d proc_noise;              // process noise vector
    Matrix<double, 6, 1> meas_noise;  // measurement noise vector

    Matrix<double, 3, 3> Q; // process noise covariance matrix
    Matrix<double, 6, 6> R; // measurement noise covariance matrix

    // Global Parameter Variables
    Vector3d g;           // Normalized gravity vector
    Vector3d mag_ref;     // Normalized mag 

    // Propagation Variables
    Matrix4d A; // state transition matrix

    // Time Update Variables
    Matrix3d P; // state estimate covariance matrix
    Matrix3d C; // rotation matrix from body to nav frame
    Matrix3d W; // jacobian of state estimate

    // Measurement Update Variables
    Vector3d a; // Acceleration measurement
    Vector3d m; // Magnetic field measurement
    Vector3d magcal_offset; // Magnetic field calibration offset
    Matrix<double,3,3> magcal_rotate; // Magnetic field calibration rotation
    Matrix<double, 6, 1> z; // Measurement vector
    
    Matrix<double, 6, 1> h; // predicted state
    Matrix<double, 6, 1> y; // innovation/residual
    Matrix<double, 6, 3> H; // measurement sensitivity matrix. See Markley 2003, Utrera 2021

    Matrix<double, 6, 6> S; // covariance of innovation/residual
    Matrix<double, 3, 6> K; // Kalman gain

    Vector3d err_x; // Error State
    Vector4d err_q; // Error Quaternion

    // Methods

    // Multiply two quaternions together
    Vector4d quat_multiply(Vector4d a, Vector4d b);
    // Create skew-symmetric matrix
    Matrix3d skew(Vector3d x);
};

#endif