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

    // Class Functions & Methods
    void propagate(double gyro[3], double dt);
    void time_update(double dt);
    void measurement_update(double (&q_orient)[4], double accel[3], double mag[3]);
    void filter_update(double (&q_orient)[4], double gyro[3], double accel[3], double mag[3], double dt);

private:
    // Noise Parameter Variables
    double g_noise, a_noise, m_noise; // squared noise values
    Vector3d proc_noise;              // process noise vector
    Matrix<double, 6, 1> meas_noise;  // measurement noise vector

    Matrix<double, 3, 3> Q; // process noise covariance matrix
    Matrix<double, 6, 6> R; // measurement noise covariance matrix

    // Global Parameter Variables
    Vector3d g;           // gravity vector
    Vector3d nav_mag;     // mag measurement in nav frame
    // Vector3d expected_mag;   // mag at measurement inclination

    // Propagation Variables
    Matrix4d A; // state transition matrix

    // Time Update Variables
    Matrix3d P; // state estimate covariance matrix
    Matrix3d C; // rotation matrix from body to nav frame
    Matrix3d W; // jacobian of state estimate

    // Measurement Update Variables
    Vector3d a;
    Vector3d m;
    Matrix<double, 6, 1> z; // actual measurements

    Matrix<double, 6, 1> h; // predicted measurements
    Matrix<double, 6, 1> y; // measurement innovation/residual
    Matrix<double, 6, 3> H; // observation matrix

    Matrix<double, 6, 6> S; // covariance of measurement innovation/residual
    Matrix<double, 3, 6> K; // Kalman gain

    Vector3d err_x; // Error State
    Vector4d err_q; // Error Quaternion

    // Class Functions & Methods
    Vector4d quat_multiply(Vector4d a, Vector4d b);
    double norm(double a, double b);
    Matrix3d skew(Vector3d x);
};

#endif