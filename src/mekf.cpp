// Multiplicative Extended Kalman Filter (MEKF) Class Implementation

#include "mekf/mekf.h"

MEKF::MEKF(double gyro_noise, double accel_noise, double mag_noise)
{
    // Noise Initialization
    g_noise = pow(gyro_noise, 2); // square imu noises
    a_noise = pow(accel_noise, 2);
    m_noise = pow(mag_noise, 2);

    proc_noise << g_noise, g_noise, g_noise; // noise vectors
    meas_noise << a_noise, a_noise, a_noise, m_noise, m_noise, m_noise;

    // Global Paramter Initialization
    g << 0, 0, -1; //initialize gravity **CHANGED FROM 9.81
    nav_mag << 0.483282, 0, 0.875465; // Computed prior, based on the AHRS github

    // Filter Initialization
    q << 1, 0, 0, 0; // initialize quaternion

    Q = proc_noise.asDiagonal(); // initialize process & measurement noise covariance
    R = meas_noise.asDiagonal();

    W.setIdentity(); // initialize state estimate Jacobian & state estimate covariance
    P = W * Q * W.transpose();
}

void MEKF::propagate(double gyro[3], double dt)
{
    // Eq 2.82 in Howard's Thesis
    A(0, 0) = 1.0; // state transition matrix
    A(0, 1) = -0.5 * gyro[0] * dt;
    A(0, 2) = -0.5 * gyro[1] * dt;
    A(0, 3) = -0.5 * gyro[2] * dt;

    A(1, 0) = 0.5 * gyro[0] * dt;
    A(1, 1) = 1.0;
    A(1, 2) = 0.5 * gyro[2] * dt;
    A(1, 3) = -0.5 * gyro[1] * dt;

    A(2, 0) = 0.5 * gyro[1] * dt;
    A(2, 1) = -0.5 * gyro[2] * dt;
    A(2, 2) = 1.0;
    A(2, 3) = 0.5 * gyro[0] * dt;

    A(3, 0) = 0.5 * gyro[2] * dt;
    A(3, 1) = 0.5 * gyro[1] * dt;
    A(3, 2) = -0.5 * gyro[0] * dt;
    A(3, 3) = 1.0;

    q = A * q; // quaternion attitude propgation (mechanization)
}

void MEKF::time_update(double dt)
{
    P = P + W * Q * W.transpose();

    // Eq 2.33 in Howard's Thesis
    C(0, 0) = pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2);
    C(0, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
    C(0, 2) = 2 * (q(0) * q(2) + q(1) * q(3));

    C(1, 0) = 2 * (q(0) * q(3) + q(1) * q(2));
    C(1, 1) = pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) - pow(q(3), 2);
    C(1, 2) = 2 * (q(2) * q(3) - q(0) * q(1)); // POSSIBLY CHANGE THIS TO HOWARD'S THESIS

    C(2, 0) = 2 * (q(1) * q(3) - q(0) * q(2));
    C(2, 1) = 2 * (q(0) * q(1) + q(2) * q(3));
    C(2, 2) = pow(q(0), 2) - pow(q(1), 2) - pow(q(2), 2) + pow(q(3), 2);

    W = C * dt;
}

void MEKF::measurement_update(double (&q_orient)[4], double accel[3], double mag[3])
{
    a << accel[0], accel[1], accel[2];
    m << mag[0], mag[1], mag[2];
    // Measurements
    // z << a.norm(),
    //     m.norm();
    z<< a.normalized(),
        m.normalized(); 
    // nav_mag << C * m;

    // These are our predictions -> Gravity makes sense to me, but the mag part does not.
    h << C.transpose() * g,
        C.transpose() * nav_mag;

    y = z - h;

    H << C.transpose() * skew(g),
        C.transpose() * skew(nav_mag);

    // Eq 24 in Utrera 2021
    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();

    // Eq 25 in Utrera 2021
    err_x = K * y;

    // Eq 26 in Utrera 2021. Note: The square root in Eq 26 is linearized and simplifies to 1/2.
    err_q << 1, 0.5 * err_x;
    q = quat_multiply(err_q, q).normalized();

    // q = (q + err_x).normalized();

    q_orient[0] = q[0];
    q_orient[1] = q[1];
    q_orient[2] = q[2];
    q_orient[3] = q[3];

    P = P - (K * S * K.transpose());
}

void MEKF::filter_update(double (&q_orient)[4], double gyro[3], double accel[3], double mag[3], double dt)
{
    propagate(gyro, dt);
    time_update(dt);
    measurement_update(q_orient, accel, mag);
}

Vector4d MEKF::quat_multiply(Vector4d a, Vector4d b)
{
    Vector4d out;
    out << a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

    return out;
}

double MEKF::norm(double a, double b)
{
    double norm = sqrt(pow(a, 2) + pow(b, 2));

    return norm;
}

Matrix3d MEKF::skew(Vector3d x)
{
    Matrix3d out;
    out << 0, -x[2], x[1],
        x[2], 0, -x[0],
        -x[1], x[0], 0;

    return out;
}
