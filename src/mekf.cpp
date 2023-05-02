// Multiplicative Extended Kalman Filter (MEKF) Class Implementation

#include "mekf/mekf.h"

MEKF::MEKF(double gyro_noise, double accel_noise, double mag_noise)
{
    // Noise Initialization
    g_noise = pow(gyro_noise, 2);
    a_noise = pow(accel_noise, 2);
    m_noise = pow(mag_noise, 2);

    proc_noise << g_noise, g_noise, g_noise; // noise vectors
    meas_noise << a_noise, a_noise, a_noise, m_noise, m_noise, m_noise;

    // Global Paramter Initialization
    g << 0, 0, -1; //initialize gravity **CHANGED FROM 9.81
    // mag_ref << 23161.2, -1862.9, 41935.2; // Computed prior, based on the AHRS github
    mag_ref << -46.588, 65.3055, 7.6486; // From WMM
    mag_ref = mag_ref.normalized();
    // Filter Initialization
    q << 1, 0, 0, 0; // initialize quaternion

    Q = proc_noise.asDiagonal(); // initialize process & measurement noise covariance
    R = meas_noise.asDiagonal();

    // W.setIdentity();
    // P = W * Q * W.transpose();
    P = Q; // Initialize covariance

    // magcal_offset << -16.8557, 48.4432, 25.2809; // From MATLAB
    // magcal_rotate.setIdentity(); // From MATLAB

    magcal_offset << -42.8444, 72.2708, -38.5865;
    Vector3d magcal_rotate_vals;
    magcal_rotate_vals << 1.0773, 0.8575, 1.0826;
    magcal_rotate = magcal_rotate_vals.asDiagonal();
}

void MEKF::predict(double gyro[3], double dt)
{
    Vector3d omega;
    omega << gyro[0], gyro[1], gyro[2];
    /* Eq 2.82 in Howard's Thesis: First order approximation of the quaternion state
        transition matrix. 
        See Also: Eq 20 in Utrera */
    A(0, 0) = 1.0;
    A(0, 1) = -0.5 * omega[0] * dt;
    A(0, 2) = -0.5 * omega[1] * dt;
    A(0, 3) = -0.5 * omega[2] * dt;

    A(1, 0) = 0.5 * omega[0] * dt;
    A(1, 1) = 1.0;
    A(1, 2) = 0.5 * omega[2] * dt;
    A(1, 3) = -0.5 * omega[1] * dt;

    A(2, 0) = 0.5 * omega[1] * dt;
    A(2, 1) = -0.5 * omega[2] * dt;
    A(2, 2) = 1.0;
    A(2, 3) = 0.5 * omega[0] * dt;

    A(3, 0) = 0.5 * omega[2] * dt;
    A(3, 1) = 0.5 * omega[1] * dt;
    A(3, 2) = -0.5 * omega[0] * dt;
    A(3, 3) = 1.0;

    // Integrate the quaternion rate of change to get the propogated quaternion
    q = A * q;
    // Update covariance estimate, Eq 21 in Utrera
    P = P + (-1.0*skew(omega)*P + P*skew(omega) + Q)*dt; 
}

void MEKF::update(double (&q_orient)[4], double accel[3], double mag[3])
{
    a << accel[0], accel[1], accel[2];
    m << mag[0], mag[1], mag[2];
    // m = ((m-magcal_offset).transpose()*magcal_rotate).transpose(); // Calibrate the mag readings
    m = mag_ref; // Setting this to ignore mag measurements. RIP :(

    // Eq 2.33 in Howard's Thesis: Basically just from global/intertial frame to body frame
    C(0, 0) = pow(q(0), 2) + pow(q(1), 2) - pow(q(2), 2) - pow(q(3), 2);
    C(0, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
    C(0, 2) = 2 * (q(0) * q(2) + q(1) * q(3));

    C(1, 0) = 2 * (q(0) * q(3) + q(1) * q(2));
    C(1, 1) = pow(q(0), 2) - pow(q(1), 2) + pow(q(2), 2) - pow(q(3), 2);
    C(1, 2) = 2 * (q(2) * q(3) - q(0) * q(1));

    C(2, 0) = 2 * (q(1) * q(3) - q(0) * q(2));
    C(2, 1) = 2 * (q(0) * q(1) + q(2) * q(3));
    C(2, 2) = pow(q(0), 2) - pow(q(1), 2) - pow(q(2), 2) + pow(q(3), 2);

    // Measurements
    z<< a.normalized(),
        m.normalized();

    // printf("Mags: %4.2lf | %4.2lf | %4.2lf",z[3],z[4],z[5]);

    // Predict what the mag and acc measurments will be based on the current estimate of orientation
    h << C.transpose() * g,
        C.transpose() * mag_ref;

    y = z - h;

    // From Markley 2003, Utrera: measurment sensitivity matrix, calculated as follows
    H << C.transpose() * skew(g),
        C.transpose() * skew(mag_ref);

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

    // Eq 27 in Utrera
    P = P - K*H*P;
}

void MEKF::filter_update(double (&q_orient)[4], double gyro[3], double accel[3], double mag[3], double dt)
{
    predict(gyro, dt);
    // time_update(dt);
    update(q_orient, accel, mag);
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

Matrix3d MEKF::skew(Vector3d x)
{
    Matrix3d out;
    out << 0, -x[2], x[1],
        x[2], 0, -x[0],
        -x[1], x[0], 0;

    return out;
}
