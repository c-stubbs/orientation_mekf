#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "mekf/mekf.h"
using std::placeholders::_1;

class OrientationNode : public rclcpp::Node
{
public:
    OrientationNode() : Node("orientation_mekf")
    {
        _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&OrientationNode::imu_callback, this, _1));
        _mag_sub = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/mag", 10, std::bind(&OrientationNode::mag_callback, this, _1));

        time = _clk.now();

        br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr _mag_sub;

    //MEKF mekf = MEKF(0.05, 0.05, 0.025);
    MEKF mekf = MEKF(0.05,0.1,1.5);
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;

    double accel[3] = {0.1, 0.1, 0.1};
    double gyro[3] = {0.1, 0.1, 0.1};
    double quat[4] = {1, 0.0, 0.0, 0.0};

    rclcpp::Clock _clk;
    rclcpp::Time time;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        accel[0] = msg->linear_acceleration.x;
        accel[1] = msg->linear_acceleration.y;
        accel[2] = msg->linear_acceleration.z;

        gyro[0] = msg->angular_velocity.x;
        gyro[1] = msg->angular_velocity.y;
        gyro[2] = msg->angular_velocity.z;
    }

    void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
    {
        double dt = (_clk.now() - time).nanoseconds() / 1e9;
        time = _clk.now();
        RCLCPP_INFO_THROTTLE(this->get_logger(),_clk,2000, "Filter Timestep: %4.2lf", dt);

        double mag[3] = {msg->magnetic_field.x,
                         msg->magnetic_field.y,
                         msg->magnetic_field.z};

        mekf.filter_update(quat, gyro, accel, mag, dt);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.child_frame_id = "torso";
        tf_msg.header.frame_id = "world";
        tf_msg.header.stamp = _clk.now();
        tf_msg.transform.rotation.w = mekf.q[0];
        tf_msg.transform.rotation.x = mekf.q[1];
        tf_msg.transform.rotation.y = mekf.q[2];
        tf_msg.transform.rotation.z = mekf.q[3];

        br->sendTransform(tf_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrientationNode>());
    rclcpp::shutdown();
    return 0;
}
