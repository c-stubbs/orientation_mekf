#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float64.hpp"

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
        _roll_pub = this->create_publisher<std_msgs::msg::Float64>("/roll",10);
        _pitch_pub = this->create_publisher<std_msgs::msg::Float64>("/pitch",10);
        _yaw_pub = this->create_publisher<std_msgs::msg::Float64>("/yaw",10);

        time = _clk.now();

        br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr _mag_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _roll_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pitch_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _yaw_pub;

    //MEKF mekf = MEKF(0.05, 0.05, 0.025);
    // MEKF mekf = MEKF(0.05,1.64,200);
     MEKF mekf = MEKF(0.05,0.167,200);
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

        double roll, pitch, yaw;
        roll = atan2(2*(mekf.q[0]*mekf.q[3] + mekf.q[1]*mekf.q[2]),pow(mekf.q[0],2) + pow(mekf.q[1],2) - pow(mekf.q[2],2) - pow(mekf.q[3],2));
        pitch = asin(2*(mekf.q[0]*mekf.q[2] - mekf.q[1]*mekf.q[3]));
        yaw = atan2(2*(mekf.q[0]*mekf.q[1] + mekf.q[2]*mekf.q[3]),pow(mekf.q[0],2) - pow(mekf.q[1],2) - pow(mekf.q[2],2) + pow(mekf.q[3],2));
        
        std_msgs::msg::Float64 roll_msg, pitch_msg, yaw_msg;
        roll_msg.data = roll;
        pitch_msg.data = pitch;
        yaw_msg.data = yaw;

        _yaw_pub->publish(roll_msg);
        _roll_pub->publish(pitch_msg);
        _pitch_pub->publish(yaw_msg);

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
