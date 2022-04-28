#include <ros/ros.h>

#include <geometry_msgs/AccelStamped.h>

#include <sstream>

#include "mpu6050/mpu6050.h"

MPU6050 mpu_device(0x68, false);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpu");

    ros::NodeHandle private_note("~");
    ros::Publisher acceleration_pub = private_note.advertise<geometry_msgs::AccelStamped>("acceleration", 1000);

    ros::Rate loop_rate(10);

    float ax, ay, az;
    float gx, gy, gz;

    while (ros::ok())
    {
        mpu_device.manual_update();
        
        geometry_msgs::AccelStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "mpu";

        mpu_device.getAccel(&ax, &ay, &az);
        msg.accel.linear.x = ax;
        msg.accel.linear.y = ay;
        msg.accel.linear.z = az;

        mpu_device.getGyro(&gx, &gy, &gz);
        msg.accel.angular.x = gx;
        msg.accel.angular.y = gy;
        msg.accel.angular.z = gz;

        acceleration_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
};
