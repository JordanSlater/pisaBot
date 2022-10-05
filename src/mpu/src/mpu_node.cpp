#include <ros/ros.h>

#include <geometry_msgs/AccelStamped.h>

#include <sstream>

#include "mpu6050/mpu6050.h"

bool loadCalibrationData(const ros::NodeHandle & private_note, CalibrationData & calibration_data) {
    ROS_INFO("Waiting for MPU Calibration Data...");
    while (!private_note.hasParam("calibration")) {
        if (!ros::ok()) return false;
    }
    ROS_INFO("Found MPU Calibration Data.");

    double acceleration_sensitivity, gyro_sensitivity;
    float acceleration_offset_X;
    float acceleration_offset_Y;
    float acceleration_offset_Z;
    float gyro_offset_R;
    float gyro_offset_P;
    float gyro_offset_Y;
    
    if ( !(private_note.getParam("calibration/acceleration/sensitivity", acceleration_sensitivity)
        && private_note.getParam("calibration/gyro/sensitivity", gyro_sensitivity)
        && private_note.getParam("calibration/acceleration/offset/X", acceleration_offset_X)
        && private_note.getParam("calibration/acceleration/offset/Y", acceleration_offset_Y)
        && private_note.getParam("calibration/acceleration/offset/Z", acceleration_offset_Z)
        && private_note.getParam("calibration/gyro/offset/R", gyro_offset_R)
        && private_note.getParam("calibration/gyro/offset/P", gyro_offset_P)
        && private_note.getParam("calibration/gyro/offset/Y", gyro_offset_Y)))
    {
        ROS_ERROR("Could not get mpu calibration.");
        return false;
    }

	calibration_data.accel_range = acceleration_sensitivity;
    calibration_data.gyro_range = gyro_sensitivity;
	calibration_data.accel_offset_X = acceleration_offset_X;
    calibration_data.accel_offset_Y = acceleration_offset_Y;
    calibration_data.accel_offset_Z = acceleration_offset_Z;
    calibration_data.gyro_offset_X = gyro_offset_R;
    calibration_data.gyro_offset_Y = gyro_offset_P;
    calibration_data.gyro_offset_Z = gyro_offset_Y;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpu");
    ros::NodeHandle private_note("~");

    CalibrationData calibration_data;
    bool loaded = loadCalibrationData(private_note, calibration_data);
    if (!loaded) {
        return 0;
    }

    double update_rate;
    if (!private_note.getParam("update_rate", update_rate))
    {
        ROS_ERROR_STREAM("Could not load mpu update rate.");
        return 0;
    }

    MPU6050 mpu_device(0x68, calibration_data, false);

    ros::Publisher acceleration_pub = private_note.advertise<geometry_msgs::AccelStamped>("acceleration", 1000);

    ros::Rate loop_rate(update_rate);

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
