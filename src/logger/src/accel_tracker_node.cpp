#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TransformStamped.h>
// #include <turtlesim/Pose.h>


// ros::Time prev_accel_msg_time;
// geometry_msgs::TransformStamped transformStamped;
tf2::Stamped<tf2::Transform> transformStamped;

void accelCallback(const geometry_msgs::AccelStamped& accel_msg){
    static tf2_ros::TransformBroadcaster br;
    double dt = (accel_msg.header.stamp - transformStamped.stamp_).toSec();

    transformStamped.stamp_ = ros::Time::now();
    transformStamped.frame_id_ = "map";
    // transformStamped.child_frame_id = "mpu2";

    transformStamped.setOrigin(tf2::Vector3(0, 0, 0));
    // transformStamped.transform.translation.x += accel_msg.accel.linear.x * dt * dt;
    // transformStamped.transform.translation.y += accel_msg.accel.linear.y * dt * dt;
    // // subtract one g of gravity
    // transformStamped.transform.translation.z += (accel_msg.accel.linear.z - 1) * dt * dt;

    // tf2::Vector3 translation;
    // tf2::fromMsg(transformStamped.transform.translation, translation);

    // if (translation.length() > 1)
    // {
    //     transformStamped.transform.translation.x = 0;
    //     transformStamped.transform.translation.y = 0;
    //     transformStamped.transform.translation.z = 0;
    // }

    double roll, pitch, yaw;
    tf2::Matrix3x3(transformStamped.getRotation()).getRPY(roll, pitch, yaw);
    tf2::Quaternion new_rotation;
    new_rotation.setRPY(
        roll + accel_msg.accel.angular.x * dt, 
        pitch + accel_msg.accel.angular.y * dt,
        yaw + accel_msg.accel.angular.z * dt);
    transformStamped.setRotation(new_rotation);

    ROS_INFO_STREAM("sent transform");
    auto transform_msg = tf2::toMsg(transformStamped);
    transform_msg.child_frame_id = "mpu2";
    br.sendTransform(transform_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accel_tracker");

    ros::NodeHandle node;

    transformStamped.stamp_ = ros::Time::now();
    ros::Subscriber sub = node.subscribe("/mpu/acceleration", 1000, accelCallback);
    ros::spin();
    return 0;
};
