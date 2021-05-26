#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TransformStamped.h>
// #include <turtlesim/Pose.h>


ros::Time prev_accel_msg_time;
geometry_msgs::TransformStamped transformStamped;

void accelCallback(const geometry_msgs::AccelStamped& msg){
    static tf2_ros::TransformBroadcaster br;
    double dt = (msg.header.stamp - prev_accel_msg_time).toSec();

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "mpu2";

    transformStamped.transform.translation.x += msg.accel.linear.x * dt * dt;
    transformStamped.transform.translation.y += msg.accel.linear.y * dt * dt;
    transformStamped.transform.translation.z += msg.accel.linear.z * dt * dt;

    tf2::Vector3 vector;
    tf2::fromMsg(transformStamped.transform.translation, vector);

    if (vector.length() > 1)
    {
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;
    }

    // tf2::Quaternion q;
    // q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = 0.0;// q.x();
    transformStamped.transform.rotation.y = 0.0;// q.y();
    transformStamped.transform.rotation.z = 0.0;// q.z();
    transformStamped.transform.rotation.w = 1.0;// q.w();

    ROS_INFO_STREAM("sent transform");
    br.sendTransform(transformStamped);
    prev_accel_msg_time = msg.header.stamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accel_tracker");

    ros::NodeHandle node;

    prev_accel_msg_time = ros::Time::now();
    ros::Subscriber sub = node.subscribe("/mpu/acceleration", 1000, accelCallback);
    ros::spin();
    return 0;
};
