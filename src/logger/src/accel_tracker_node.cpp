#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TransformStamped.h>
// #include <turtlesim/Pose.h>


ros::Time prev_accel_msg_time;
tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped transformStamped;

void accelCallback(const geometry_msgs::AccelStamped& msg){
    double dt = (msg.header.stamp - prev_accel_msg_time).toSec();

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = msg.header.frame_id;
    transformStamped.child_frame_id = "mpu";

    transformStamped.transform.translation.x += msg.accel.linear.x * dt * dt;
    transformStamped.transform.translation.y += msg.accel.linear.y * dt * dt;
    transformStamped.transform.translation.z += msg.accel.linear.z * dt * dt;
    // tf2::Quaternion q;
    // q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = 0.0;// q.x();
    transformStamped.transform.rotation.y = 0.0;// q.y();
    transformStamped.transform.rotation.z = 0.0;// q.z();
    transformStamped.transform.rotation.w = 1.0;// q.w();

    br.sendTransform(transformStamped);
    prev_accel_msg_time = msg.header.stamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpu");

    ros::NodeHandle private_note("~");

    prev_accel_msg_time = ros::Time::now();

    private_note.subscribe<geometry_msgs::AccelStamped>("mpu/acceleration", 1000, &accelCallback);
    ros::spin();
    return 0;
};
