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

void UpdateTransform(geometry_msgs::Accel accel, double dt)
{
    // this was tuned when dt was roughly 0.054 to 0.066
    const double alpha = 1.0 + 0 *0.94;

    static double gx, gy, gz;
    static bool first = true;
    if (first)
    {
        gx = accel.angular.x;
        gy = accel.angular.y;
        gz = accel.angular.z;
        first = false;
        return;
    } else {
        gx = gx * (1 - alpha) + accel.angular.x * alpha;
        gy = gy * (1 - alpha) + accel.angular.y * alpha;
        gz = gz * (1 - alpha) + accel.angular.z * alpha;
    }

    // ROS_INFO_STREAM("\n gx: " << gx << "\n gy: " << gy << "\n gz: " << gz);

    // transformStamped.setOrigin(tf2::Vector3(0, 0, 0));
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(transformStamped.getRotation()).getRPY(roll, pitch, yaw);
    // tf2::Quaternion new_rotation;
    // const double IDKWHY = 1.0;
    // const double degToRad = M_PI / 180.0;
    // new_rotation.setRPY(
    //     roll + degToRad * gx * dt * IDKWHY,
    //     pitch + degToRad * gy * dt * IDKWHY,
    //     yaw + degToRad * gz * dt * IDKWHY);


    tf2::Quaternion rotation;
    const double degToRad = M_PI / 180.0;
    rotation.setRPY(
        degToRad * gx * dt,
        degToRad * gy * dt,
        degToRad * gz * dt);

    transformStamped.mult(transformStamped, tf2::Transform(rotation));
    ROS_INFO_STREAM("dt: " << dt);
    // transformStamped.setRotation(new_rotation);
}

void accelCallback(const geometry_msgs::AccelStamped& accel_msg){
    static tf2_ros::TransformBroadcaster br;
    double dt = (accel_msg.header.stamp - transformStamped.stamp_).toSec();

    // static bool first = true;
    // if (first)
    // {
    //     first = false;
    // }
    if (dt < 0)
        ROS_ERROR_STREAM("accel_msg.header.stamp: " << accel_msg.header.stamp << ", transformStamped.stamp_: " << transformStamped.stamp_);
    transformStamped.stamp_ = accel_msg.header.stamp;

    transformStamped.frame_id_ = "map";

    UpdateTransform(accel_msg.accel, dt);

    auto transform_msg = tf2::toMsg(transformStamped);
    transform_msg.child_frame_id = "mpu";
    br.sendTransform(transform_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accel_tracker");

    ros::NodeHandle node;

    tf2::Quaternion rotation = transformStamped.getRotation();
    rotation.setRPY(0, 0, 0.75 * M_PI);
    transformStamped.setRotation(rotation);
 
    ros::Subscriber sub = node.subscribe("/mpu/acceleration", 1000, accelCallback);
    ros::spin();
    return 0;
};
