#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TransformStamped.h>
// #include <turtlesim/Pose.h>

tf2::Stamped<tf2::Transform> transformStamped;

void UpdateTransform(geometry_msgs::Accel accel, double dt)
{
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

    tf2::Quaternion rotation;
    const double degToRad = M_PI / 180.0;
    rotation.setRPY(
        degToRad * gx * dt,
        degToRad * gy * dt,
        degToRad * gz * dt);

    transformStamped.mult(transformStamped, tf2::Transform(rotation));
}

void computeBasicTransform(const geometry_msgs::AccelStamped& accel_msg){
    static tf2_ros::TransformBroadcaster br;
    double dt = (accel_msg.header.stamp - transformStamped.stamp_).toSec();

    if (dt < 0)
        ROS_ERROR_STREAM("Negative time difference");
    transformStamped.stamp_ = accel_msg.header.stamp;

    transformStamped.frame_id_ = "map";

    UpdateTransform(accel_msg.accel, dt);

    auto transform_msg = tf2::toMsg(transformStamped);
    transform_msg.child_frame_id = "basic_mpu";
    br.sendTransform(transform_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_mpu_transform");

    ros::NodeHandle node;

    tf2::Quaternion rotation = transformStamped.getRotation();
    transformStamped.setRotation(rotation);
 
    ros::Subscriber sub = node.subscribe("/mpu/acceleration", 1000, computeBasicTransform);
    ros::spin();
    return 0;
};
