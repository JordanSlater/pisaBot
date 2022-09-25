#include <memory>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TransformStamped.h>

std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;
tf2::Stamped<tf2::Transform> robotLocation{};
tf2::Transform bodyToMpu;

tf2::Quaternion loadInitialRotation(
    const ros::NodeHandle & private_node)
{
    tf2::Quaternion rotation;
    double r, p, y;
    private_node.param("starting_orientation/roll", r, 0.0);
    private_node.param("starting_orientation/pitch", p, 0.0);
    private_node.param("starting_orientation/yaw", y, 0.0);
    const double degrees_to_rad = M_PI / 180.0;
    rotation.setRPY(
        r * degrees_to_rad,
        p * degrees_to_rad,
        y * degrees_to_rad);
    ROS_INFO_STREAM("Set initial rotation to: " << r << ", " << p << ", " << y);
    return rotation;
}

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

    constexpr double degToRad = M_PI / 180.0;
    tf2::Quaternion rotation;
    rotation.setRPY(
        degToRad * gx * dt,
        degToRad * gy * dt,
        degToRad * gz * dt);

    // Puts the mpu rotation in the body's frame before rotating the robot location.
    robotLocation *= tf2::Transform(bodyToMpu.inverse().getRotation() * rotation * bodyToMpu.getRotation());
}

double getChangeInTimeSinceLastUpdate(const ros::Time & newStamp) {
    double dt = (newStamp - robotLocation.stamp_).toSec();

    if (dt < 0)
        ROS_ERROR_STREAM("Calculated negative change in time. New time stamp: " << newStamp << ", previous time stamp: " << robotLocation.stamp_);

    return dt;
}

void accelCallback(const geometry_msgs::AccelStamped& accel_msg){

    double dt = getChangeInTimeSinceLastUpdate(accel_msg.header.stamp);
    robotLocation.stamp_ = accel_msg.header.stamp;
    UpdateTransform(accel_msg.accel, dt);

    geometry_msgs::TransformStamped transformMsg = tf2::toMsg(robotLocation);
    transformMsg.child_frame_id = "body";
    transformBroadcaster->sendTransform(transformMsg);
}

tf2::Transform GetBodyToMpuTransform()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ROS_INFO_STREAM("Getting mpu transform...");
    tf2::Transform bodyToMpu;
    geometry_msgs::TransformStamped stampedBodyToMpuMsg = tfBuffer.lookupTransform("body", "mpu", ros::Time(0), ros::Duration(20));
    geometry_msgs::Transform bodyToMpuMsg = stampedBodyToMpuMsg.transform;
    tf2::fromMsg(bodyToMpuMsg, bodyToMpu);
    ROS_INFO_STREAM("Done");
    return bodyToMpu;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle private_node("~");
    ros::NodeHandle node;

    robotLocation.setIdentity();
    tf2::Quaternion rotation = loadInitialRotation(private_node);
    robotLocation.setRotation(rotation);
    robotLocation.frame_id_ = "map";

    transformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();

    bodyToMpu = GetBodyToMpuTransform();

    ros::Subscriber sub = node.subscribe("/mpu/acceleration", 1000, accelCallback);

    ros::spin();
    return 0;
};
