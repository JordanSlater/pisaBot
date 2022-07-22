#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

tf2::Stamped<tf2::Transform> stampedMpuTransform;
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

    tf2::Quaternion rotation;
    const double degToRad = M_PI / 180.0;
    rotation.setRPY(
        degToRad * gx * dt,
        degToRad * gy * dt,
        degToRad * gz * dt);

    stampedMpuTransform.mult(stampedMpuTransform, tf2::Transform(rotation));
    ROS_INFO_STREAM("dt: " << dt);
}

void accelCallback(const geometry_msgs::AccelStamped& accel_msg){
    static tf2_ros::TransformBroadcaster br;
    double dt = (accel_msg.header.stamp - stampedMpuTransform.stamp_).toSec();

    if (dt < 0)
        ROS_ERROR_STREAM("accel_msg.header.stamp: " << accel_msg.header.stamp << ", stampedMpuTransform.stamp_: " << stampedMpuTransform.stamp_);

    UpdateTransform(accel_msg.accel, dt);

    stampedMpuTransform.stamp_ = accel_msg.header.stamp;

    tf2::Stamped<tf2::Transform> stampedBodyTransform = stampedMpuTransform;

    auto transform_msg = tf2::toMsg(stampedBodyTransform);
    transform_msg.child_frame_id = "mpu";
    br.sendTransform(transform_msg);
}

bool getTree(ros::NodeHandle & node, KDL::Tree & tree)
{
    std::string robot_description;
    node.param("robot_description", robot_description, std::string());
    if (!kdl_parser::treeFromString(robot_description, tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    return true;
}

tf2::Transform GetBodyToImuTransform()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ROS_INFO_STREAM("Getting mpu transform...");
    tf2::Transform bodyToMpu;
    geometry_msgs::TransformStamped stampedBodyToMpuMsg = tfBuffer.lookupTransform("body", "mpu", ros::Time(10));
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

    KDL::Tree tree;
    if (!getTree(private_node, tree))
        return -1;

    robot_state_publisher::RobotStatePublisher robotStatePublisher(tree);
    robotStatePublisher.publishFixedTransforms();

    bodyToMpu = GetBodyToImuTransform();

    tf2::Quaternion rotation = loadInitialRotation(private_node);
    stampedMpuTransform.setRotation(rotation);
    stampedMpuTransform.frame_id_ = "map";

    ros::Subscriber sub = node.subscribe("/mpu/acceleration", 1000, accelCallback);
    ros::spin();
    return 0;
};
