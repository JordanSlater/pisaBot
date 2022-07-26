#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

inline tf2::Vector3 projectOntoPlane(tf2::Vector3 vector, tf2::Vector3 planeNormal) {
    return vector - (planeNormal.dot(vector)) * planeNormal / planeNormal.length2();
}

static float getAngleInDegrees(const geometry_msgs::TransformStamped transform) {
    tf2::Stamped<tf2::Transform> stampedTransform;
    tf2::fromMsg(transform, stampedTransform);

    constexpr size_t Y_AXIS = 1;
    tf2::Vector3 wheelAxis = stampedTransform.getBasis().getColumn(Y_AXIS);

    constexpr size_t Z_AXIS = 2;
    tf2::Vector3 robotUp = stampedTransform.getBasis().getColumn(Z_AXIS);
    tf2::Vector3 globalUp{0, 0, 1};
    tf2::Vector3 targetVector = projectOntoPlane(globalUp, wheelAxis).normalize();

    float angle = acos(targetVector.dot(robotUp));
    int sign = (targetVector.cross(robotUp)).dot(wheelAxis) > 0 ? 1 : -1;

    return sign * angle * 180.0 / M_PI;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10); // Hz
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("map", "body", ros::Time(0));
            float angle = getAngleInDegrees(transformStamped);
            ROS_INFO_STREAM("Angle: " << angle);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
    }
    return 0;
};
