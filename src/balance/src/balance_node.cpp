#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <memory>

std::unique_ptr<ros::Publisher> vectorPub;

inline tf2::Vector3 projectOntoPlane(tf2::Vector3 vector, tf2::Vector3 planeNormal) {
    return vector - (planeNormal.dot(vector)) * planeNormal / planeNormal.length2();
}

static float getErrorInDegrees(const geometry_msgs::TransformStamped transform) {
    tf2::Stamped<tf2::Transform> stampedTransform;
    tf2::fromMsg(transform, stampedTransform);

    constexpr size_t Y_AXIS = 1;
    tf2::Vector3 wheelAxis = stampedTransform.getBasis().getColumn(Y_AXIS);

    constexpr size_t Z_AXIS = 2;
    tf2::Vector3 robotDown = -stampedTransform.getBasis().getColumn(Z_AXIS);
    tf2::Vector3 globalDown{0, 0, -1};
    tf2::Vector3 targetDownVector = projectOntoPlane(globalDown, wheelAxis).normalize();


    auto displayVector = targetDownVector;
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time();
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    constexpr double distance = 0.4;
    marker.pose.position.x = displayVector.getX() * distance;
    marker.pose.position.y = displayVector.getY() * distance;
    marker.pose.position.z = displayVector.getZ() * distance;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;

    vectorPub->publish(marker);

    /*
    What I want to do is get the plane perpendicular to the y basis vector.
    Then I want to get the vector closest to the down vector.
    Then I want to get the difference between this new in plane down vector and the down vector in the imu frame (in this case the x basis vector).
    */
    return 0.0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    vectorPub = std::make_unique<ros::Publisher>(node.advertise<visualization_msgs::Marker>("display", 1000));

    ros::Rate rate(10); // Hz
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("map", "body", ros::Time(0));
            getErrorInDegrees(transformStamped);
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
