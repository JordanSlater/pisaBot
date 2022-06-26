#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static float getErrorInDegrees(const geometry_msgs::TransformStamped transform) {
    tf2::Stamped<tf2::Transform> stampedTransform;
    tf2::fromMsg(transform, stampedTransform);
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

    ros::Rate rate(10); // Hz
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("mpu", "map", ros::Time(0));
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
