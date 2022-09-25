#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>

inline tf2::Vector3 projectOntoPlane(tf2::Vector3 vector, tf2::Vector3 planeNormal) {
    return vector - (planeNormal.dot(vector)) * planeNormal / planeNormal.length2();
}

static double getAngleInDegrees(const geometry_msgs::TransformStamped transform) {
    tf2::Stamped<tf2::Transform> stampedTransform;
    tf2::fromMsg(transform, stampedTransform);

    constexpr size_t Y_AXIS = 1;
    tf2::Vector3 wheelAxis = stampedTransform.getBasis().getColumn(Y_AXIS);

    constexpr size_t Z_AXIS = 2;
    tf2::Vector3 robotUp = stampedTransform.getBasis().getColumn(Z_AXIS);
    tf2::Vector3 globalUp{0, 0, 1};
    tf2::Vector3 targetVector = projectOntoPlane(globalUp, wheelAxis).normalize();

    double angle = acos(targetVector.dot(robotUp));
    int sign = (targetVector.cross(robotUp)).dot(wheelAxis) > 0 ? 1 : -1;

    return sign * angle * 180.0 / M_PI;
}

void balance(double angle) {
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    double targetAngleInDegrees;
    node.param("target_angle_degrees", targetAngleInDegrees, 0.0);
    ROS_INFO_STREAM("Loaded target_angle_degrees = " << targetAngleInDegrees);
    
    bool balancing = false;
    bool wasBalancing = false;

    ros::Publisher balancingPublisher = node.advertise<std_msgs::Bool>("balancing", 100);
    ros::Publisher emergencyStopPublisher = node.advertise<std_msgs::Empty>("emergency_stop", 100);
    ros::Publisher errorAnglePublisher = node.advertise<std_msgs::Float64>("angle_error", 100);
    ros::Publisher setPointAnglePublisher = node.advertise<std_msgs::Float64>("angle_setpoint", 100);

    ros::Rate rate(10); // Hz
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("map", "body", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        double angle = getAngleInDegrees(transformStamped);
        ROS_INFO_STREAM("Angle: " << angle);

        if (fabs(angle - targetAngleInDegrees) < 10)
            balancing = true;
        else if (fabs(angle - targetAngleInDegrees) > 45)
            balancing = false;

        std_msgs::Bool balancingMsg;
        balancingMsg.data = balancing;
        balancingPublisher.publish((balancingMsg));

        std_msgs::Float64 setPointAngleMsg;
        setPointAngleMsg.data = targetAngleInDegrees;
        setPointAnglePublisher.publish(setPointAngleMsg);

        if (balancing) {
            std_msgs::Float64 errorAngleMsg;
            errorAngleMsg.data = angle;
            errorAnglePublisher.publish(errorAngleMsg);
        }
        if (!balancing && wasBalancing) {
            std_msgs::Empty emptyMsg;
            emergencyStopPublisher.publish(emptyMsg);
        }
        wasBalancing = balancing;

        rate.sleep();
    }
    return 0;
};
