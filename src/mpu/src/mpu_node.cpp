#include <ros/ros.h>

#include "std_msgs/String.h"

#include <sstream>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <turtlesim/Pose.h>

// std::string turtle_name;

// void poseCallback(const turtlesim::PoseConstPtr& msg){
//   static tf2_ros::TransformBroadcaster br;
//   geometry_msgs::TransformStamped transformStamped;

//   transformStamped.header.stamp = ros::Time::now();
//   transformStamped.header.frame_id = "world";
//   transformStamped.child_frame_id = turtle_name;
//   transformStamped.transform.translation.x = msg->x;
//   transformStamped.transform.translation.y = msg->y;
//   transformStamped.transform.translation.z = 0.0;
//   tf2::Quaternion q;
//   q.setRPY(0, 0, msg->theta);
//   transformStamped.transform.rotation.x = q.x();
//   transformStamped.transform.rotation.y = q.y();
//   transformStamped.transform.rotation.z = q.z();
//   transformStamped.transform.rotation.w = q.w();

//   br.sendTransform(transformStamped);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpu_tf2_broadcaster");

    ros::NodeHandle node;
    ros::Publisher chatter_pub = node.advertise<std_msgs::String>("pose", 1000);

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
};
