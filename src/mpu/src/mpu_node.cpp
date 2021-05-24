#include <ros/ros.h>

#include <geometry_msgs/AccelStamped.h>

#include <sstream>

#include <MPU6050.h>

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

MPU6050 mpu_device(0x68, false);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpu");

    ros::NodeHandle private_note("~");
    ros::Publisher acceleration_pub = private_note.advertise<geometry_msgs::AccelStamped>("acceleration", 1000);

    ros::Rate loop_rate(10);

    float ax, ay, az;
    float gx, gy, gz;

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
        mpu_device.update();
        
        geometry_msgs::AccelStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "mpu";

        mpu_device.getAccel(&ax, &ay, &az);
        msg.accel.linear.x = ax;
        msg.accel.linear.y = ay;
        msg.accel.linear.z = az;

        mpu_device.getGyro(&gx, &gy, &gz);
        msg.accel.angular.x = gx;
        msg.accel.angular.y = gy;
        msg.accel.angular.z = gz;

        acceleration_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
};
