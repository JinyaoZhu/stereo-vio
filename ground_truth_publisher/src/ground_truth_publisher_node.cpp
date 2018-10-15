#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

ros::Publisher mocap_odom_pub;


// T_BS:
//   cols: 4
//   rows: 4
//   data: [ 0.33638, -0.01749,  0.94156,  0.06901,
//          -0.02078, -0.99972, -0.01114, -0.02781,
//           0.94150, -0.01582, -0.33665, -0.12395,
//               0.0,      0.0,      0.0,      1.0]
// x y z w
//    0.8174279 
//   -0.0117032
//    0.5759101
//   -0.0014282

void mocap_callback(const geometry_msgs::TransformStampedConstPtr &msg)
{
    static tf::Transform tf1, tf2;
    static tf::Transform tf_body_vicon; //Sensor extrinsics wrt. the body-frame.
    static tf::TransformBroadcaster tf_pub;

    tf1.setOrigin(tf::Vector3(msg->transform.translation.x, msg->transform.translation.y,msg->transform.translation.z));
    tf1.setRotation(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w));


    tf_body_vicon.setOrigin(tf::Vector3(0.06901,-0.02781,-0.12395));
    tf_body_vicon.setRotation(tf::Quaternion(0.8174279,-0.0117032,0.5759101,-0.0014282));

    // inv_tf1 = tf1.inverse();
    tf2 = tf1*tf_body_vicon.inverse();

    nav_msgs::Odometry odometry;
    odometry.header.stamp = msg->header.stamp;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "vicon_body_odom";

    odometry.pose.pose.position.x = tf2.getOrigin().getX();
    odometry.pose.pose.position.y = tf2.getOrigin().getY();
    odometry.pose.pose.position.z = tf2.getOrigin().getZ();
    odometry.pose.pose.orientation.w = tf2.getRotation().getW();
    odometry.pose.pose.orientation.x = tf2.getRotation().getX();
    odometry.pose.pose.orientation.y = tf2.getRotation().getY();
    odometry.pose.pose.orientation.z = tf2.getRotation().getZ();

    tf_pub.sendTransform(tf::StampedTransform(
          tf1, msg->header.stamp, "world", "vicon"));

    tf_pub.sendTransform(tf::StampedTransform(
          tf2, msg->header.stamp, "world", "vicon_body"));

    mocap_odom_pub.publish(odometry);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_truth_publisher");
    ros::NodeHandle nh("~");

    mocap_odom_pub = nh.advertise<nav_msgs::Odometry>("mocap_odom", 10);

    ros::Subscriber sub_odom = nh.subscribe("mocap_pose", 100, mocap_callback);

    ros::spin();
}
