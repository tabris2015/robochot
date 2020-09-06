#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

//
tf::TransformListener listener;
nav_msgs::Odometry odom_msg;
//


void cCb(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("hola");
}

void twistCb(const geometry_msgs::TwistStamped::ConstPtr& twist_msg)
{
    double r, p ,y;
    tf::StampedTransform t;
    
    try
    {
        listener.lookupTransform("/base_link", "/odom", ros::Time(0), t);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    
    odom_msg.header.stamp = twist_msg->header.stamp;
    odom_msg.header.frame_id = "/odom";
    odom_msg.child_frame_id = "/base_link";
    odom_msg.pose.pose.position.x = t.getOrigin().x();
    odom_msg.pose.pose.position.y = t.getOrigin().y();
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.w = t.getRotation().getW();
    odom_msg.pose.pose.orientation.x = t.getRotation().getX();
    odom_msg.pose.pose.orientation.y = t.getRotation().getY();
    odom_msg.pose.pose.orientation.z = t.getRotation().getZ();
    odom_msg.twist.twist.linear.x = twist_msg->twist.linear.x;
    odom_msg.twist.twist.angular.z = twist_msg->twist.angular.z;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robochot_base");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("chatter", 1000, cCb);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    // ros::spin();
    ros::Rate r(50);
    while(nh.ok())
    {
        ros::spinOnce();
        odom_pub.publish(odom_msg);
        r.sleep();
    }

    return 0;
}