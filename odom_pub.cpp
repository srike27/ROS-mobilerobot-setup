/* add these to package manifest

<depend package="tf"/>
<depend package="nav_msgs"/>
*/

#include <ros/ros.h>
/*
Since we're going to be publishing both a transform from the "odom" coordinate frame to the "base_link" coordinate frame and a nav_msgs/Odometry message, we need to include the relevant header files. 
*/
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
/*
We need to create both a ros::Publisher and a tf::TransformBroadcaster to be able to send messages out using ROS and tf respectively.
*/
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
//We'll assume that the robot starts at the origin of the "odom" coordinate frame initially.
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
/*
Here we'll set some velocities that will cause the "base_link" frame to move in the "odom" frame at a rate of 0.1m/s in the x direction, -0.1m/s in the y direction, and 0.1rad/s in the th direction. This will more or less cause our fake robot to drive in a circle. 
*/
  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
/*
We'll publish odometry information at a rate of 1Hz in this example to make introspection easy, most systems will want to publish odometry at a much higher rate. 
*/
  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
/*
Here we'll update our odometry information based on the constant velocities we set. A real odometry system would, of course, integrate computed velocities instead.
*/
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
