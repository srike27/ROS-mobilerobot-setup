#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>  // to make sure we can use the data type of geometry_msgs PointStamped
#include <tf/transform_listener.h>  //create a tf::transform listener so that we can automatically subscribe to the transform message

void transformPoint(const tf::TransformListener& listener){
/*
we'll create a point in the base_laser frame that we'd like to transform to the base_link frame. 
This function also acts as a call back for ros::Timer created in the main() of our program and will fire every round
*/
  geometry_msgs::PointStamped laser_point; //create the object


  laser_point.header.frame_id = "base_laser";
  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();
  //these two allows us to associate timestamp and a frame_id with the message

  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
/*

	we call the transformPoint() fuction and give it 3 arguments.
	frame_id
	point that we are transforming
	storage of the transformed point

now base_point holds the same information as laser_point but in base_link frame

*/

catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();// make sure it is synchronous

}
