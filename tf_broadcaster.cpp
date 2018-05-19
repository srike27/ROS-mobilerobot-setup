#include <ros/ros.h> // standard ros include
#include <tf/transform_broadcaster.h> 
//implementation of the tf:transform broadcaster to make it easier to do it

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher"); // create a node called robot_tf_publisher
  ros::NodeHandle n;  //create object n

  ros::Rate r(100); //set rate of publishing in hertz with contructor of object type Rate

  tf::TransformBroadcaster broadcaster; //create object broadcaster of TransformBroadcaster

	/* used to send the base_link --> base_laser transform  */

  while(n.ok()){

    broadcaster.sendTransform(
/*
	sending a transform message requires 5 arguments
		1) rotation transform specified by Quaternion constructed form pitch,roll,yaw,wrench
		2) Vector3 for translation
		3)a timestamp (eg  ros::Time::now)
		4)name of the parent node of the link
		5)name of the child node of the link
*/
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();  // used to make sure the rate at which data transfer occurs is maintained
  }
}

